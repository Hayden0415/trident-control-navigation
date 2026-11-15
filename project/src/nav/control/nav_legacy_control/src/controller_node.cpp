#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <queue>
#include <cmath>
#include <algorithm>
#include "nav_legacy_control/legacy_pid.hpp"
class Controller : public rclcpp::Node {
    using Grid=nav_msgs::msg::OccupancyGrid;
    using Odom=nav_msgs::msg::Odometry;
    using Pose=geometry_msgs::msg::PoseStamped;
public:
    Controller():Node("legacy_controller") {
        speed_=declare_parameter("max_speed",.65);accel_=declare_parameter("max_acceleration",1.);
        timeout_=declare_parameter("input_timeout",1.);
        if(speed_<=0||accel_<=0||timeout_<=0)throw std::invalid_argument("limits must be positive");
        map_sub_=create_subscription<Grid>("/nav/map",rclcpp::QoS(1).reliable().transient_local(),
            [this](Grid::ConstSharedPtr m){if(valid(*m))grid_=m;else {grid_.reset();route_.clear();}});
        odom_sub_=create_subscription<Odom>("/nav/odom",rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr m){odom_=m;});
        goal_sub_=create_subscription<Pose>("/nav/goal",10,[this](Pose::ConstSharedPtr m){goal_=m;route_.clear();path_.reset();pid_.reset();arrived_=false;});
        path_sub_=create_subscription<nav_msgs::msg::Path>("/nav/planning/path",10,[this](nav_msgs::msg::Path::ConstSharedPtr m){
            path_=m;route_.clear();
            for(const auto& p:m->poses) {
                if(!std::isfinite(p.pose.position.x+p.pose.position.y)){path_.reset();route_.clear();return;}
                route_.emplace_back(p.pose.position.x,p.pose.position.y);
            }
        });
        cmd_pub_=create_publisher<geometry_msgs::msg::Twist>("/nav/cmd_vel",10);
        status_pub_=create_publisher<std_msgs::msg::String>("/nav/control/status",1);
        timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    }
private:
    bool valid(const Grid& m) const {
        const auto& i=m.info;
        const auto& q=i.origin.orientation;
        return m.header.frame_id=="map" && i.resolution>0 && std::isfinite(i.resolution) &&
            i.width>0 && i.height>0 && uint64_t(i.width)*i.height<=250000 &&
            m.data.size()==uint64_t(i.width)*i.height && std::isfinite(i.origin.position.x) &&
            std::isfinite(i.origin.position.y) && std::abs(q.x)<1e-6 && std::abs(q.y)<1e-6 &&
            std::abs(q.z)<1e-6 && std::abs(std::abs(q.w)-1)<1e-6 &&
            std::all_of(m.data.begin(),m.data.end(),[](int8_t v){return v==0||v==100||v==-1;});
    }
    bool fresh(const builtin_interfaces::msg::Time& stamp) const {
        double age=(now()-rclcpp::Time(stamp)).seconds();
        return age>=-0.1 && age<timeout_;
    }
    std::pair<int,int> index(double x,double y) const {
        return {int(std::floor((x-grid_->info.origin.position.x)/grid_->info.resolution)),
                int(std::floor((y-grid_->info.origin.position.y)/grid_->info.resolution))};
    }
    bool free(double x,double y) const {
        auto [ix,iy]=index(x,y);
        return ix>=0 && iy>=0 && ix<int(grid_->info.width) && iy<int(grid_->info.height) &&
            grid_->data[ix+iy*grid_->info.width]==0;
    }
    bool visible(double x,double y,double tx,double ty) const {
        int steps=std::max(1,int(std::ceil(std::hypot(tx-x,ty-y)/(grid_->info.resolution*0.25))));
        for(int k=0;k<=steps;++k) {
            double t=double(k)/steps;
            if(!free(x+t*(tx-x),y+t*(ty-y))) return false;
        }
        return true;
    }
    void status(const std::string& s) {
        if(s==last_status_) return;
        std_msgs::msg::String m; m.data=s; status_pub_->publish(m); last_status_=s;
    }
    void stop(const std::string& reason) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        pid_.reset(); out_x_=out_y_=0; status(reason);
    }
    void tick() {
        double t=now().seconds();
        double dt=t-last_tick_; last_tick_=t;
        if(dt<=0 || dt>0.5) {route_.clear(); stop("clock_reset_or_pause"); return;}
        if(!grid_ || !odom_) {stop("waiting_for_map_and_odom"); return;}
        if(!fresh(grid_->header.stamp)||!fresh(odom_->header.stamp)) {stop("stale_input"); return;}
        if(odom_->header.frame_id!="map" || odom_->child_frame_id!="base_link") {stop("frame_mismatch"); return;}
        if(!goal_ || goal_->header.frame_id!="map") {stop("waiting_for_map_goal"); return;}
        auto& pos=odom_->pose.pose.position;
        auto& q=odom_->pose.pose.orientation;
        auto& vel=odom_->twist.twist.linear;
        double gx=goal_->pose.position.x, gy=goal_->pose.position.y;
        double qnorm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if(!std::isfinite(pos.x+pos.y+gx+gy+vel.x+vel.y+qnorm) || std::abs(qnorm-1)>0.01) {
            stop("invalid_state"); return;
        }
        double distance=std::hypot(gx-pos.x,gy-pos.y);
        if(distance<0.18 || arrived_) {arrived_=true; stop("goal_reached"); return;}
        if(!free(pos.x,pos.y)) {route_.clear(); stop("start_or_goal_occupied"); return;}
        if(!path_ || path_->header.frame_id!="map" || !fresh(path_->header.stamp)) {route_.clear();stop("waiting_for_fresh_path");return;}
        if(route_.empty()) {stop("no_path"); return;}
        // Legacy six-cell lookahead, with collision checking before shortcuts.
        size_t nearest=0;
        for(size_t i=1;i<route_.size();++i)
            if(std::hypot(route_[i].first-pos.x,route_[i].second-pos.y)<
               std::hypot(route_[nearest].first-pos.x,route_[nearest].second-pos.y)) nearest=i;
        auto target=route_[nearest];
        bool found=false;
        for(size_t i=nearest;i<std::min(route_.size(),nearest+7);++i) {
            if(visible(pos.x,pos.y,route_[i].first,route_[i].second)) {target=route_[i];found=true;}
            else break;
        }
        if(distance<0.5 && visible(pos.x,pos.y,gx,gy)) {target={gx,gy};found=true;}
        if(!found) {route_.clear(); stop("path_blocked"); return;}
        double dx=target.first-pos.x, dy=target.second-pos.y, norm=std::hypot(dx,dy);
        if(norm<1e-5) {route_.clear();stop("replan");return;}
        double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        // Legacy terminal speed taper; conservative coursework speed limit.
        double desired=speed_*std::min(1.0,0.2+0.8*distance/2.0);
        double bx=(std::cos(yaw)*dx+std::sin(yaw)*dy)*desired/norm;
        double by=(-std::sin(yaw)*dx+std::cos(yaw)*dy)*desired/norm;
        double ux=pid_.updateAxis(pid_.x,bx,vel.x,dt), uy=pid_.updateAxis(pid_.y,by,vel.y,dt);
        double mag=std::hypot(ux,uy);
        if(mag>speed_) {ux*=speed_/mag; uy*=speed_/mag;}
        double delta=std::hypot(ux-out_x_,uy-out_y_);
        if(delta>accel_*dt) {ux=out_x_+(ux-out_x_)*accel_*dt/delta;uy=out_y_+(uy-out_y_)*accel_*dt/delta;}
        geometry_msgs::msg::Twist cmd; cmd.linear.x=ux;cmd.linear.y=uy;
        cmd_pub_->publish(cmd);out_x_=ux;out_y_=uy;status("tracking");
    }
    Grid::ConstSharedPtr grid_;Odom::ConstSharedPtr odom_;Pose::ConstSharedPtr goal_;
    nav_msgs::msg::Path::ConstSharedPtr path_;
    rclcpp::Subscription<Grid>::SharedPtr map_sub_;
    rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
    rclcpp::Subscription<Pose>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    LegacyPid pid_;std::vector<std::pair<double,double>> route_;
    double speed_,accel_,timeout_,last_tick_=0,out_x_=0,out_y_=0;
    bool arrived_=false;std::string last_status_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Controller>());rclcpp::shutdown();}
