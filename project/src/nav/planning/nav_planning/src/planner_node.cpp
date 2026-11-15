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
#include "nav_planning/legacy_dstar.hpp"
class Planner : public rclcpp::Node {
    using Grid=nav_msgs::msg::OccupancyGrid;
    using Odom=nav_msgs::msg::Odometry;
    using Pose=geometry_msgs::msg::PoseStamped;
public:
    Planner():Node("legacy_planner") {
        timeout_=declare_parameter("input_timeout",1.0);
        if(timeout_<=0) throw std::invalid_argument("timeout must be positive");
        map_sub_=create_subscription<Grid>("/nav/map",rclcpp::QoS(1).reliable().transient_local(),
            [this](Grid::ConstSharedPtr m){if(valid(*m))grid_=m;else grid_.reset();});
        odom_sub_=create_subscription<Odom>("/nav/odom",rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr m){odom_=m;});
        goal_sub_=create_subscription<Pose>("/nav/goal",10,[this](Pose::ConstSharedPtr m){goal_=m;publish_empty();});
        path_pub_=create_publisher<nav_msgs::msg::Path>("/nav/planning/path",1);
        timer_=create_wall_timer(std::chrono::milliseconds(200),[this]{tick();});
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
    // Supplied adapter: choose a reachable local subgoal when the global goal
    // is outside the rolling window. No global completeness is claimed.
    std::pair<int,int> local_goal(int sx,int sy,double gx,double gy) const {
        const int w=grid_->info.width, h=grid_->info.height;
        std::vector<bool> seen(w*h,false);
        std::queue<std::pair<int,int>> pending;
        pending.emplace(sx,sy); seen[sx+sy*w]=true;
        std::pair<int,int> best={sx,sy}; double score=INFINITY;
        while(!pending.empty()) {
            auto [x,y]=pending.front(); pending.pop();
            double wx=grid_->info.origin.position.x+(x+0.5)*grid_->info.resolution;
            double wy=grid_->info.origin.position.y+(y+0.5)*grid_->info.resolution;
            double d=std::hypot(wx-gx,wy-gy);
            if(d<score) {score=d;best={x,y};}
            for(auto [dx,dy]:{std::pair<int,int>{1,0},{-1,0},{0,1},{0,-1}}) {
                int nx=x+dx,ny=y+dy;
                if(nx<0||ny<0||nx>=w||ny>=h||seen[nx+ny*w]||grid_->data[nx+ny*w]!=0) continue;
                seen[nx+ny*w]=true;pending.emplace(nx,ny);
            }
        }
        return best;
    }
    void publish_empty() {
        nav_msgs::msg::Path path;path.header.frame_id="map";path.header.stamp=now();path_pub_->publish(path);
    }
    void tick() {
        if(!grid_||!odom_||!goal_||!fresh(grid_->header.stamp)||!fresh(odom_->header.stamp)
           ||goal_->header.frame_id!="map"||odom_->header.frame_id!="map") {publish_empty();return;}
        const auto& p=odom_->pose.pose.position;
        double gx=goal_->pose.position.x,gy=goal_->pose.position.y;
        if(!std::isfinite(p.x+p.y+gx+gy)||!free(p.x,p.y)){publish_empty();return;}
        auto [sx,sy]=index(p.x,p.y);auto [ex,ey]=index(gx,gy);
        if(ex>=0&&ey>=0&&ex<int(grid_->info.width)&&ey<int(grid_->info.height)) {
            if(!free(gx,gy)){publish_empty();return;}
        } else {auto g=local_goal(sx,sy,gx,gy);ex=g.first;ey=g.second;}
        dstarlite planner(grid_->info.width,grid_->info.height,grid_->data);
        auto cells=planner.plan(sx,sy,ex,ey);
        nav_msgs::msg::Path path;path.header=grid_->header;path.header.stamp=now();
        for(auto [ix,iy]:cells) {
            Pose pose;pose.header=path.header;pose.pose.orientation.w=1.;
            pose.pose.position.x=grid_->info.origin.position.x+(ix+.5)*grid_->info.resolution;
            pose.pose.position.y=grid_->info.origin.position.y+(iy+.5)*grid_->info.resolution;
            path.poses.push_back(pose);
        }
        path_pub_->publish(path);
    }
    Grid::ConstSharedPtr grid_;Odom::ConstSharedPtr odom_;Pose::ConstSharedPtr goal_;
    double timeout_;
    rclcpp::Subscription<Grid>::SharedPtr map_sub_;
    rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
    rclcpp::Subscription<Pose>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Planner>());rclcpp::shutdown();}
