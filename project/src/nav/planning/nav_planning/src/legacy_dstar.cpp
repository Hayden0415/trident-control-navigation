// Source: trident-rm/2027-navi-demo bot_sim/src/dstarlite.cpp; see docs/provenance.json.
#include "nav_planning/legacy_dstar.hpp"
#include <cstdint>
#include <stdexcept>
dstarlite::dstarlite(int width, int height, const std::vector<int8_t>& occupancy)
    : max_x(width), max_y(height) {
    if (width <= 0 || height <= 0 || occupancy.size() != size_t(width)*height)
        throw std::invalid_argument("invalid occupancy dimensions");
    map = new Nodeptr*[max_x];
    for (int x=0; x<max_x; ++x) {
        map[x] = new Nodeptr[max_y];
        for (int y=0; y<max_y; ++y) {
            map[x][y] = new Node(x,y);
            map[x][y]->obstacle_possibility = map[x][y]->static_obstacle_possibility =
                occupancy[x+y*max_x] < 0 ? 100 : occupancy[x+y*max_x];
        }
    }
    lct = new LinkCutTree();
}
dstarlite::~dstarlite() {
    for(int x=0;x<max_x;++x) {
        for(int y=0;y<max_y;++y) delete map[x][y];
        delete[] map[x];
    }
    delete[] map;
    delete lct;
}
double dstarlite::calculate_edge_value(Nodeptr cur) const {
    return std::max(1 + 80/(1 + std::exp(-0.2*(cur->static_obstacle_possibility-70))),
                   (1 + 80/(1 + std::exp(-0.2*(cur->obstacle_possibility-70))))*0.5);
}
std::vector<std::pair<int,int>> dstarlite::plan(int sx,int sy,int gx,int gy) {
    auto inside=[this](int x,int y){return x>=0 && y>=0 && x<max_x && y<max_y;};
    if(!inside(sx,sy)||!inside(gx,gy)) return {};
    if(map[sx][sy]->obstacle_possibility>=100 || map[gx][gy]->obstacle_possibility>=100) return {};
    dstar_list.clear();
    for(int x=0;x<max_x;++x) for(int y=0;y<max_y;++y) map[x][y]->clear();
    final_goal_node=map[gx][gy];
    final_goal_node->rhs=0;
    final_goal_node->dstar_list_status=IN_LIST;
    dstar_list.insert(final_goal_node);
    dstar_update(map[sx][sy]);
    std::vector<std::pair<int,int>> path;
    Nodeptr cur=map[sx][sy];
    for(int i=0;cur && i<max_x*max_y;++i) {
        if(cur->obstacle_possibility>=100) return {};
        path.emplace_back(cur->x,cur->y);
        if(cur==final_goal_node) return path;
        cur=cur->succ;
    }
    return {};  // unreachable or successor loop
}
void dstarlite::dstar_update_node(Nodeptr cur){
    if(cur == final_goal_node)return;
    if(cur->dstar_list_status == IN_LIST){
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
    }
    Nodeptr previous_succ = cur->succ;
    double previous_rhs = cur->rhs;
    double ox = 0, oy = 0, min_acceptable_val = 0;
    cur->rhs = INF;
    cur->succ = nullptr;
    if(previous_succ != nullptr){
        ox = previous_succ->x;
        oy = previous_succ->y;
        min_acceptable_val = 0.1;
    }
    for(int i = 0; i < 8; i++){
        int nx = cur->x + dx[i];
        int ny = cur->y + dy[i];
        if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
        if (cur->obstacle_possibility >= 100 || map[nx][ny]->obstacle_possibility >= 100) continue;
        if (i >= 4 && (map[cur->x][ny]->obstacle_possibility >= 100 || map[nx][cur->y]->obstacle_possibility >= 100)) continue;
        double value = calculate_edge_value(cur)*calculate_edge_value(map[nx][ny]);
        double new_rhs = i<4?map[nx][ny]->dis_to_goal + value : map[nx][ny]->dis_to_goal + 1.414 * value;
        if(new_rhs + sqrt((nx - ox) * (nx - ox) + (ny - oy) * (ny - oy) * value) * min_acceptable_val < cur->rhs){
            cur->rhs = new_rhs;
            cur->succ = map[nx][ny];
        }
    }
    if(previous_rhs == cur -> dis_to_goal && previous_succ != nullptr)lct->del(cur, previous_succ);
    if(cur->rhs == cur->dis_to_goal && cur->succ != nullptr)lct->link(cur, cur->succ);
    if(cur -> rhs != cur->dis_to_goal){
        cur->dstar_list_status = IN_LIST;
        dstar_list.insert(cur);
    }
}

void dstarlite::dstar_update(Nodeptr end_node){

    round++;
    while(!dstar_list.empty()){

        Nodeptr cur = *dstar_list.begin();
        dstar_list.erase(cur);
        cur->dstar_list_status = OUT_LIST;
        if(cur->last_out_list_round != round)cur->appear_time = 0;
        cur->appear_time++;
        cur->last_out_list_round = round;
        if(cur->dis_to_goal < cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = INF + 1;
            cur -> dstar_list_status = IN_LIST;
            dstar_list.insert(cur);
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
        }
        else if(cur -> dis_to_goal > cur -> rhs){
            int x = cur->x, y = cur->y;
            cur -> dis_to_goal = cur -> rhs;
            for(int i = 0; i < 8; i++){
                int nx = x + dx[i];
                int ny = y + dy[i];
                if(nx < 0 || nx >= max_x || ny < 0 || ny >= max_y)continue;
                dstar_update_node(map[nx][ny]);
            }
            if(cur->succ != nullptr)lct->link(cur, cur->succ);
        }
        if(lct->find_root(final_goal_node) == lct->find_root(end_node))break;
    }
}
