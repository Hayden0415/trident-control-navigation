#pragma once
// Adapted from trident-rm/2027-navi-demo, bot_sim/src/dstarlite.cpp.
// Original Node, LinkCutTree, rhs propagation and sigmoid edge cost retained.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <set>
#include <stack>
#include <utility>
#include <vector>
#define INF 1e9
#define NEW 0
#define IN_LIST 1
#define OUT_LIST 2
class dstarlite {
public:
    class Node;
    using Nodeptr = Node*;
        class Node{
            public:
                double dis_to_goal, rhs;
                double dis_to_obstacle, edge_value;
                double obstacle_possibility, static_obstacle_possibility;
                bool reversed = false;
                int x, y, astar_list_status, dstar_list_status, appear_time;
                int last_out_list_round, round_for_find_path, round_for_dynamic_map;
                Nodeptr succ, father, son[2];
                Node(int x, int y){
                    dis_to_goal = INF + 1;
                    appear_time = 0;
                    rhs = INF;
                    this->obstacle_possibility = this->static_obstacle_possibility = 0;
                    this->x = x;
                    this->y = y;
                    astar_list_status = dstar_list_status = NEW;
                    last_out_list_round = round_for_dynamic_map = round_for_find_path = 0;
                    succ = father = son[0] = son[1] = nullptr;
                }
                Node(){}
                void clear(){
                    dis_to_goal = INF + 1;
                    rhs = INF;
                    obstacle_possibility = static_obstacle_possibility;
                    astar_list_status= dstar_list_status = NEW;
                    succ = father = son[0] = son[1] = nullptr;
                    appear_time = 0;
                    reversed = false;
                }
                class Compare_in_Dstar{
                    public:
                        bool operator()(Nodeptr a, Nodeptr b) const {
                            if(std::min(a->dis_to_goal, a->rhs) == std::min(b->dis_to_goal, b->rhs)){
                                if(a -> x == b -> x){
                                    return a -> y < b -> y;
                                }
                                return a -> x < b -> x;
                            }
                            return std::min(a->dis_to_goal, a->rhs) < std::min(b->dis_to_goal, b->rhs);
                        }
                };
                class Compare_in_dynamic_map{
                    public:
                        bool operator()(Nodeptr a, Nodeptr b) const {
                            if(a->round_for_dynamic_map == b->round_for_dynamic_map){
                                if(a->x == b->x){
                                    return a->y < b->y;
                                }
                                return a->x < b->x;
                            }
                            return a->round_for_dynamic_map < b->round_for_dynamic_map;
                        }
                };
        };
        class LinkCutTree{
            public:
                void reverse(Nodeptr now){now->reversed ^= 1;}
                bool get_son(Nodeptr x){return x->father->son[1] == x;}
                bool is_root(Nodeptr x){return x->father == nullptr || (x->father->son[0] != x && x->father->son[1] != x);}
                void rotate(Nodeptr x){
                    Nodeptr f = x->father, ff = f->father;
                    bool son = get_son(x);
                    if(!is_root(f)) { ff->son[get_son(f)] = x; }
                    x -> father = ff;
                    f -> son[son] = x -> son[son^1]; if(x->son[son^1] != nullptr) x->son[son^1]->father = f;
                    x -> son[son^1] = f; f -> father = x;
                }
                void pushdown(Nodeptr now){
                    if(now->reversed){
                        now->reversed = false;
                        if(now->son[0] != nullptr) now->son[0]->reversed ^= 1;
                        if(now->son[1] != nullptr) now->son[1]->reversed ^= 1;
                        std::swap(now->son[0], now->son[1]);
                    }
                    return;
                }
                void splay(Nodeptr now){
                    std::stack<Nodeptr> s;
                    Nodeptr temp = now;
                    while(!is_root(temp)){
                        s.push(temp);
                        temp = temp->father;
                    }
                    s.push(temp);
                    while(!s.empty()){
                        pushdown(s.top());
                        s.pop();
                    }
                    while(!is_root(now)){
                        if(!is_root(now -> father)){
                            if(get_son(now->father) == get_son(now)) rotate(now->father);
                            else rotate(now);
                        }
                        rotate(now);
                    }
                }
                void access(Nodeptr now){
                    Nodeptr last = nullptr;
                    for(Nodeptr x = now; x != nullptr; x = x->father){
                        splay(x);
                        x->son[1] = last;
                        last = x;
                    }
                }
                void makeroot(Nodeptr now){
                    access(now);
                    splay(now);
                    reverse(now);
                }
                Nodeptr find_root(Nodeptr now){
                    access(now);
                    splay(now);
                    pushdown(now);
                    while(now->son[0] != nullptr) { now = now->son[0]; pushdown(now); }
                    return now;
                }
                void link(Nodeptr x, Nodeptr y){
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    if(find_root(x) == find_root(y)){
                        // ROS_ERROR("Link Error");
                        // exit(1);
                        return ;
                    }
                    // ROS_INFO("Link %d %d %d %d", x->x, x->y, y->x, y->y);
                    makeroot(x);
                    makeroot(y);
                    x->father = y;
                }
                void del(Nodeptr x, Nodeptr y){
                    if(x == nullptr || y == nullptr)return;
                    makeroot(x);
                    access(y);
                    splay(y);
                    if(y->son[0] == x && x->son[1] == nullptr && x->son[0] == nullptr){
                        y->son[0] = nullptr;
                        x->father = nullptr;
                    }
                }
        };

    dstarlite(int width, int height, const std::vector<int8_t>& occupancy);
    ~dstarlite();
    dstarlite(const dstarlite&) = delete;
    dstarlite& operator=(const dstarlite&) = delete;
    std::vector<std::pair<int,int>> plan(int sx, int sy, int gx, int gy);
private:
    void dstar_update_node(Nodeptr cur);
    void dstar_update(Nodeptr end_node);
    double calculate_edge_value(Nodeptr cur) const;
    LinkCutTree* lct;
    Nodeptr final_goal_node = nullptr;
    int max_x, max_y, round = 0;
    Nodeptr** map;
    std::set<Nodeptr, Node::Compare_in_Dstar> dstar_list;
    int dx[8] = {1,0,-1,0,1,-1,-1,1};
    int dy[8] = {0,1,0,-1,1,-1,1,-1};
};
