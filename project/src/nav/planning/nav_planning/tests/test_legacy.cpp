#include "nav_planning/legacy_dstar.hpp"
#include <iostream>
#include <stdexcept>
#include <queue>
#include <random>
void require(bool b,const char* message) {if(!b) throw std::runtime_error(message);}
int main() {
    std::vector<int8_t> grid(20*20,0);
    for(int y=0;y<16;++y) grid[10+y*20]=100;
    dstarlite planner(20,20,grid);
    auto path=planner.plan(2,2,17,2);
    require(!path.empty(),"detour must exist");
    require(path.front()==std::make_pair(2,2) && path.back()==std::make_pair(17,2),"path endpoints");
    for(auto [x,y]:path) require(grid[x+y*20]==0,"path crosses obstacle");
    // A second solve exercises reset of all successor / link-cut state.
    require(!planner.plan(17,2,2,2).empty(),"reverse solve");
    for(int y=0;y<20;++y) grid[10+y*20]=100;
    dstarlite blocked(20,20,grid);
    require(blocked.plan(2,2,17,2).empty(),"sealed wall must be unreachable");
    require(blocked.plan(-1,2,2,2).empty(),"negative start rejected");
    std::vector<int8_t> diagonal={0,100,100,0};
    dstarlite corner(2,2,diagonal);
    require(corner.plan(0,0,1,1).empty(),"no diagonal corner cutting");
    // Independent breadth-first reachability oracle for binary grids.
    std::mt19937 rng(20261027);
    for(int trial=0;trial<40;++trial) {
        std::vector<int8_t> sample(12*9);
        for(auto& v:sample) v=(rng()%5==0)?100:0;
        sample.front()=sample.back()=0;
        std::vector<bool> seen(sample.size(),false);
        std::queue<int> work;work.push(0);seen[0]=true;
        while(!work.empty()) {
            int i=work.front();work.pop();int x=i%12,y=i/12;
            for(int a=-1;a<=1;++a) for(int b=-1;b<=1;++b) {
                int nx=x+a,ny=y+b;
                if((!a&&!b)||nx<0||nx>=12||ny<0||ny>=9)continue;
                if(sample[nx+ny*12] || (a&&b&&(sample[nx+y*12]||sample[x+ny*12])))continue;
                int j=nx+ny*12;if(!seen[j]) {seen[j]=true;work.push(j);}
            }
        }
        dstarlite search(12,9,sample);
        auto route=search.plan(0,0,11,8);
        require((!route.empty())==seen.back(),"random BFS reachability disagreement");
        for(size_t i=0;i<route.size();++i) {
            auto [x,y]=route[i];require(sample[x+y*12]==0,"random route collision");
            if(i) {
                auto [px,py]=route[i-1];
                require(std::abs(x-px)<=1 && std::abs(y-py)<=1,"nonadjacent successor");
                if(x!=px&&y!=py) require(!sample[x+py*12]&&!sample[px+y*12],"random diagonal cuts corner");
            }
        }
    }
    std::cout<<"legacy D* Lite math checks passed\n";
}
