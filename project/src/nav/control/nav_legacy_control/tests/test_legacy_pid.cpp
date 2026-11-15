#include "nav_legacy_control/legacy_pid.hpp"
#include <stdexcept>
int main() {
 LegacyPid pid;
 if(std::abs(pid.updateAxis(pid.x,1,0,.05)-1.35)>1e-9) throw std::runtime_error("feedforward/P");
 pid.x.ki=1;pid.x.kd=0;
 for(int i=0;i<100;++i) pid.updateAxis(pid.x,1,0,.05);
 if(pid.x.integral!=.5) throw std::runtime_error("integral clamp");
 pid.reset();if(pid.x.integral!=0||pid.x.has_previous_error) throw std::runtime_error("reset");
}
