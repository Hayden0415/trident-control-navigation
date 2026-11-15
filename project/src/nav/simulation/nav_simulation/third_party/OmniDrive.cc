// Copyright 2024 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mutex>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/math/PID.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Rand.hh>

#include "OmniDrive.hh"

#define WHEEL_NUM 4

using namespace ignition;
using namespace gazebo;
using namespace systems;

// ---------- SDF element names for the four wheel joints ----------
const std::string kSdfElemJointNames[WHEEL_NUM] = {
    "front_left_joint",   // 0
    "front_right_joint",  // 1
    "rear_left_joint",    // 2
    "rear_right_joint"    // 3
};

// ---------- private data ----------
class ignition::gazebo::systems::OmniDrivePrivate
{
public:
    enum WheelIndex
    {
        FRONT_LEFT  = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT   = 2,
        REAR_RIGHT  = 3
    };

public:
    /// \brief cmd_vel callback (runs in ignition-transport thread)
    void OnCmdVel(const ignition::msgs::Twist &_msg);

    /// \brief publish odometry message
    void UpdateOdometry(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm);

    /// \brief mecanum inverse kinematics: chassis velocity → wheel angular velocity
    void ComputeWheelVelocities(double _vx, double _vy, double _omega,
                                double _target[WHEEL_NUM]);

public:
    // ---- transport ----
    transport::Node node;

    // ---- model & links ----
    Model model{kNullEntity};
    std::string chassisLinkName;
    Entity chassisLink{kNullEntity};

    // ---- wheel joints ----
    std::string wheelJointNames[WHEEL_NUM];
    Entity wheelJoints[WHEEL_NUM];

    // ---- geometry ----
    double wheelRadius{0.076};
    double wheelAxisProjection{1.0};
    bool resetIntegralOnStop{false};
    std::string wheelControlMode{"force_pid"};
    double wheelVelocityLimit{35.0};
    double wheelSeparationX{0.286};   // front–rear distance
    double wheelSeparationY{0.284};   // left–right distance

    // ---- wheel velocity PID ----
    ignition::math::PID wheelPids[WHEEL_NUM];
    double wheelTargetVel[WHEEL_NUM] = {0, 0, 0, 0};

    // ---- cmd_vel ----
    msgs::Twist targetVel;
    std::mutex targetVelMutex;

    // ---- odometry ----
    bool initFlag = false;
    std::string odomFrameId;
    std::string odomChildFrameId;
    ignition::math::Pose3d initPose;
    transport::Node::Publisher odomPub;
    std::chrono::steady_clock::duration lastOdomTime{0};
};

OmniDrive::OmniDrive()
    : dataPtr(std::make_unique<OmniDrivePrivate>())
{
}

void OmniDrive::Configure(const Entity &_entity,
                          const std::shared_ptr<const sdf::Element> &_sdf,
                          EntityComponentManager &_ecm,
                          EventManager & /*_eventMgr*/)
{
    if (_sdf->HasElement("benchmark_seed"))
    {
        const auto seed = _sdf->Get<unsigned int>("benchmark_seed");
        math::Rand::Seed(seed);
        std::srand(seed);
    }
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        ignerr << "OmniDrive plugin should be attached to a model entity."
               << std::endl;
        return;
    }

    this->dataPtr->wheelAxisProjection = _sdf->Get<double>("wheel_axis_projection", 1.0).first;
    this->dataPtr->resetIntegralOnStop = _sdf->Get<bool>("wheel_integrator_reset_on_stop", false).first;
    this->dataPtr->wheelControlMode = _sdf->Get<std::string>("wheel_control_mode", "force_pid").first;
    this->dataPtr->wheelVelocityLimit = _sdf->Get<double>("wheel_velocity_limit", 35.0).first;
    if (this->dataPtr->wheelControlMode != "force_pid" &&
        this->dataPtr->wheelControlMode != "velocity_motor")
        throw std::invalid_argument("wheel_control_mode must be force_pid or velocity_motor");
    if (!(std::isfinite(this->dataPtr->wheelVelocityLimit) && this->dataPtr->wheelVelocityLimit > 0.0))
        throw std::invalid_argument("wheel_velocity_limit must be finite and positive");
    if (!(this->dataPtr->wheelAxisProjection > 0.0 && this->dataPtr->wheelAxisProjection <= 1.0))
        throw std::invalid_argument("wheel_axis_projection must lie in (0, 1]");

    // ----- chassis link -----
    this->dataPtr->chassisLinkName = _sdf->Get<std::string>("chassis_link");
    this->dataPtr->chassisLink =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->chassisLinkName);
    if (this->dataPtr->chassisLink == kNullEntity)
    {
        ignerr << "OmniDrive: chassis link [" << this->dataPtr->chassisLinkName
               << "] not found." << std::endl;
        return;
    }

    // ----- wheel joints -----
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        this->dataPtr->wheelJointNames[i] =
            _sdf->Get<std::string>(kSdfElemJointNames[i]);
        this->dataPtr->wheelJoints[i] =
            this->dataPtr->model.JointByName(
                _ecm, this->dataPtr->wheelJointNames[i]);
        if (this->dataPtr->wheelJoints[i] == kNullEntity)
        {
            ignerr << "OmniDrive: wheel joint ["
                   << this->dataPtr->wheelJointNames[i] << "] not found."
                   << std::endl;
            return;
        }
    }

    // ----- geometry (use defaults if not specified) -----
    if (_sdf->HasElement("wheel_radius"))
        this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius");
    if (_sdf->HasElement("wheel_separation_x"))
        this->dataPtr->wheelSeparationX = _sdf->Get<double>("wheel_separation_x");
    if (_sdf->HasElement("wheel_separation_y"))
        this->dataPtr->wheelSeparationY = _sdf->Get<double>("wheel_separation_y");

    igndbg << "OmniDrive geometry: r=" << this->dataPtr->wheelRadius
           << " sep_x=" << this->dataPtr->wheelSeparationX
           << " sep_y=" << this->dataPtr->wheelSeparationY << std::endl;

    // ----- subscribe cmd_vel -----
    std::string topic{this->dataPtr->model.Name(_ecm) + "/cmd_vel"};
    this->dataPtr->node.Subscribe(
        topic, &OmniDrivePrivate::OnCmdVel, this->dataPtr.get());
    ignmsg << "OmniDrive subscribing to [" << topic << "]" << std::endl;

    // ----- odometry publisher -----
    std::string odomTopic{this->dataPtr->model.Name(_ecm) + "/odometry"};
    this->dataPtr->odomPub =
        this->dataPtr->node.Advertise<msgs::Odometry>(odomTopic);
    this->dataPtr->odomFrameId = this->dataPtr->model.Name(_ecm) + "/odom";
    this->dataPtr->odomChildFrameId =
        this->dataPtr->model.Name(_ecm) + "/" +
        ignition::common::replaceAll(
            this->dataPtr->chassisLinkName, "::", "/");

    // ----- init wheel velocity PID -----
    // (P, I, D, integral torque max/min, command max/min, command offset)
    const double wheelPidP = _sdf->Get<double>("wheel_pid_p", 0.5).first;
    const double wheelPidI = _sdf->Get<double>("wheel_pid_i", 0.0).first;
    const double wheelIntegralLimit = _sdf->Get<double>("wheel_integral_torque_limit", 0.0).first;
    const double wheelTorqueLimit = _sdf->Get<double>("wheel_torque_limit", 50.0).first;
    if (!(wheelPidP > 0.0 && wheelPidI >= 0.0 && wheelIntegralLimit >= 0.0 && wheelTorqueLimit > 0.0))
        throw std::invalid_argument("wheel PID gain and torque limit must be positive");
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        this->dataPtr->wheelPids[i].Init(wheelPidP, wheelPidI, 0.0,
                                       wheelIntegralLimit, -wheelIntegralLimit,
                                       wheelTorqueLimit, -wheelTorqueLimit, 0.0);
    }

    igndbg << "OmniDrive configured successfully." << std::endl;
}


void OmniDrive::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                          ignition::gazebo::EntityComponentManager &_ecm)
{
    IGN_PROFILE("OmniDrive::PreUpdate");

    // nothing to do if paused
    if (_info.paused)
        return;

    // ----- ensure chassis components exist -----
    if (!_ecm.Component<components::WorldPose>(this->dataPtr->chassisLink))
    {
        _ecm.CreateComponent(this->dataPtr->chassisLink,
                             components::WorldPose());
    }
    // Gazebo only computes optional velocity components when requested.
    if (!_ecm.Component<components::LinearVelocity>(this->dataPtr->chassisLink))
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::LinearVelocity());
    if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->chassisLink))
        _ecm.CreateComponent(this->dataPtr->chassisLink, components::AngularVelocity());

    // ----- read target velocity (thread-safe) -----
    msgs::Twist targetVel;
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetVelMutex);
        targetVel = this->dataPtr->targetVel;
    }

    double vx = targetVel.linear().x();
    double vy = targetVel.linear().y();
    double omega = targetVel.angular().z();

    // ----- mecanum inverse kinematics -----
    double targets[WHEEL_NUM];
    this->dataPtr->ComputeWheelVelocities(vx, vy, omega, targets);

    // ----- PID control per wheel -----
    for (int i = 0; i < WHEEL_NUM; i++)
    {
        auto jointEntity = this->dataPtr->wheelJoints[i];

        if (this->dataPtr->wheelControlMode == "velocity_motor")
        {
            // Fortress DiffDrive uses this same joint motor command component.
            // It reaches the DART SERVO actuator through Physics, not a chassis
            // velocity/pose reset. Joint contacts, inertia and slip remain live.
            // Never create JointForceCmd in this mode: Physics gives force
            // commands precedence over JointVelocityCmd, even for zero force.
            const double limit = this->dataPtr->wheelVelocityLimit;
            const double target = std::max(-limit, std::min(limit, targets[i]));
            auto velCmd = _ecm.Component<components::JointVelocityCmd>(jointEntity);
            if (!velCmd)
                _ecm.CreateComponent(jointEntity, components::JointVelocityCmd({target}));
            else
                _ecm.SetComponentData<components::JointVelocityCmd>(jointEntity, {target});
            if (!_ecm.Component<components::JointVelocity>(jointEntity))
                _ecm.CreateComponent(jointEntity, components::JointVelocity());
            this->dataPtr->wheelTargetVel[i] = target;
            continue;
        }

        // ensure JointVelocity component exists
        auto velComp = _ecm.Component<components::JointVelocity>(jointEntity);
        if (!velComp)
        {
            _ecm.CreateComponent(jointEntity, components::JointVelocity());
            continue;  // skip this iteration, velocity not available yet
        }

        // current angular velocity [rad/s]
        if (velComp->Data().empty())
            continue;  // physics fills a newly requested component on its next update
        double currentVel = velComp->Data().at(0);
        // A stopped wheel must not retain a previous driving integral torque.
        // P-only braking remains active and the original profile is unchanged.
        if (this->dataPtr->resetIntegralOnStop && std::abs(targets[i]) < 1e-6)
            this->dataPtr->wheelPids[i].Reset();
        double error = currentVel - targets[i];
        double forceCmd = this->dataPtr->wheelPids[i].Update(error, _info.dt);

        // ensure JointForceCmd component exists
        auto forceComp =
            _ecm.Component<components::JointForceCmd>(jointEntity);
        if (!forceComp)
        {
            _ecm.CreateComponent(jointEntity,
                                 components::JointForceCmd({0.0}));
        }
        _ecm.SetComponentData<components::JointForceCmd>(
            jointEntity, {forceCmd});

        // store for logging
        this->dataPtr->wheelTargetVel[i] = targets[i];
    }
}

void OmniDrive::PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                           const ignition::gazebo::EntityComponentManager &_ecm)
{
    if (_info.paused || _info.simTime - this->dataPtr->lastOdomTime < std::chrono::milliseconds(10))
        return;
    this->dataPtr->lastOdomTime = _info.simTime;
    this->dataPtr->UpdateOdometry(_info, _ecm);
}



void OmniDrivePrivate::OnCmdVel(const ignition::msgs::Twist &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetVelMutex);
    this->targetVel = _msg;
}

void OmniDrivePrivate::ComputeWheelVelocities(double _vx, double _vy,
                                              double _omega,
                                              double _target[WHEEL_NUM])
{
    // omni wheel inverse kinematics (rollers ⟂ wheel plane)
    //
    //   lx = wheel_separation_x / 2   (half wheelbase, front–rear)
    //   ly = wheel_separation_y / 2   (half track width, left–right)
    //   L  = lx + ly
    //
    // Each wheel hub is mounted at ±45°; drive force is along hub axis.
    // Positive ω produces rolling in:
    //   FL → (+x, +y)    FR → (+x, -y)
    //   RL → (-x, +y)    RR → (-x, -y)
    //
    //   ω_fl = (+vx + vy - L·ωz) / r
    //   ω_fr = (+vx - vy - L·ωz) / r
    //   ω_rl = (-vx + vy - L·ωz) / r
    //   ω_rr = (-vx - vy - L·ωz) / r
    //
    // ROS convention: x=forward, y=left, ωz=CCW looking from top

    double lx = wheelSeparationX / 2.0;
    double ly = wheelSeparationY / 2.0;
    double L  = lx + ly;
    // Physical ±45-degree wheel axes project each chassis velocity component
    // by 1/sqrt(2); the legacy profile keeps projection=1 for compatibility.
    double r  = wheelRadius / wheelAxisProjection;

    _target[FRONT_LEFT]  = (+_vx - _vy - L * _omega) / r;
    _target[FRONT_RIGHT] = (-_vx - _vy - L * _omega) / r;
    _target[REAR_LEFT]   = (+_vx + _vy - L * _omega) / r;
    _target[REAR_RIGHT]  = (-_vx + _vy - L * _omega) / r;
}

void OmniDrivePrivate::UpdateOdometry(
    const ignition::gazebo::UpdateInfo &_info,
    const ignition::gazebo::EntityComponentManager &_ecm)
{
    // Guard against missing components (e.g. first frame before physics
    // initialises velocities, or simulation in paused state).
    const auto *poseComp =
        _ecm.Component<components::WorldPose>(this->chassisLink);
    const auto *linVelComp =
        _ecm.Component<components::LinearVelocity>(this->chassisLink);
    const auto *angVelComp =
        _ecm.Component<components::AngularVelocity>(this->chassisLink);
    if (!poseComp || !linVelComp || !angVelComp)
        return;

    Link chassisLinkEnt(this->chassisLink);
    const auto chassisPose = poseComp->Data();
    const auto linearVel   = linVelComp->Data();
    const auto angularVel  = angVelComp->Data();

    if (!this->initFlag)
    {
        this->initPose = chassisPose;
        this->initFlag = true;
    }

    // build odometry message
    msgs::Odometry msg;

    // pose
    msg.mutable_pose()->mutable_position()->set_x(chassisPose.X());
    msg.mutable_pose()->mutable_position()->set_y(chassisPose.Y());
    msg.mutable_pose()->mutable_position()->set_z(chassisPose.Z());
    msgs::Set(msg.mutable_pose()->mutable_orientation(), chassisPose.Rot());

    // twist
    msg.mutable_twist()->mutable_linear()->set_x(linearVel.X());
    msg.mutable_twist()->mutable_linear()->set_y(linearVel.Y());
    msg.mutable_twist()->mutable_angular()->set_z(angularVel.Z());

    // header
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        convert<msgs::Time>(_info.simTime));
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->odomFrameId);
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->odomChildFrameId);

    this->odomPub.Publish(msg);
}

IGNITION_ADD_PLUGIN(OmniDrive,
                    ignition::gazebo::System,
                    OmniDrive::ISystemConfigure,
                    OmniDrive::ISystemPreUpdate,
                    OmniDrive::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(OmniDrive, "ignition::gazebo::systems::OmniDrive")
