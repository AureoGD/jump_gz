#ifndef JUMP_STATES_H
#define JUMP_STATES_H

#include <memory>
#include <set>
#include <string>

#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Joint.hh"
#include <gz/sim/components/JointPositionReset.hh>
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointForce.hh"

#include <gz/msgs.hh>

#include <OsqpEigen/OsqpEigen.h>
#include "tools_gz/tools_gz.h"

// #include "gz/custom_msgs/low_states.pb.h"
// #include "gz/custom_msgs/vector.pb.h"

#include "jump/msgs/lowstates.pb.h"
#include "jump/msgs/lowcmd.pb.h"

class JumpStates : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate,
                   public gz::sim::ISystemReset
{
public:
    JumpStates();

    ~JumpStates();

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override;

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm);

    void Reset(const gz::sim::UpdateInfo &_info,
               gz::sim::EntityComponentManager &_ecm);

private:
    void CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                          gz::sim::v8::Entity _joint);

    bool JumpStatesCB(const gz::msgs::Boolean &req, jump::msgs::LowStates &rep);

    void TouchCB(const gz::msgs::Wrench &msg);

    void HFE_TorqueCB(const gz::msgs::Wrench &msg);

    void KFE_TorqueCB(const gz::msgs::Wrench &msg);

    void LowCmdCB(const jump::msgs::LowCmd &_msg);

    bool testeCB(const gz::msgs::Boolean &req, gz::msgs::Boolean &rep);

    std::string ts_topic = "/JumpRobot/foot/torqueSensor";

    std::string hfe_f_topic = "/JumpRobot/hfe/torqueSensor";

    std::string kfe_f_topic = "/JumpRobot/kfe/torqueSensor";

    std::string low_level_topic_name = "/JumpRobot/Control/lowCmd";

    std::string service_name = "/JumpRobot/States/lowState";

    ToolsGZ _toolsGz;

    gz::transport::Node _Node;

    gz::sim::v8::Model model;

    std::string model_name;

    std::vector<gz::sim::v8::Entity> joint_entities;

    Eigen::VectorXd q, dq, tau, qr, dqr;

    std::mutex JumpStatesMutex;

    bool first_scan = false;

    bool b_var = false;

    bool touch_st = false;
    bool touch_st_cb = false;
    bool last_touch_state_cb = false;
};

#endif
