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

class JumpStates : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPostUpdate
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

private:
    void CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                          gz::sim::v8::Entity _joint);

    bool JumpStatesCB(const gz::msgs::Boolean &req, jump::msgs::LowStates &resp_msg);

    bool testeCB(const gz::msgs::Boolean &req, gz::msgs::Boolean &resp_msg);

    ToolsGZ _toolsGz;

    gz::transport::Node _Node;

    gz::sim::v8::Model model;

    std::string service_name = "jump/low_state";

    std::string model_name;

    std::vector<gz::sim::v8::Entity> joint_entities;

    Eigen::VectorXd q, dq, tau;

    std::mutex JumpStatesMutex;

    bool b_var = false;
};

#endif
