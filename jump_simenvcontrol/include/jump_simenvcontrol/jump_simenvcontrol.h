#ifndef JUMP_SIMENVCONTROL_H
#define JUMP_SIMENVCONTROL_H

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
#include "gz/sim/components/JointForceCmd.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocity.hh"
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>

#include <OsqpEigen/OsqpEigen.h>

#include "jump/msgs/lowstates.pb.h"
#include "jump/msgs/lowcmd.pb.h"

#include "tools_gz/tools_gz.h"

class JumpSimEnvControl : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemReset
{
public:
    /// Constructor
    JumpSimEnvControl();

    /// Destructor
    ~JumpSimEnvControl();

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr);

    void Reset(const gz::sim::UpdateInfo &_info,
               gz::sim::EntityComponentManager &_ecm);

private:
    ToolsGZ _toolsGz;

    gz::transport::Node _Node;

    gz::sim::v8::Model model;

    std::string model_name;

    std::vector<gz::sim::v8::Entity> joint_entities;

    std::vector<std::string> joint_names;

    Eigen::VectorXd qaux, qmin, qmax;
};

#endif
