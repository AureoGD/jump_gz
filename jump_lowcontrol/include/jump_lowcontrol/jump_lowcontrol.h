#ifndef JUMP_LOWCONTROL_H
#define JUMP_LOWCONTROL_H

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

#include <OsqpEigen/OsqpEigen.h>

#include "jump/msgs/lowstates.pb.h"
#include "jump/msgs/lowcmd.pb.h"

#include "tools_gz/tools_gz.h"

class JumpLowControl : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
    /// Constructor
    JumpLowControl();

    /// Destructor
    ~JumpLowControl();

    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr);

    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm);

    // void Reset(const gz::sim::UpdateInfo &_info,
    //            gz::sim::EntityComponentManager &_ecm) override;

private:
    void CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                          gz::sim::v8::Entity _joint);

    void SetJointPos(Eigen::VectorXd &_q);

    void LowCmdCB(const jump::msgs::LowCmd &_msg);

    bool ResetJointsPosCB(const gz::msgs::Double_V &req, gz::msgs::Boolean &rep);

    bool qrReqCB(const gz::msgs::Empty &req, jump::msgs::LowCmd &rep);

    std::string low_level_topic_name = "JumpRobot/Control/lowCmd";

    std::string reset_pos_service_name = "JumpRobot/Control/setJointPos";

    std::string req_qr_service_name = "JumpRobot/Control/qrReq";

    double kp, kd;

    ToolsGZ _toolsGz;

    gz::transport::Node _Node;

    gz::sim::v8::Model model;

    std::string model_name;

    std::vector<gz::sim::v8::Entity> joint_entities;

    std::vector<std::string> joint_names;

    Eigen::VectorXd qr, q0, qaux;

    std::chrono::steady_clock::duration updatePeriod{0};

    std::chrono::steady_clock::duration lastUpdateTime{0};

    std::mutex JumpControlMutex;

    int variavel_qlqr = 0;

    gz::sim::EntityComponentManager *ecm_;
};

#endif