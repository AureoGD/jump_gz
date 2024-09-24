#ifndef REFGOVCON_H
#define REFGOVCON_H

#include "jump_rgc/op_wrapper.h"
#include <OsqpEigen/OsqpEigen.h>
#include "jump_rgc/jump_robot_model.h"

#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#include "gz/custom_msgs/low_cmd.pb.h"
#include "gz/custom_msgs/low_states.pb.h"

#include "tools_gz/tools_gz.h"

#include "jump/msgs/lowstates.pb.h"
#include "jump/msgs/lowcmd.pb.h"

class RefGovCon
{
public:
    RefGovCon();

    ~RefGovCon();

    bool rgc(int mode);

    bool get_states();

    bool update_model();

    bool rgc_cb(const gz::msgs::Int32 &mode_msg, jump::msgs::LowCmd &low_cmd_msg);

private:
    std::string service_name = "JumpRobot/RGC/lowCmd";

    Op_Wrapper _optProblem;

    Eigen::Matrix<double, 2, 1> b, db, q, dq, qr;

    Eigen::VectorXd msg_q, msg_dq, msg_qr;

    JumpRobot _JumpRobot;

    gz::transport::Node _Node;

    jump::msgs::LowStates _low_states;

    ToolsGZ _toolsGz;

    bool op_retur = false;
};
#endif