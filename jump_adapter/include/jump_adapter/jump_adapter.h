#ifndef JUMP_ADAPTER_H
#define JUMP_ADAPTER_H

#include <gz/transport/Node.hh>
#include "tools_gz/tools_gz.h"
#include <gz/msgs.hh>

#include "jump/msgs/lowstates.pb.h"

class JumpAdapter
{

public:
    JumpAdapter();

    ~JumpAdapter();

    void init();

private:
    bool cb_states(const gz::msgs::Boolean &req, gz::msgs::Double_V &res_msg);

    gz::transport::Node _Node;

    gz::msgs::Empty empty_req;

    jump::msgs::LowStates lowStates_res;

    double value_teste = 0;
};

#endif