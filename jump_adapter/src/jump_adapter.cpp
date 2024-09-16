#include "jump_adapter/jump_adapter.h"

JumpAdapter::JumpAdapter()
{
}

JumpAdapter::~JumpAdapter()
{
}

void JumpAdapter::init()
{
    if (this->_Node.Advertise<JumpAdapter, gz::msgs::Boolean, gz::msgs::Double_V>("jump/adapter/req_states", &JumpAdapter::cb_states, this))
    {
        std::cout << "The service [" << "jump/adapter/req_states" << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error advertising service [" << "jump/adapter/req_states" << "]" << std::endl;
    }
}

bool JumpAdapter::cb_states(const gz::msgs::Boolean &req, gz::msgs::Double_V &res_msg)
{
    bool result;
    // bool executed = _Node.Request("/jump/low_state", this->empty_req, 10000, this->lowStates_res, result);
    // std::cout << executed << std::endl;
    gz::msgs::Boolean msg;
    gz::msgs::Boolean _req;

    _req.set_data(false);
    msg.Clear();
    bool executed = _Node.Request("ser/teste", _req, 500, msg, result);
    std::cout << executed << ", " << result << ", " << msg.data() << std::endl;

    value_teste += 1;
    res_msg.mutable_data()->Add(value_teste);

    return 1;
}