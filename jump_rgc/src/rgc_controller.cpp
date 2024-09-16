#include "jump_rgc/rgc_controller.h"

RefGovCon::RefGovCon()
    : q(Eigen::Matrix<double, 2, 1>::Zero()),
      dq(Eigen::Matrix<double, 2, 1>::Zero()),
      b(Eigen::Matrix<double, 2, 1>::Zero()),
      db(Eigen::Matrix<double, 2, 1>::Zero()),
      qr(Eigen::Matrix<double, 2, 1>::Zero()),
      _JumpRobot(&q, &dq, &b, &db),
      _optProblem(&_JumpRobot, &q, &dq, &qr)
{
    this->msg_q.resize(3, 1);
    this->msg_q.setZero();

    this->msg_dq.resize(3, 1);
    this->msg_dq.setZero();

    if (this->_Node.Advertise<RefGovCon, gz::msgs::Int32, jump::msgs::LowCmd>(this->service_name, &RefGovCon::rgc_cb, this))
        std::cout << "The service [" << '/' << this->service_name << "] was created" << std::endl;
    else
        std::cout << "Error advertising service [" << '/' << this->service_name << "]" << std::endl;
}

RefGovCon::~RefGovCon()
{
}

bool RefGovCon::rgc_cb(const gz::msgs::Int32 &mode_msg, jump::msgs::LowCmd &low_cmd_msg)
{
    bool solved = this->rgc(mode_msg.data());

    if (solved)
    {
        low_cmd_msg.Clear();
        _toolsGz.EigenVec2VecMsg(this->qr, low_cmd_msg.mutable_qr());
        return 1;
    }
    else
        return 0;
}

bool RefGovCon::rgc(int mode)
{
    if (!this->get_states())
    {
        std::cout << "'get_states' error";
        return 0;
    }

    this->_JumpRobot.UpdateSysMatrices();

    op_retur = _optProblem.ChooseRGCPO(mode);
    return 1;
}

bool RefGovCon::get_states()
{
    // chama o serviço para solicitar os estados internos
    bool result;
    bool executed = _Node.Request("/ChoseAction", 5, this->_low_states, result);
    if (executed)
    {
        if (result)
        {
            _toolsGz.VecMsg2VecEigen(this->_low_states.q(), &this->msg_q);
            _toolsGz.VecMsg2VecEigen(this->_low_states.dq(), &this->msg_dq);

            this->b << 0.0, this->msg_q(0);
            this->db << 0.0, this->msg_dq(0);
            this->q << this->msg_q(1), this->msg_q(2);
            this->dq << this->msg_dq(1), this->msg_dq(2);
        }
        else
        {
            std::cout << "Service call failed" << std::endl;
            return 0;
        }
    }
    else
    {
        std::cout << "Service call timed out" << std::endl;
        return 0;
    }
    return 1;
}