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

    this->msg_qr.resize(2, 1);
    this->msg_qr.setZero();

    if (this->_Node.Advertise<RefGovCon, gz::msgs::Int32, jump::msgs::LowCmd>(this->service_name, &RefGovCon::rgc_cb, this))
        std::cout << "The service [" << '/' << this->service_name << "] was created" << std::endl;
    else
        std::cout << "Error advertising service [" << '/' << this->service_name << "]" << std::endl;

    if (this->_Node.Advertise<RefGovCon, gz::msgs::Int32, gz::msgs::Boolean>("JumpRobot/RGC/reqsol", &RefGovCon::solve_cb, this))
        std::cout << "The service [JumpRobot/RGC/reqsol] was created" << std::endl;
    else
        std::cout << "Error advertising service [JumpRobot/RGC/reqsol]" << std::endl;

    this->rgcPub = this->_Node.Advertise<jump::msgs::LowCmd>(this->topic_name);
    if (!this->rgcPub)
    {
        std::cerr << "Error advertising topic [" << this->topic_name << "]" << std::endl;
    }

    this->rgcError = this->_Node.Advertise<gz::msgs::Boolean>(this->topic_error_name);
    if (!this->rgcError)
    {
        std::cerr << "Error advertising topic [" << this->topic_error_name << "]" << std::endl;
    }

    this->_optProblem.RGCConfig(1.0 / 100.0, 60, 5);

    // this->rgc_pub = new gz::transport::Node::Advertise<jump::msgs::LowCmd>(this->topic_name);

    this->run = true;
    this->rgcThread = new std::thread(std::bind(&RefGovCon::thControl, this));
}

RefGovCon::~RefGovCon()
{
    if (this->rgcThread && this->run)
    {
        this->run = false;
        this->rgcThread->join();
    }
    this->rgcThread = nullptr;
}

bool RefGovCon::rgc_cb(const gz::msgs::Int32 &mode_msg, jump::msgs::LowCmd &low_cmd_msg)
{
    std::cout << mode_msg.data() << std::endl;
    // bool solved = this->rgc(mode_msg.data());

    if (this->rgc(mode_msg.data()))
        low_cmd_msg.set_valid(true);
    else
        low_cmd_msg.set_valid(false);

    low_cmd_msg.Clear();
    _toolsGz.EigenVec2VecMsg(this->qr, low_cmd_msg.mutable_qr());

    return true;
}

bool RefGovCon::solve_cb(const gz::msgs::Int32 &req, gz::msgs::Boolean &rep)
{
    std::lock_guard<std::mutex> lock(this->rgcMutex);
    this->mode = req.data();
    rep.set_data(true);
    // std::cout << req.data() << std::endl;
    // std::cout << "send cb response" << std::endl;
    return true;
}

void RefGovCon::thControl()
{
    while (this->run)
    {
        if (this->mode != -1)
        {
            std::lock_guard<std::mutex> lock(this->rgcMutex);

            if (!this->get_states())
                std::cout << "'get_states' error";
            else
            {
                this->_JumpRobot.UpdateSysMatrices();
                int valid_sol = _optProblem.ChooseRGCPO(mode);
                if (valid_sol == -1)
                {
                    gz::msgs::Boolean msg;
                    msg.set_data(0);
                    this->rgcError.Publish(msg);
                }
                jump::msgs::LowCmd low_cmd;
                low_cmd.set_valid(bool(valid_sol));
                _toolsGz.EigenVec2VecMsg(this->qr, low_cmd.mutable_qr());
                this->rgcPub.Publish(low_cmd);
                this->mode = -1;
            }
        }
    }
}

bool RefGovCon::rgc(int mode)
{
    if (!this->get_states())
    {
        std::cout << "'get_states' error";
        return 0;
    }

    this->_JumpRobot.UpdateSysMatrices();

    return _optProblem.ChooseRGCPO(mode);
}

bool RefGovCon::get_states()
{
    // chama o serviço para solicitar os estados internos
    bool result;
    gz::msgs::Boolean req;
    req.set_data(true);
    bool executed = _Node.Request("/JumpRobot/States/lowState", req, 5000, this->_low_states, result);
    if (executed)
    {
        if (result)
        {
            _toolsGz.VecMsg2VecEigen(this->_low_states.q(), &this->msg_q);
            _toolsGz.VecMsg2VecEigen(this->_low_states.dq(), &this->msg_dq);
            _toolsGz.VecMsg2VecEigen(this->_low_states.qr(), &this->msg_qr);

            this->b << 0.0, this->msg_q(0);
            this->db << 0.0, this->msg_dq(0);
            this->q << this->msg_q(1), this->msg_q(2);
            this->dq << this->msg_dq(1), this->msg_dq(2);
            this->qr << this->msg_qr(0), this->msg_qr(1);
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