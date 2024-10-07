#include "jump_lowcontrol/jump_lowcontrol.h"

JumpLowControl::JumpLowControl()
{
    this->q0.resize(3, 1);
    this->q0.setZero();

    this->qaux.resize(3, 1);
    this->qaux.setZero();

    this->qr.resize(2, 1);
    this->qr.setZero();
}

JumpLowControl::~JumpLowControl()
{
}

void JumpLowControl::Configure(const gz::sim::Entity &_entity,
                               const std::shared_ptr<const sdf::Element> &_sdf,
                               gz::sim::EntityComponentManager &_ecm,
                               gz::sim::EventManager &_eventMgr)
{

    std::cout << "========================== Low Controller =========================" << std::endl;

    ecm_ = &_ecm;

    std::cout << ecm_ << std::endl;

    this->model = gz::sim::Model(_entity);

    this->model_name = this->model.Name(_ecm);

    // std::cout << this->model_name << std::endl;

    this->kp = _sdf->Get<double>("kp");

    this->kd = _sdf->Get<double>("kd");

    double rate = _sdf->Get<double>("update_rate", 100).first;

    std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};

    std::string dataFromSDF;

    if (_sdf->HasElement("joint_name"))
    {
        int i = 0;
        auto elem = _sdf->FindElement("joint_name");
        while (elem)
        {
            std::string jointName = elem->Get<std::string>();
            gz::sim::v8::Entity jointEntity = this->model.JointByName(_ecm, jointName);
            if (jointEntity != gz::sim::v8::kNullEntity)
            {
                this->CreateComponents(_ecm, jointEntity);
                // _ecm.SetComponentData<gz::sim::v8::components::JointPositionReset>(jointEntity, {q0(i)});
                i++;
            }
            else
            {
                gzerr << "Joint with name[" << jointName << "] not found. "
                      << "The JointStatePublisher will not publish this joint.\n";
            }

            elem = elem->GetNextElement("joint_name");
        }
    }
    else
    {
        gzerr << "Atribute 'joint_name' was not found in the SDF file .\n";
    }

    this->joint_names.push_back("base");

    this->qr << -3.14 * 30 / 180, 3.14 * 50 / 180;

    this->q0 << -3.14 * 50 / 180, 3.14 * 80 / 180, 0.6;

    // this->SetJointPos(_ecm, this->q0);
    this->SetJointPos(this->q0);

    // service to put the robot at new pose
    if (this->_Node.Advertise<JumpLowControl, gz::msgs::Double_V, gz::msgs::Boolean>(this->reset_pos_service_name, &JumpLowControl::ResetJointsPosCB, this))
    {
        std::cout << "The service [" << this->reset_pos_service_name << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error advertising service [" << this->reset_pos_service_name << "]" << std::endl;
    }

    // service that request current reference
    if (this->_Node.Advertise<JumpLowControl, gz::msgs::Empty, jump::msgs::LowCmd>(this->req_qr_service_name, &JumpLowControl::qrReqCB, this))
    {
        std::cout << "The service [" << this->req_qr_service_name << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error advertising service [" << this->req_qr_service_name << "]" << std::endl;
    }

    // subscribe to the topic for new reference
    if (this->_Node.Subscribe<JumpLowControl, jump::msgs::LowCmd>(this->low_level_topic_name, &JumpLowControl::LowCmdCB, this))
    {
        std::cout << "Subscribed to the topic [" << this->low_level_topic_name << "]" << std::endl;
    }
    else
    {
        std::cout << "Error to subscribe to the topic [" << this->low_level_topic_name << "]" << std::endl;
    }
    std::cout << "===================================================================" << std::endl;
}

void JumpLowControl::CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                                      gz::sim::v8::Entity _joint)
{
    this->joint_names.push_back(_ecm.ComponentData<gz::sim::components::Name>(_joint).value());

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointPosition().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointPosition());
    }

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointVelocity().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointVelocity());
    }

    if (!_ecm.EntityHasComponentType(_joint,
                                     gz::sim::v8::components::JointForceCmd().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointForceCmd({0}));
    }
}

void JumpLowControl::SetJointPos(Eigen::VectorXd &_q)
{
    for (int idx = 0; idx < 3; idx++)
    {
        // std::cout << &this->joint_names[idx] << std::endl;1
        std::cout << (_q)(idx) << std::endl;
        this->ecm_->SetComponentData<gz::sim::v8::components::JointPositionReset>(this->model.JointByName(*this->ecm_, this->joint_names[idx]), {(_q)(idx)});
    }
    std::cout << "----" << std::endl;
}

bool JumpLowControl::ResetJointsPosCB(const gz::msgs::Double_V &req, gz::msgs::Boolean &rep)
{
    _toolsGz.VecMsg2VecEigen(req, &this->qaux);
    this->SetJointPos(this->qaux);
    rep.set_data(true);
    return true;
}

bool JumpLowControl::qrReqCB(const gz::msgs::Empty &req, jump::msgs::LowCmd &rep)
{
    _toolsGz.EigenVec2VecMsg(this->qr, rep.mutable_qr());
    return 1;
}

void JumpLowControl::LowCmdCB(const jump::msgs::LowCmd &_msg)
{
    // std::cout << "hey, low controller" << std::endl;
    std::lock_guard<std::mutex> lock(this->JumpControlMutex);
    _toolsGz.VecMsg2VecEigen(_msg.qr(), &this->qr);
    // _toolsGz.VecMsg2VecEigen(_msg.dq(), &this->dqr);
}

void JumpLowControl::PreUpdate(const gz::sim::UpdateInfo &_info,
                               gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused)
        return;

    std::lock_guard<std::mutex> lock(this->JumpControlMutex);

    auto elapsed = _info.simTime - this->lastUpdateTime;

    if (elapsed > std::chrono::steady_clock::duration::zero() && elapsed < this->updatePeriod)
        return;

    for (int idx = 0; idx < 2; idx++)
    {
        const auto *jointPositions = _ecm.Component<gz::sim::v8::components::JointPosition>(this->model.JointByName(_ecm, this->joint_names[idx]));
        const auto *jointVelocity = _ecm.Component<gz::sim::v8::components::JointVelocity>(this->model.JointByName(_ecm, this->joint_names[idx]));

        if (jointPositions == nullptr || jointPositions->Data().empty() || jointVelocity == nullptr || jointVelocity->Data().empty())
            return;

        double tau = this->kp * (this->qr[idx] - jointPositions->Data()[0]) - this->kd * jointVelocity->Data()[0];
        auto forceComp = _ecm.Component<gz::sim::v8::components::JointForceCmd>(this->model.JointByName(_ecm, this->joint_names[idx]));
        *forceComp = gz::sim::v8::components::JointForceCmd({tau});
    }
    this->lastUpdateTime = _info.simTime;
}

GZ_ADD_PLUGIN(JumpLowControl,
              gz::sim::System,
              JumpLowControl::ISystemConfigure,
              JumpLowControl::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(JumpLowControl, "JumpLowControl")
