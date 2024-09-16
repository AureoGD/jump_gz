#include "jump_lowcontrol/jump_lowcontrol.h"

JumpLowControl::JumpLowControl()
{
    this->q0.resize(3, 1);
    this->q0.setZero();

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
    this->model = gz::sim::Model(_entity);

    this->model_name = this->model.Name(_ecm);

    std::cout << this->model_name << std::endl;

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

    this->qr << -3.14 * 60 / 180, 3.14 * 120 / 180;

    this->q0 << -3.14 * 30 / 180, 3.14 * 120 / 180, 0.45;

    this->SetJointPos(_ecm, this->q0);

    if (this->_Node.Subscribe<JumpLowControl, jump::msgs::LowStates>(this->low_level_topic_name, &JumpLowControl::LowCmdCB, this))
    {
        std::cout << "The topic [" << this->low_level_topic_name << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error to create the topic [" << this->low_level_topic_name << "]" << std::endl;
    }
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

void JumpLowControl::SetJointPos(gz::sim::v8::EntityComponentManager &_ecm, Eigen::VectorXd &_q)
{
    for (int idx = 0; idx < 3; idx++)
    {
        _ecm.SetComponentData<gz::sim::v8::components::JointPositionReset>(this->model.JointByName(_ecm, this->joint_names[idx]), {(_q)(idx)});
    }
}

void JumpLowControl::LowCmdCB(const jump::msgs::LowStates &_msg)
{
    std::lock_guard<std::mutex> lock(this->JumpControlMutex);
    _toolsGz.VecMsg2VecEigen(_msg.q(), &this->qr);
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
    variavel_qlqr += 1;
    if (variavel_qlqr == 1000)
    {
        // gz::msgs::Empty empty;
        // jump::msgs::LowStates msg;

        // requisição direta no serviço dos estados - funciona
        // bool result;
        // bool executed = this->_Node.Request("/jump/low_state", empty, 5, msg, result);
        // std::cout << result << std::endl;
        // std::cout << msg.mutable_q()->data(1) << std::endl;

        // reuisitando no mesmo tópico do adapter:
        bool result;
        gz::msgs::Boolean msg;
        gz::msgs::Boolean _req;
        _req.set_data(false);
        msg.Clear();
        bool executed = _Node.Request("ser/teste", _req, 500, msg, result);
        std::cout << executed << ", " << result << ", " << msg.data() << std::endl;
        variavel_qlqr = 0;
    }
}

GZ_ADD_PLUGIN(JumpLowControl,
              gz::sim::System,
              JumpLowControl::ISystemConfigure,
              JumpLowControl::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(JumpLowControl, "JumpLowControl")
