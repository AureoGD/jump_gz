#include "jump_states/jump_states.h"

JumpStates::JumpStates()
{
    this->q.resize(3, 1);
    this->q.setZero();

    this->dq.resize(3, 1);
    this->dq.setZero();

    this->tau.resize(3, 1);
    this->tau.setZero();

    this->qr.resize(2, 1);
    this->qr.setZero();

    this->dqr.resize(2, 1);
    this->dqr.setZero();
}

JumpStates::~JumpStates()
{
}

void JumpStates::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr)
{
    this->model = gz::sim::Model(_entity);

    // this->model_name = this->model.Name(_ecm);

    this->joint_entities = _ecm.ChildrenByComponents(this->model.Entity(), gz::sim::components::Joint());

    for (const gz::sim::v8::Entity &joint : this->joint_entities)
    {
        this->CreateComponents(_ecm, joint);
    }

    if (this->_Node.Advertise<JumpStates, gz::msgs::Boolean, jump::msgs::LowStates>(this->service_name, &JumpStates::JumpStatesCB, this))
    {
        std::cout << "The service [" << this->service_name << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error advertising service [" << this->service_name << "]" << std::endl;
    }

    // /////////////////////////////////////////////////////////////////

    if (this->_Node.Subscribe<JumpStates, gz::msgs::Wrench>(this->ts_topic, &JumpStates::TouchCB, this))
    {
        std::cout << "Subscribed to topic [" << this->ts_topic << "]" << std::endl;
    }

    if (this->_Node.Subscribe<JumpStates, gz::msgs::Wrench>(this->hfe_f_topic, &JumpStates::HFE_TorqueCB, this))
    {
        std::cout << "Subscribed to topic [" << this->hfe_f_topic << "]" << std::endl;
    }

    if (this->_Node.Subscribe<JumpStates, gz::msgs::Wrench>(this->kfe_f_topic, &JumpStates::KFE_TorqueCB, this))
    {
        std::cout << "Subscribed to topic [" << this->kfe_f_topic << "]" << std::endl;
    }

    // /////////////////////////////////////////////////

    if (this->_Node.Subscribe<JumpStates, jump::msgs::LowCmd>(this->low_level_topic_name, &JumpStates::LowCmdCB, this))
    {
        std::cout << "Subscribed to the topic [" << this->low_level_topic_name << "]" << std::endl;
    }
    else
    {
        std::cout << "Error to subscribe to the topic  [" << this->low_level_topic_name << "]" << std::endl;
    }
}

void JumpStates::LowCmdCB(const jump::msgs::LowCmd &_msg)
{
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    _toolsGz.VecMsg2VecEigen(_msg.qr(), &this->qr);
    _toolsGz.VecMsg2VecEigen(_msg.dqr(), &this->dqr);
}

void JumpStates::CreateComponents(gz::sim::v8::EntityComponentManager &_ecm,
                                  gz::sim::v8::Entity _joint)
{
    std::string joint_names = _ecm.ComponentData<gz::sim::components::Name>(_joint).value();

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
                                     gz::sim::v8::components::JointForce().TypeId()))
    {
        _ecm.CreateComponent(_joint, gz::sim::v8::components::JointForce());
    }
}

bool JumpStates::JumpStatesCB(const gz::msgs::Boolean &req, jump::msgs::LowStates &rep)
{
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    _toolsGz.EigenVec2VecMsg(this->q, rep.mutable_q());
    _toolsGz.EigenVec2VecMsg(this->dq, rep.mutable_dq());
    _toolsGz.EigenVec2VecMsg(this->tau, rep.mutable_tau());
    _toolsGz.EigenVec2VecMsg(this->qr, rep.mutable_qr());
    _toolsGz.EigenVec2VecMsg(this->dqr, rep.mutable_dqr());
    rep.set_fc(this->touch_st);

    return true;
}

void JumpStates::TouchCB(const gz::msgs::Wrench &msg)
{
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    if (abs(msg.force().z()) > 10)
        this->touch_st_cb = true;
    else
        this->touch_st_cb = false;
    this->touch_st = this->touch_st_cb * this->last_touch_state_cb;
    this->last_touch_state_cb = this->touch_st_cb;
}

void JumpStates::HFE_TorqueCB(const gz::msgs::Wrench &msg)
{
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    this->tau(1) = msg.torque().z();
}

void JumpStates::KFE_TorqueCB(const gz::msgs::Wrench &msg)
{
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    this->tau(2) = msg.torque().z();
}

void JumpStates::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm)
{

    if (this->joint_entities.empty())
        return;

    if (_info.paused)
        return;

    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);

    for (int idx = 0; idx < 3; idx++)
    {

        const auto *jointPositions = _ecm.Component<gz::sim::v8::components::JointPosition>(this->joint_entities[idx]);
        const auto *jointVelocity = _ecm.Component<gz::sim::v8::components::JointVelocity>(this->joint_entities[idx]);
        const auto *jointTorque = _ecm.Component<gz::sim::v8::components::JointForce>(this->joint_entities[idx]);

        if (jointPositions == nullptr || jointPositions->Data().empty() ||
            jointVelocity == nullptr || jointVelocity->Data().empty())
            return;

        this->q[idx] = jointPositions->Data()[0];
        this->dq[idx] = jointVelocity->Data()[0];
    }
}

GZ_ADD_PLUGIN(JumpStates,
              gz::sim::System,
              JumpStates::ISystemConfigure,
              JumpStates::ISystemPostUpdate);

GZ_ADD_PLUGIN_ALIAS(JumpStates, "JumpStates")