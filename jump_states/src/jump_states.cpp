#include "jump_states/jump_states.h"

JumpStates::JumpStates()
{
    this->q.resize(3, 1);
    this->q.setZero();

    this->dq.resize(3, 1);
    this->dq.setZero();

    this->tau.resize(3, 1);
    this->tau.setZero();
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

    if (this->_Node.Advertise<JumpStates, gz::msgs::Boolean, gz::msgs::Boolean>("ser/teste", &JumpStates::testeCB, this))
    {
        std::cout << "The service [" << "ser/teste" << "] was created" << std::endl;
    }
    else
    {
        std::cout << "Error advertising service [" << this->service_name << "]" << std::endl;
    }
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
        std::cout << "Create a 'Force Component' for the joint "
                  << joint_names << ", component: "
                  << _joint << std::endl;
    }
}

bool JumpStates::JumpStatesCB(const gz::msgs::Boolean &req, jump::msgs::LowStates &resp_msg)
{
    // std::cout << "hei" << std::endl;
    std::lock_guard<std::mutex> lock(this->JumpStatesMutex);
    _toolsGz.EigenVec2VecMsg(this->q, resp_msg.mutable_q());
    _toolsGz.EigenVec2VecMsg(this->dq, resp_msg.mutable_dq());
    // _toolsGz.EigenVec2VecMsg(this->tau, resp_msg.mutable_tau());
    // std::cout << this->service_name << "Service CB. Data: " << resp_msg.q().data(1) << std::endl;

    return true;
}

bool JumpStates::testeCB(const gz::msgs::Boolean &req, gz::msgs::Boolean &resp_msg)
{
    if (req.data())
        std::cout << "Req true" << std::endl;

    else
        std::cout << "Req false" << std::endl;
    b_var = !b_var;
    resp_msg.set_data(b_var);
    std::cout << "'ser/teste' data: " << resp_msg.data() << std::endl;

    return true;
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

        // if (jointTorque == nullptr)
        //     std::cout << "Null" << std::endl;

        // if (!jointTorque->Data().empty())
        //     std::cout << jointTorque->Data()[0] << std::endl;
        // std::cout << jointTorque->Data().size() << std::endl;

        // if (jointPositions == nullptr || jointPositions->Data().empty() ||
        //     jointVelocity == nullptr || jointVelocity->Data().empty() ||
        //     jointTorque == nullptr || jointTorque->Data().empty())
        // {

        //     return;
        // }

        if (jointPositions == nullptr || jointPositions->Data().empty() ||
            jointVelocity == nullptr || jointVelocity->Data().empty())
            return;

        this->q[idx] = jointPositions->Data()[0];
        this->dq[idx] = jointVelocity->Data()[0];
        // this->tau[idx] = jointTorque->Data()[0];
    }
}

GZ_ADD_PLUGIN(JumpStates,
              gz::sim::System,
              JumpStates::ISystemConfigure,
              JumpStates::ISystemPostUpdate);

GZ_ADD_PLUGIN_ALIAS(JumpStates, "JumpStates")