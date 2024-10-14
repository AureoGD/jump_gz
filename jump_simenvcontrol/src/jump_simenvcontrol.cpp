#include "jump_simenvcontrol/jump_simenvcontrol.h"
#include <gz/math/Rand.hh>

JumpSimEnvControl::JumpSimEnvControl()
{
    this->qaux.resize(3, 1);
    this->qaux.setZero();
    this->qmin.resize(3, 1);
    this->qmin.setZero();
    this->qmax.resize(3, 1);
    this->qmax.setZero();
}

JumpSimEnvControl::~JumpSimEnvControl() {}

void JumpSimEnvControl::Configure(const gz::sim::Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  gz::sim::EntityComponentManager &_ecm,
                                  gz::sim::EventManager &_eventMgr)
{

    std::cout << "========================== Sim Env Control ========================" << std::endl;

    this->model = gz::sim::Model(_entity);

    this->model_name = this->model.Name(_ecm);

    this->qaux << -3.14 * 50 / 180, 3.14 * 80 / 180, 0.6;

    if (_sdf->HasElement("joint_name"))
    {
        auto elem = _sdf->FindElement("joint_name");
        while (elem)
        {
            std::string jointName = elem->Get<std::string>();
            gz::sim::v8::Entity jointEntity = this->model.JointByName(_ecm, jointName);
            if (jointEntity != gz::sim::v8::kNullEntity)
            {
                this->joint_names.push_back(_ecm.ComponentData<gz::sim::components::Name>(jointEntity).value());
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

    auto dataFromSDF = _sdf->Get<std::string>("qmin");
    _toolsGz.VectorDataFromSDF(dataFromSDF, this->qmin);
    this->qmin.block(0, 0, 2, 1) = this->qmin.block(0, 0, 2, 1) * GZ_PI / 180;

    dataFromSDF = _sdf->Get<std::string>("qmax");
    _toolsGz.VectorDataFromSDF(dataFromSDF, this->qmax);
    this->qmax.block(0, 0, 2, 1) = this->qmax.block(0, 0, 2, 1) * GZ_PI / 180;

    std::cout << "===================================================================" << std::endl;
}

void JumpSimEnvControl::Reset(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm)
{
    double d = 0;
    while (d < 0.1)
    {
        qaux(0) = gz::math::Rand::DblUniform(qmin(0), qmax(0));
        qaux(1) = gz::math::Rand::DblUniform(qmin(1), qmax(1));
        qaux(2) = gz::math::Rand::DblUniform(qmin(2), qmax(2));
        d = qaux(2) - 0.25 * cos(qaux(0) + qaux(1)) - 0.2850 * cos(qaux(0)) - 0.125;
    }
    for (int idx = 0; idx < 3; idx++)
    {
        _ecm.SetComponentData<gz::sim::v8::components::JointPositionReset>(this->model.JointByName(_ecm, this->joint_names[idx]), {(qaux)(idx)});
    }
}

GZ_ADD_PLUGIN(JumpSimEnvControl,
              gz::sim::System,
              JumpSimEnvControl::ISystemConfigure,
              JumpSimEnvControl::ISystemReset);

GZ_ADD_PLUGIN_ALIAS(JumpSimEnvControl, "JumpSimEnvControl")
