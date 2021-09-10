#include "multirotor.h"
#include <thread>

Multirotor::Multirotor(std::string name)
    : m_name(name)
    , m_param_creator(MultiRotorParamsFactory::createConfig(
        AirSimSettings::singleton().getVehicleSetting("simpleflight"),
        std::make_shared<SensorFactory>()))
{
    m_kinematics.reset();
    m_environment_body.update();
    m_environment_world.update();

    spawned = true;
}

bool Multirotor::spawn()
{
    m_api = m_param_creator->createMultirotorApi();
    m_api->setName(m_name);
    m_api->setSimulatedGroundTruth(&m_kinematics.getState(), &m_environment_world);
    m_api->reset();
    m_body.reset(new MultiRotorPhysicsBody(m_param_creator.get(), m_api.get(), &m_kinematics, &m_environment_body));
    return true;
}

bool Multirotor::start()
{
    if (spawned)
    {
        m_api->enableApiControl(true);
        m_api->armDisarm(true);
        auto t = new std::thread([&] {m_api->moveToPosition(0, 0, -5, 20, 5, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, YawMode(),1,0.75); });
        
        //m_api->moveByVelocity(0, 0, 0, 5, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, YawMode());
        return true;
    }
    else
    {
        throw std::exception("œ»spawn, ‘Ÿstart£°");
    }
}
bool Multirotor::end()
{
    return true;
}

std::shared_ptr<msr::airlib::MultiRotorPhysicsBody> Multirotor::getBody()
{
    return m_body;
}

std::unique_ptr<msr::airlib::MultirotorApiBase> &Multirotor::getApi()
{
    return m_api;
}

std::string Multirotor::name()
{
    return m_name;
}