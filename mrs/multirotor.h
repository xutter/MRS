#pragma once
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/SteppableClock.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include <memory.h>
#include <string>

class Multirotor
{
public:
private:
    std::string m_name;
    msr::airlib::Kinematics m_kinematics;
    msr::airlib::Pose m_init_pose{ {0,0,-100},msr::airlib::Quaternionr::Identity() };
    msr::airlib::Environment::State m_initial_environment{ m_init_pose.position, GeoPoint(47.641468, -122.140165, 122) };
    msr::airlib::Environment m_environment_body{ m_initial_environment };
    msr::airlib::Environment m_environment_world{ m_initial_environment };
    std::shared_ptr<msr::airlib::MultiRotorPhysicsBody> m_body{ nullptr };
    std::unique_ptr<msr::airlib::MultirotorApiBase> m_api{ nullptr };
    std::shared_ptr<MultiRotorParams> m_param_creator;
    bool spawned{ false };

public:
    Multirotor(std::string name);
    bool spawn();
    bool start();
    bool end();
    std::shared_ptr<msr::airlib::MultiRotorPhysicsBody> getBody();
    std::unique_ptr<msr::airlib::MultirotorApiBase> &getApi();
    std::string name();

};