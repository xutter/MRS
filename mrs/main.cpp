#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "physics/PhysicsWorld.hpp"
#include "physics/FastPhysicsEngine.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/SteppableClock.hpp"
#include "vehicles/multirotor/MultiRotorPhysicsBody.hpp"
#pragma comment(lib,"AirLib.lib")
#pragma comment(lib,"MavLinkCom.lib")
void main()
{
    auto clock = std::make_shared<SteppableClock>(3E-3f);
    ClockFactory::get(clock);

    //Settings& settings = Settings::loadJSonFile("settings.json");
    //unused(settings);

    SensorFactory sensor_factory;
    //Eigen::Vector3d, Eigen::Quaternion<double, Eigen::DontAlign>
    Eigen::Vector3f init_position = Eigen::Vector3f::Zero();
    Eigen::Quaternion<float, Eigen::DontAlign> init_roration{ 0,0,0,0};
    Pose init_pose{ init_position,init_roration };
    AirSimSettings::singleton().addVehicleSetting("SimpleFlight", "simpleflight", init_pose);
    std::unique_ptr<MultiRotorParams> params = MultiRotorParamsFactory::createConfig(
        AirSimSettings::singleton().getVehicleSetting("simpleflight"),
        std::make_shared<SensorFactory>());
    auto api = params->createMultirotorApi();

    std::unique_ptr<msr::airlib::Kinematics> kinematics;
    std::unique_ptr<msr::airlib::Environment> environment;
    Kinematics::State initial_kinematic_state = Kinematics::State::zero();

    initial_kinematic_state.pose = init_pose;
    kinematics.reset(new Kinematics(initial_kinematic_state));
    kinematics->reset();

    Environment::State initial_environment;
    initial_environment.position = init_position;// initial_kinematic_state.pose.position;
    initial_environment.geo_point = GeoPoint();
    environment.reset(new Environment(initial_environment));

    MultiRotorPhysicsBody vehicle(params.get(), api.get(), kinematics.get(), environment.get());

    std::vector<UpdatableObject*> vehicles = { &vehicle };
    std::unique_ptr<PhysicsEngineBase> physics_engine(new FastPhysicsEngine());
    PhysicsWorld physics_world(std::move(physics_engine), vehicles, static_cast<uint64_t>(clock->getStepSize() * 1E9));
    //physics_world.reset();


    //clock->sleep_for(0.04f);

    //Utils::getSetMinLogLevel(true, 100);

    api->setSimulatedGroundTruth(&initial_kinematic_state,environment.get());
    api->reset();
    api->enableApiControl(true);
    api->armDisarm(true);
    api->setSimulatedGroundTruth(&initial_kinematic_state, environment.get());
    api->takeoff(10);

    //clock->sleep_for(2.0f);

    //Utils::getSetMinLogLevel(true);

    api->moveToPosition(-5, -5, -5, 5, 5, DrivetrainType::MaxDegreeOfFreedom, YawMode(true, 0), -1, 0);

    //clock->sleep_for(2.0f);

    std::vector<std::string> messages_;
    while (true) {
        clock->sleep_for(0.1f);
        api->getStatusMessages(messages_);
        for (const auto& status_message : messages_) {
            std::cout << status_message << std::endl;
        }
        messages_.clear();
    }
}
