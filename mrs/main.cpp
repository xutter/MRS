#pragma comment(lib,"AirLib.lib")
#pragma comment(lib,"MavLinkCom.lib")
#pragma comment(lib,"rpc.lib")

#include "multirotor.h"
#include <winsock.h>

const double RadToDeg = 180.0 / 3.1415926;

struct SendToServer
{
    int id{ 0 };
    float data[18]{ 0 };
};

void PrintInfo(msr::airlib::Kinematics::State &kine, int id, bool allInfo = true)
{
    if (allInfo)
    {
        printf("%4d%16.2f%16.2f%16.2f%16.2f%16.2f%16.2f\n"
            , id
            , kine.pose.position.x(), kine.pose.position.y(), kine.pose.position.z()
            , kine.twist.linear.x(), kine.twist.linear.y(), kine.twist.linear.z()
            , kine.accelerations.linear.x(), kine.accelerations.linear.y(), kine.accelerations.linear.z()
            , kine.pose.orientation.x() * RadToDeg, kine.pose.orientation.y() * RadToDeg, kine.pose.orientation.z() * RadToDeg
            , kine.twist.angular.x() * RadToDeg, kine.twist.angular.y() * RadToDeg, kine.twist.angular.z() * RadToDeg
            , kine.accelerations.angular.x() * RadToDeg, kine.accelerations.angular.y() * RadToDeg, kine.accelerations.angular.z() * RadToDeg
        );
    }
    else
    {
        printf("ID=%4d    地速(m/s)=%-5.1f    垂速(m/s)=%-5.1f    三轴位移(m)=(%-8.2f,%-8.2f,%-8.2f)    姿态角(°)=(%-5.2f,%-5.2f,%-5.2f)\n"
            , id
            , sqrt(pow(kine.twist.linear.x(),2.0) + pow(kine.twist.linear.y(),2.0)), kine.twist.linear.z()
            , kine.pose.position.x(), kine.pose.position.y(), kine.pose.position.z()
            , kine.pose.orientation.x() * 180, kine.pose.orientation.y() * 180, kine.pose.orientation.z() * 180
        );
    }
}
void main(int argc, char *argv[])
{
    int num = 1;
    if (argc > 1)
    {
        num = atoi(argv[1]);
    }
    int nThread = num;
    if (argc > 2)
    {
        nThread = atoi(argv[2]);
    }
    __int64 so = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    sockaddr_in haddr{ 0 };
    haddr.sin_family = AF_INET;
    haddr.sin_port = htons(22887);
    haddr.sin_addr.S_un.S_addr = inet_addr("0.0.0.0");
    bind(so,(sockaddr *)&haddr,sizeof(sockaddr_in));

    sockaddr_in raddr{ 0 };
    raddr.sin_family = AF_INET;
    raddr.sin_port = htons(58008);
    raddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");

    // 仿真计算周期 3ms
    auto clock = std::make_shared<SteppableClock>(3E-3f);
    //auto clock = std::make_shared<SteppableClock>(0.05);
    ClockFactory::get(clock);

    // 创建默认的参数
    AirSimSettings::singleton().load([] {return "Multirotor"; });

    // 创建飞机
    std::vector<UpdatableObject*> vehicles;
    std::vector<Multirotor*> multirotors;
    for (int i = 0; i < num; i++)
    {
        char name[64]{ 0 };
        _snprintf(name, sizeof(name),"MR-%d", i);
        Multirotor* sp = new Multirotor(name);
        sp->spawn();
        multirotors.push_back(sp);
        vehicles.push_back(sp->getBody().get());
    }

    // 创建物理引擎，启动物理引擎
    for (int i = 0; i < num; i++)
    {
        multirotors[i]->start();
        //multirotors[i]->getApi()->enableApiControl(true);
        //multirotors[i]->getApi()->armDisarm(true);
        //multirotors[i]->getApi()->hover();
    }

    std::unique_ptr<PhysicsEngineBase> physics_engine(new FastPhysicsEngine());
    PhysicsWorld physics_world(std::move(physics_engine), vehicles, static_cast<uint64_t>(clock->getStepSize() * 1E9));
    //clock->sleep_for(2);

    //创建并启动rpc
    ApiProvider api_provider(nullptr);
    for (auto it : multirotors)
    {
        api_provider.insert_or_assign(it->name(),it->getApi().get(),nullptr);
    }
    msr::airlib::MultirotorRpcLibServer server(&api_provider, "0.0.0.0");
    //start server in async mode
    server.start(false, nThread);

    int index = 0;
    while (true) {
        index += 1;
        clock->sleep_for(0.1); 
        int i = 1;
        for (auto it : multirotors)
        {
            auto kine = it->getBody()->getKinematics();
            SendToServer _data{ i,
                { kine.pose.position.x(), kine.pose.position.y(), kine.pose.position.z()
                , kine.twist.linear.x(), kine.twist.linear.y(), kine.twist.linear.z()
                , kine.accelerations.linear.x(), kine.accelerations.linear.y(), kine.accelerations.linear.z()
                , kine.pose.orientation.x() * RadToDeg, kine.pose.orientation.y() * RadToDeg, kine.pose.orientation.z() * RadToDeg
                , kine.twist.angular.x() * RadToDeg, kine.twist.angular.y() * RadToDeg, kine.twist.angular.z() * RadToDeg
                , kine.accelerations.angular.x() * RadToDeg, kine.accelerations.angular.y() * RadToDeg, kine.accelerations.angular.z() * RadToDeg }
            };

            sendto(so,(const char *)&_data,sizeof(_data),0,(sockaddr *)&raddr,sizeof(raddr));
            if(index % 10 == 0)
                PrintInfo(kine,i,false);
            i += 1;
        }
    }
}
