#include <iostream>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace unitree::common;
using namespace unitree::robot;



// #define TOPIC_SPORT_STATE "rt/odommodestate"//high frequency
// #define TOPIC_SPORT_LF_STATE "rt/lf/odommodestate"//low frequency
#define TOPIC_LOWSTATE "rt/lowstate"

class Custom
{
public:
    explicit Custom()
    {}

    ~Custom()
    {}

    void Init();

private:

    // /*high frequency message handler function for subscriber*/
    // void HighFreOdomMessageHandler(const void* messages);
    // /*low frequency message handler function for subscriber*/
    // void LowFreOdomMessageHandler(const void* messages);

    void LowStateMessageHandler(const void* messages);

private:

    // unitree_go::msg::dds_::SportModeState_ estimator_state{};
    // unitree_go::msg::dds_::SportModeState_ lf_estimator_state{};

    unitree_go::msg::dds_::LowState_ low_state{};  // default init
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    // ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> estimate_state_subscriber;
    // ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> lf_estimate_state_subscriber;
};


void Custom::Init()
{
    /*create subscriber*/
    // estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_STATE));
    // estimate_state_subscriber->InitChannel(std::bind(&Custom::HighFreOdomMessageHandler, this, std::placeholders::_1), 1);

    // /*create subscriber*/
    // lf_estimate_state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_SPORT_LF_STATE));
    // lf_estimate_state_subscriber->InitChannel(std::bind(&Custom::LowFreOdomMessageHandler, this, std::placeholders::_1), 1);

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;

    std::cout << "joint info: " << std::endl;
    for (int i = 0; i < 12; i++)
    {
        std::cout << "joint " << i << " position: " << low_state.motor_state()[i].q() << std::endl;
        std::cout << "joint " << i << " velocity: " << low_state.motor_state()[i].dq() << std::endl;
        std::cout << "joint " << i << " acceleration: " << low_state.motor_state()[i].ddq() << std::endl;
        std::cout << "joint " << i << " Torque: " << low_state.motor_state()[i].tau_est() << std::endl;
    }

    
    std::cout << "eular angle info: " << std::endl;
    std::cout << "x: " << low_state.imu_state().rpy()[0] << std::endl;
    std::cout << "y: " << low_state.imu_state().rpy()[1] << std::endl;
    std::cout << "z: " << low_state.imu_state().rpy()[2] << std::endl;


    std::cout << "Quaternion info: " << std::endl;
    std::cout << "w: " << low_state.imu_state().quaternion()[0] << std::endl;
    std::cout << "x: " << low_state.imu_state().quaternion()[1] << std::endl;
    std::cout << "y: " << low_state.imu_state().quaternion()[2] << std::endl;
    std::cout << "z: " << low_state.imu_state().quaternion()[3] << std::endl;
    
    std::cout<<"Imu gyroscope : "<<"x: "<<low_state.imu_state().gyroscope()[0]<<" y: "<<low_state.imu_state().gyroscope()[1]<<" z: "<<low_state.imu_state().gyroscope()[2]<<std::endl;
    std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
}


int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();

    while (1)
    {
        sleep(10);
    }

    return 0;
}