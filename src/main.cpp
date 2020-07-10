#include "handeye/handeye.hpp"
#include <sys/wait.h>
#include <pthread.h>

using namespace std;

extern bool workstate;
extern bool isDone;

Handeye* hand_eye_ptr = NULL;
ros::Time current_time;
std::mutex valMutex;

void visionCatch();
void openDobotServer();
void openYOLO();
void openRealsense();
bool signalCallback(robot_communication::SignalLaunch::Request& req, robot_communication::SignalLaunch::Response& res);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_eye");

    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("launch_signal", signalCallback);


    valMutex.lock();
    ROS_INFO("ROBOT WORKSTATE: %d", workstate);
    valMutex.unlock();  

    std::thread t0(openDobotServer);

    sleep(5);
    
    Handeye x(n);

    hand_eye_ptr = &x;

    //接收zed图像话题
    std::thread t(visionCatch);
   
    while( ros::ok() );
}
//
void openYOLO()
{
    system("/home/nvidia/catkin_d415/src/robot/axis_tf/src/camera_launch.sh");
}
void openRealsense()
{
    system("/home/nvidia/catkin_d415/src/robot/axis_tf/src/yolo_launch.sh");
}
void visionCatch()
{
    ros::Rate r(30);
    while ( ros::ok() )
    {
        ROS_INFO("vision thread begin!");
        valMutex.lock();
        current_time = ros::Time::now();
        ros::spinOnce();// 在其中执行回调函数不能使用互斥锁
        r.sleep();
        valMutex.unlock();
    }
}

void openDobotServer()
{
    system("/home/nvidia/catkin_d415/src/ARM_PART/robot/axis_tf/src/dobot.sh");
}

bool signalCallback(robot_communication::SignalLaunch::Request& req, robot_communication::SignalLaunch::Response& res)
{
    // std::thread p1(openYOLO);    
    //停止订阅,进入工作状态
    workstate = 1;
    if ( req.decision == 0 ) {
        hand_eye_ptr->doTempTask();
    } else {
        hand_eye_ptr->backhome();
        hand_eye_ptr->toPrePose();
        workstate = 0;
    }
    workstate = 0;
    return true; 
}




