#include "handeye/handeye.hpp"
#include <sys/wait.h>
#include <pthread.h>
using namespace std;

extern bool workstate;
extern bool isDone;

//ros::NodeHandle* node_ptr = NULL;

std::mutex valMutex;

void visionCatch();
void openDobotServer();

void openYOLO();
void openRealsense();
bool signalCallback(robot_communication::signal::Request& req, robot_communication::signal::Response& res);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_eye");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("launch_signal", signalCallback);
    //node_ptr = &n;

    valMutex.lock();
    ROS_INFO("ROBOT WORKSTATE: %d", workstate);
    valMutex.unlock();    
    std::thread t0(openDobotServer);

    //system("rosrun dobot DobotServer ttyUSB0");
    sleep(5);
    
    Handeye x(n);
    x.goHome();
    //x.toPrePose();

    std::thread t(visionCatch);
    //std::thread t1(newThread);
    
    while(1) {

        ROS_INFO("waiting for work..");

        valMutex.lock();
        if( !workstate ) {
        valMutex.unlock();
            
            continue;
        } else { 
            ROS_INFO("work is coming...");
            break;
        }
    }

    cout<<"dobot begin to work!"<<endl;

    valMutex.lock();
    x.doTask();
    valMutex.unlock();
    
    while(1);
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
    //ROS_INFO("vision thread begin");
    
    ros::Rate r(30);
    ros::Time current_time;
    while ( ros::ok() )
    {
        
        valMutex.lock();
        //cout<<"id: "<<pthread_self()<<endl;
        current_time = ros::Time::now();
        ros::spinOnce();
        r.sleep();
        valMutex.unlock();
        //ROS_INFO("debug");
        //pthread_mutex_unlock(&mutex);
    }
}
void openDobotServer()
{
    system("rosrun dobot DobotServer ttyUSB0");
}
bool signalCallback(robot_communication::signal::Request& req, robot_communication::signal::Response& res)
{
    //接收到移动机器人信号后,打开相机和yolo节点
    std::thread p1(openYOLO);
    std::thread p2(openRealsense);
    
    sleep(60);//延时并等待相机完成启动

    p1.~thread();
    p2.~thread();

    valMutex.lock();
    workstate = 1;
    valMutex.unlock();

    // system("rosnode kill /darknet_ros");
    // system("rosnode kill /camera/realsense2_camera");
    // system("rosnode kill /camera/realsense2_camera_manager");

    while(1) {

        valMutex.lock();
        if( !isDone ) {
            valMutex.unlock();
            continue;
        } else {
            res.isDone = 1;
            return true;
        }

    }
    
}




