#include "handeye/handeye.hpp"
#include <sys/wait.h>
#include <pthread.h>

using namespace std;

extern bool workstate;
extern bool isDone;
Handeye* hand_eye_ptr = NULL;
ros::Time current_time;

//ros::NodeHandle* node_ptr = NULL;

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
    std::thread t(visionCatch);

    //while(1);
    cout<<" wait !"<<endl;
    
    // while(1) {
        
    //     valMutex.lock();
    //     //cout<<"workstate: "<<workstate<<endl;
    //     bool temp = workstate;
    //     valMutex.unlock();
    //     if( !temp ) {
    //         continue;
    //     } else { 
    //         ROS_INFO("work is coming...");
    //         break;
    //     }
    // }
 
    // cout<<"dobot begin to work!"<<endl;

    //workstate = 1;

    // valMutex.lock();
    // //x.doTask();
    // //x.doTempTask();
    // isDone = 1;
    // valMutex.unlock();

    //exit(0);
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
  
    
    ros::Rate r(30);
    while ( ros::ok() )
    {
        ROS_INFO("vision thread begin");
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
    system("/home/nvidia/catkin_d415/src/ARM_PART/robot/axis_tf/src/dobot.sh");
}
bool signalCallback(robot_communication::SignalLaunch::Request& req, robot_communication::SignalLaunch::Response& res)
{
    //此函数内不能使用线程锁
    cout<<"callback"<<endl;

    system("/home/nvidia/catkin_d415/src/robot/axis_tf/src/cluster.sh");

    sleep(15);
   
    // std::thread p1(openYOLO);
    // std::thread p2(openRealsense);
    
    //sleep(10);//延时并等待相机完成启动

    // p1.~thread();
    // p2.~thread();
    ///valMutex.lock();
    workstate = 1;

    if ( req.decision == 0 ) {

        hand_eye_ptr->doTempTask();

    } else {

        hand_eye_ptr->backhome();
        hand_eye_ptr->toPrePose();
        workstate = 0;
    }
   
    return true; 

}




