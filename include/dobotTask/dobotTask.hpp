#include <ros/ros.h>
//#include "geometry_msgs/PointStamped.h"
#include <map>
#include <opencv2/core/core.hpp>
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/GetDeviceSN.h"
#include "dobot/GetAlarmsState.h"
#include "dobot/ClearAllAlarmsState.h"
#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/GetEndEffectorParams.h"
#include "dobot/GetEndEffectorGripper.h"
#include <dobot/SetEndEffectorSuctionCup.h>
#include <dobot/GetHOMEParams.h>
#include <dobot/SetHOMECmd.h>
#include <dobot/GetPose.h>

#include <robot_communication/signal.h>
#include <robot_communication/SignalLaunch.h>

using namespace std;



class dobotTask
{
    public:
    ros::NodeHandle node;
    //ros::ServiceServer server = this->node.advertiseService("Signal_to_work", &dobotTask::launch, this);
    ros::ServiceClient client_Jump_param = node.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
    ros::ServiceClient client_PTP = this->node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    ros::ServiceClient client_sup =  node.serviceClient<dobot::SetEndEffectorSuctionCup>("/DobotServer/SetEndEffectorSuctionCup");
    ros::ServiceClient client_pose =  this->node.serviceClient<dobot::GetPose>("/DobotServer/GetPose");;
    ros::ServiceClient client_pre = this->node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    ros::ServiceClient m_clear_queue = this->node.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    ros::ServiceClient m_alarm_state = this->node.serviceClient<dobot::GetAlarmsState>("/DobotServer/GetAlarmsState");
    ros::ServiceClient m_alarm_clear = node.serviceClient<dobot::ClearAllAlarmsState>("/DobotServer/ClearAllAlarmsState");
    ros::ServiceClient client_end_params = node.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    ros::ServiceClient m_get_pose = node.serviceClient<dobot::GetPose>("/DobotServer/GetPose");
    ros::ServiceClient m_home_client = node.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
    ros::ServiceClient m_PTP_client = node.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    ros::ServiceClient m_suction_client = node.serviceClient<dobot::SetEndEffectorSuctionCup>
                                                    ("/DobotServer/SetEndEffectorSuctionCup");
    ros::ServiceClient m_endeffector_params;

    ros::ServiceClient get_bias = node.serviceClient<dobot::GetEndEffectorParams>
                                                    ("/DobotServer/GetEndEffectorParams");


    vector<int> current_pose;
    vector<string> assortment;
    map <string, vector< cv::Point3d > > target_points;

    vector<cv::Point3d> home_points;

    vector<cv::Point3d> temp_target_points;

    bool isworking = 0;
    bool isDone = 0;

    public:
    dobotTask( ros::NodeHandle& node ) : node(node){
        this->alarmClear();
        this->goHome();
        this->toPrePose();
        this->dobotParamInit();
        dobot::GetEndEffectorParams srv;
        this->get_bias.call(srv);
        cout<<"params"<<srv.response.xBias<<"  "<<srv.response.yBias<<" "<<srv.response.zBias<<" "<<endl;
    }

    ~dobotTask() = default;

    void getDeviceSN();
    void goHome();

    void alarmState();
    void alarmClear();

    void updateCurrentPose();
    void goToPoint( float x, float y, float z );
    bool pointLimitJudge( float x, float y, float z );

    void dobotParamInit();
    void pick();
    void place();

    void ifDestination();

    void toPrePose();
    void doTask();
    void doTempTask();

    void clearEndParams();

    void backhome();
    
};

void dobotTask::backhome()
{
    this->pick();
    this->clearEndParams();
    this->goToPoint(209, -10, 14);
    this->place();
    //this->goHome();
}

void dobotTask::clearEndParams()
 {
    dobot::SetEndEffectorParams srv;
    srv.request.isQueued = 1;
    srv.request.xBias = 0;
    srv.request.yBias = 0;
    srv.request.zBias = 0;
    // srv.request.xBias = 61;
    // srv.request.yBias = 0;
    // srv.request.zBias =20;
    client_end_params.call(srv);
 }

void dobotTask::doTempTask()
{
    cout<<"药盒数量: "<<this->temp_target_points.size()<<endl;
    for(int i = 0; i < this->temp_target_points.size() ; i++)
        {
            cout<<"抓取第 "<<i<<" 个"<<endl;
            
            cv::Point3f x;
            
            x.x = (float)this->temp_target_points[i].x;
            x.y = (float)this->temp_target_points[i].y;
            x.z = (float)this->temp_target_points[i].z;
            cout<<"下一个点的坐标: "<<x.x*1000<<" : "<<x.y*1000<<" : "<<x.z*1000<<endl;
            //sleep(3);

            if( this->pointLimitJudge( x.x*1000, x.y*1000, x.z*1000 ) ){

                ROS_INFO("NO LIMITING");

                this->goToPoint( x.x*1000 -10, x.y*1000, x.z*1000 + 20 );///TRICK HERE!!!
            }
            
            this->pick();
            this->clearEndParams();

            this->goToPoint( -71.0, -142.00, -33.00 );

            this->place();
            
            //sleep(5); 
        }
        cout<<"Task finished!"<<endl; 
        
} 

void dobotTask::toPrePose()
{

    cout<<"Go to prePose"<<endl;
    dobot::SetPTPCmd srv_P;

    srv_P.request.ptpMode = 1;

    // srv.request.x = 207;
    // srv.request.y = -75;
    // srv.request.z = -3;
    srv_P.request.x =    -86;
    srv_P.request.y =   -167;
    srv_P.request.z =   46.8;
    client_pre.call(srv_P);
    cout<<"PrePose Answer: "<<srv_P.response.result<<endl;
}

void dobotTask::updateCurrentPose()
{
    dobot::GetPose srv;
    this->client_pose.call(srv);
    this->current_pose.push_back(static_cast<int>(srv.response.x));
    this->current_pose.push_back(static_cast<int>(srv.response.y));
    this->current_pose.push_back(static_cast<int>(srv.response.z));
}

void dobotTask::goHome()
{
    dobot::SetHOMECmd srv;
    this->m_home_client.call(srv);

    if( srv.response.result == 0 ){
        cout<<"回零完成!"<<endl;
    } else {
        cout<<"回零出错!"<<endl;
    }

}

void dobotTask::goToPoint( float x, float y, float z )
{
    dobot::SetPTPCmd srv;
    srv.request.ptpMode = 0;//joint-jump mode
    cout<<"target: "<< x <<" : "<< y <<" : "<< z << endl;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    this->client_PTP.call(srv);
   
}

void dobotTask::dobotParamInit()
{
    //末端偏移参数设置初始化
    //this->client_end_params = node.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv;
    srv.request.isQueued = 1;
    // srv.request.xBias = 0;
    // srv.request.yBias = 0;
    // srv.request.zBias = 0;
    srv.request.xBias = 61;
    srv.request.yBias = 0;
    srv.request.zBias = 0;
    client_end_params.call(srv);

    //jump抬升高度
    dobot::SetPTPJumpParams srv_;
    srv_.request.jumpHeight = 30;
    srv_.request.isQueued = true;
    this->client_Jump_param.call(srv_);
}

bool dobotTask::pointLimitJudge( float x, float y, float z )
{
    if( (x*x + y*y <= 325*325) && (  x*x + y*y >= 188*188) ) {

        return true;

    } else {

        cout<<"can't reach that point"<<endl;
        return false; 
    }
}

void dobotTask::alarmState()
{
    this->m_alarm_state = node.serviceClient<dobot::GetAlarmsState>("/DobotServer/GetAlarmsState");
    dobot::GetAlarmsState srv;
    this->m_alarm_state.call(srv);

    cout<<"alarm_size: "<<srv.response.alarmsState.size()<<endl;

    for ( int i = 0; i < srv.response.alarmsState.size(); i++ ) {

        cout<<srv.response.alarmsState[i]<<endl;

    }

}

void dobotTask::alarmClear()
{
    this->m_clear_queue = this->node.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv;
    m_clear_queue.call(srv);   

    this->m_alarm_clear = this->node.serviceClient<dobot::ClearAllAlarmsState>("/DobotServer/ClearAllAlarmsState");
    dobot::ClearAllAlarmsState srv_alarm;
    this->m_alarm_clear.call(srv_alarm);
    //cout<<"clearAlarm result :"<<srv.response.result<<endl;
}

void dobotTask::pick()
{
    dobot::SetEndEffectorSuctionCup srv;
    srv.request.enableCtrl = 1;//使能
    srv.request.suck = 1; //吸气
    srv.request.isQueued = 1; // 加入指令队列
    client_sup.call(srv);
}

void dobotTask::place()
{
    dobot::SetEndEffectorSuctionCup srv;
    srv.request.enableCtrl = 0;
    srv.request.suck = 0; //吸气
    client_sup.call(srv);
}

void dobotTask::doTask()
{
    //药品(物品)的种类cell phone, 
    for(int i = 0; i < this->assortment.size(); i++){
        cout<<"now grab "<<this->assortment[i]<<endl;
        sleep(5);
    //每一个种类的数量,每一个物体本身
        for(int j = 0; j < this->target_points[assortment[i]].size(); j++)
            {
                cout<<"抓取第 "<<j<<" 个"<<this->assortment[i]<<endl;
                
                cv::Point3f x;
                
                x.x = (float)this->target_points[assortment[i]][j].x;
                x.y = (float)this->target_points[assortment[i]][j].y;
                x.z = (float)this->target_points[assortment[i]][j].z;
                cout<<"下一个点的坐标: "<<x.x*1000<<" : "<<x.y*1000<<" : "<<x.z*1000<<endl;
                sleep(10);

                if( this->pointLimitJudge( x.x*1000, x.y*1000, x.z*1000 ) ){

                    this->goToPoint( x.x*1000, x.y*1000, x.z*1000 );
                }
                
                this->pick();

                this->goToPoint( 0.00, -200.00, 40.00 );

                this->place();
                
                sleep(5); 
            }
    } 

    cout<<"Task finished!"<<endl; 
}


