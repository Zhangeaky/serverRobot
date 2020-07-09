#include <dobotTask/dobotTask.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <thread>
#include <mutex>

using namespace std;
bool workstate = 0;
bool isDone = 0;

extern ros::Time current_time;

class Handeye final : public dobotTask
{

public:
    Handeye(ros::NodeHandle& node):
    m_node(node), dobotTask(node)
    {
        //pthread_mutex_init(&mutex, NULL);
        //to be modified
        // //intrinx for zed2
        this->camera_matrix = (cv::Mat_<double>(3,3) <<  541.5486450195312,               0.0, 631.6287841796875,
                                                                       0.0, 541.5486450195312, 356.4523010253906,
                                                                       0.0,               0.0,                1.0);
        // //for realsense
        // this->camera_matrix = (cv::Mat_<double>(3,3) <<  614.2579345703125,               0.0,  313.2626647949219, 
        //                                                                0.0, 612.4469604492188, 241.12669372558594, 
        //                                                                0.0,               0.0,                1.0);

        this->dist_coeffs =(cv::Mat_<double>(1,5) << 0.0,0.0,0.0,0.0,0.0);

        image_transport::ImageTransport it_(m_node);
        this->m_point_stamped_publisher = this->m_node.advertise<geometry_msgs::PointStamped>("camera_point",100);
        this->m_test_point_stamped_publisher = this->m_node.advertise<geometry_msgs::PointStamped>("test_point",100);

        //for zed2
        m_image_sub = it_.subscribe("/zed2/zed_node/left/image_rect_color",100,&Handeye::callbackImage,this);
        //m_depth_image_sub = it_.subscribe("/zed2/zed_node/depth/depth_registered",100,&Handeye::alignDepthcallbackImage,this);
        this->camera_stamped_points_msgs.header.frame_id = "zed2_left_camera_optical_frame";

        //for realsense
        // m_sub_darknet_result = this->m_node.subscribe("/darknet_ros/bounding_boxes", 10, &Handeye::callbackBoundingBoxes,this);
        // m_image_sub = it_.subscribe("/camera/color/image_raw",100,&Handeye::callbackImage,this);
        // m_depth_image_sub = it_.subscribe("/camera/aligned_depth_to_color/image_raw",100,&Handeye::alignDepthcallbackImage,this);
        // this->camera_stamped_points_msgs.header.frame_id = "camera_color_optical_frame";
        // this->m_sub_point_cloud = this->m_node.subscribe("/darknet_ros/bounding_boxes", 10, &Handeye::callbackBoundingBoxes,this);
        
    }

    void newThread();

    void cameraAxisCalculation(string cLass, cv::Point2i point );
    void testCalculation();

    void callbackImage(const sensor_msgs::ImageConstPtr& msg);
    void alignDepthcallbackImage(const sensor_msgs::ImageConstPtr& msg);
    void callbackBoundingBoxes( const darknet_ros_msgs::BoundingBoxesConstPtr& msg );

    void loadCalibrationFiles(string& input_path, cv::Mat& camera_matrix, cv::Mat& distcoeffs, double scale);

    void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center);
    void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center);
    void sendMarkerTf(vector<cv::Vec3d>& marker_vecs, vector<cv::Vec3d>& marker_rotate_vecs);
    void publishTarget2BaseTF();

   // void publishWorld2BaseTF();
    void searchTF();
    void getdepth(int u, int v);

    void getPointFromPC();
  
 
public:

    bool ifdepthget = 0;

    ros::NodeHandle& m_node;
    ros::Publisher m_point_stamped_publisher;
    ros::Publisher m_test_point_stamped_publisher;

    //ros::ServiceClient client_pose;
    ros::Subscriber m_sub_darknet_result;
    ros::Subscriber m_sub_point_cloud;
    image_transport::Subscriber m_image_sub;
    image_transport::Subscriber m_depth_image_sub;

    tf::TransformBroadcaster camera_to_marker_tf_broadcaster;
    tf::TransformBroadcaster marker_to_base_tf_broadcaster;
    tf::TransformBroadcaster world_to_base_tf_broadcaster;
    tf::TransformListener    listener;
    tf::TransformListener    listener_temp;

    vector< cv::Point2f > marker_center;
    cv::Ptr< cv::aruco::Dictionary > dictionary;
    float markerLength = 0.053;
    //float markerLength = 0.0875;

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    vector< vector< cv::Point2f > > marker_corners;

    Eigen::Matrix<double,3,3> camera_matix;
    Eigen::Matrix<double,3,3> camera_matix_inverse;

    cv::Mat picture;
    cv::Mat depth_align_picture;
    vector<unsigned char> depths;

    vector< vector< cv::Point > > contours;
    vector< cv::Vec4i > hierarcy;
    vector< cv::Point2f > points;//像素坐标
    //vector< cv::Point3d > camera_points;
    vector< cv::Point3d > base_points;//机器人坐标系
    //vector< vector <cv::Point2f > > marker_corners

    geometry_msgs::PointStamped camera_stamped_points_msgs;
    geometry_msgs::PointStamped pout;

    map< string, vector< cv::Point2f > > pixel_points;
    map< string, vector< cv::Point3f > > camera_points;

    //string camera_frame = "camera_color_optical_frame";
    string camera_frame = "zed2_left_camera_optical_frame";
    vector< cv::Point3d > temp_camera_targets;
};

void Handeye::getPointFromPC()
{
    ROS_INFO("Get Point from PC");
    if( workstate ) {
        ROS_INFO("Main Thread working!");
        return;
    }
    this->temp_camera_targets.clear();
    this->temp_target_points.clear();
    tf::StampedTransform storage;
    string parenet_id;
    ros::Time time = ros::Time::now();
    try {
        bool a = this->listener_temp.waitForTransform(camera_frame,"drug",  ros::Time(0), ros::Duration(3.0));
        if( a == 0 ) {
            ROS_ERROR("THE CAR HAS NOT ARRIVED, POINTCLOUD NODE IS NOT READY");
        }
        this->listener_temp.lookupTransform(camera_frame,"drug",  ros::Time(0), storage);
    } catch ( tf::TransformException& e) {

        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
        return;
    }
    

    this->temp_camera_targets.push_back(cv::Point3d(storage.getOrigin().getX(), storage.getOrigin().getY(),storage.getOrigin().getZ()));
    geometry_msgs::PointStamped pin;
    pin.header.frame_id = camera_frame;
    pin.header.stamp = time;
    pin.point.x = storage.getOrigin().getX();
    pin.point.y = storage.getOrigin().getY();
    pin.point.z = storage.getOrigin().getZ(); 

    geometry_msgs::PointStamped pout;  

    try
    {
        this->listener_temp.waitForTransform(camera_frame,"magician_origin",  ros::Time(0), ros::Duration(3.0));
        this->listener_temp.transformPoint("magician_origin", ros::Time(0),  pin, this->camera_frame, pout);    
    }
    catch( tf::TransformException& e )
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
        return;
    }

    //发布时间戳点在Rviz中验证准确性
    m_test_point_stamped_publisher.publish(pout);
    this->temp_target_points.push_back(cv::Point3d( pout.point.x, pout.point.y, pout.point.z ));
    cout<<"存点完毕: "<<endl;

    //cout<<"a: "<<a<<endl;
    //this->listener_temp.lookupTransform(camera_frame,"drug",  time, storage);
    
    cout<<"x: "<<temp_target_points[0].x<<endl;   
    cout<<"y: "<<temp_target_points[0].y<<endl;
    cout<<"z: "<<temp_target_points[0].z<<endl;

   
    //this->listener_temp.getParent("drug",ros::Time::now(), parenet_id);f
    //cout<<"frame_id: "<<parenet_id<<endl;
}

void Handeye::searchTF()
{   
    this->temp_camera_targets.clear();

    tf::StampedTransform transform;
    try{

        listener.lookupTransform("", "/turtle1",
                                ros::Time(0), transform);
    }catch ( tf::TransformException ex ){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void Handeye::callbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("color image topic subscribed!");
    if ( workstate ) {
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    getMarker(cv_ptr->image,this->marker_center);
    this->picture = cv_ptr->image.clone();
    this->getPointFromPC();
    //this->testCalculation();
}

void Handeye::alignDepthcallbackImage(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("aligned depth topic subscribed!");
    this->ifdepthget = 1;
    //cout<<"isworking:"<<this->isworking<<endl;
    if ( workstate ) {
        //ROS_INFO("主线程正在工作");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); 
    }

    catch (cv_bridge::Exception& e)
    {
        ("cv_bridge exception: %s", e.what());
        return;
    }
    this->depths = msg->data;
   
    //int centerIdx = 100 + msg->width * 125;
    //cout<<"frame: "<<msg->header.frame_id<<endl;
    this->depth_align_picture = cv_ptr->image;
    // cout<<"w: "<<msg->width<<endl;
    // cout<<"h: "<<msg->height<<endl;
    //ROS_INFO("Center distance : %g m", this->depths[centerIdx]);
    //cv::medianBlur(this->depth_align_picture, this->depth_align_picture, 3);
    //cout<<"------end------"<<endl;
}

void Handeye::getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center)
{
    vector<int> ids;
    vector<cv::Vec3d> rotate_vecs, trans_vecs;

    this->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    if( !marker_image.empty() ) {

        cv::aruco::detectMarkers(marker_image, dictionary, this->marker_corners, ids);
        cv::aruco::drawDetectedMarkers(marker_image, this->marker_corners, ids);
        cv::aruco::estimatePoseSingleMarkers(this->marker_corners, this->markerLength, this->camera_matrix, this->dist_coeffs, rotate_vecs, trans_vecs);//0.1645

        if ( rotate_vecs.empty()&&trans_vecs.empty() ) {

           // ROS_ERROR("No Marker detected!!!");
            return;

        } else {

            //cout << "二维码数量:" << ids.size() << endl;
            //cout << "ID of the marker is: " << ids[0] << endl;
            //cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs ,rotate_vecs, trans_vecs, 0.1);
            //getMarkerCoordinate(corners, ids, marker_center);
        }

        cv::Mat pic = marker_image.clone();

        for ( int i = 0; i < this->marker_corners[0].size(); i++ ) {

            cv::circle( pic, marker_corners[0][i], 2, (255,0,0), 5 );
        }

        // cv::imshow("maker", pic);
        // cv::waitKey(1);
            //cout<<trans_vecs[0]<<endl;

            //cv::circle(marker_image,point2,2,(255,0,0),5);
            //char num[10];
            //sprintf(num,"logitech",camera_index_);
            sendMarkerTf(rotate_vecs,trans_vecs);
    }else{
        ROS_ERROR("check your camera device!");
        return;
    }
}

void Handeye::getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center)
    {
        cv::Point2f center(0.f, 0.f);
        for(int i = 0;i < corners[0].size();i++)
            {
                center += corners[0][i];
            }
        center /= 4.0;
        marker_center.push_back(center);
        //cout<<marker_center[0].x<<","<<marker_center[0].y<<endl;
    }

void Handeye::sendMarkerTf(vector<cv::Vec3d>& marker_rotate_vecs,vector<cv::Vec3d>& marker_trans_vecs)
{
    if( marker_rotate_vecs.size()==0&&marker_rotate_vecs.size()==0 ) {

            cout<<"haven't received any vecs yet"<<endl;

    } else {

            cv::Mat rotated_matrix(3, 3, CV_64FC1);

            cv::Rodrigues(marker_rotate_vecs[0],rotated_matrix);
            rotated_matrix.convertTo(rotated_matrix, CV_64FC1);

            tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<double>(0,0), rotated_matrix.at<double>(0,1),rotated_matrix.at<double>(0,2),
                                rotated_matrix.at<double>(1,0), rotated_matrix.at<double>(1,1), rotated_matrix.at<double>(1,2),
                                rotated_matrix.at<double>(2,0), rotated_matrix.at<double>(2,1), rotated_matrix.at<double>(2,2));

            tf::Vector3 tf_tvec(marker_trans_vecs[0][0],marker_trans_vecs[0][1],marker_trans_vecs[0][2]);

            tf::Transform transform(tf_rotated_matrix, tf_tvec);
            this->camera_to_marker_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), this->camera_frame, "target_marker"));
            this->publishTarget2BaseTF();
    }
}

void Handeye::publishTarget2BaseTF()
{
    tf::Transform transform;
    tf::Quaternion q;

    //transform.setOrigin( tf::Vector3( -0.0195, 0.2635, 0.0000) );
    transform.setOrigin( tf::Vector3( 0.0155, 0.1105, 0.0000) );

    q.setRPY(0.00,0.00,-3.14);
    transform.setRotation(q);

    this->marker_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"target_marker","magician_base"));

    transform.setOrigin( tf::Vector3( 0.0000, 0.0000, 0.124724) );

    q.setRPY(0.00,0.00,0.00);
    transform.setRotation(q);

    this->marker_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"magician_base","magician_origin"));
}

// void Handeye::publishWorld2BaseTF()
// {
//     tf::Transform transform_;
//     tf::Quaternion q_;
//     q_.setRPY(0.00,0.00,0.00);
//     transform_.setRotation(q_);
//     transform_.setOrigin( tf::Vector3( 0.0000, 0.0000, -0.127424) );
//     this->world_to_base_tf_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(),"world","magician_base"));
// }


void Handeye::testCalculation( )
{
    cout<<"test"<<endl;

    if ( marker_corners.size() == 0) return;

    for ( int i = 0; i < this->marker_corners[0].size(); i++ ) {

        cout<< "二维码角点: "<< marker_corners[0].size() <<endl;

        if( !ifdepthget ) return;

        //for zed2
        int centerIdx = (int)(marker_corners[0][i].x) + 1280 * (int)(marker_corners[0][i].y);
        float* depths_ptr = (float*)(&this->depths[0]);
        float Zc = (float)(depths_ptr[centerIdx]);
        int value = this->depth_align_picture.at<int>((int)(marker_corners[0][i].y),(int)(marker_corners[0][i].x));
       
        //float Zc = this->depth_align_picture.at<unsigned char>(marker_corners[0][i].y, marker_corners[0][i].x);
        cout<<"第"<<i<<"个角点的深度值是"<<Zc<<" m"<<endl;

        //Zc /= 1000;
        double Xcam = ( marker_corners[0][i].x - this->camera_matrix.at<double>(0,2) )*Zc*( 1/this->camera_matrix.at<double>(0,0) );
        double Ycam = ( marker_corners[0][i].y - this->camera_matrix.at<double>(1,2) )*Zc*( 1/this->camera_matrix.at<double>(1,1) );
        float  Zcam = Zc;

        camera_stamped_points_msgs.point.x = Xcam;
        camera_stamped_points_msgs.point.y = Ycam;
        camera_stamped_points_msgs.point.z = Zc;
        camera_stamped_points_msgs.header.stamp = ros::Time::now();
        //this->m_point_stamped_publisher.publish(camera_stamped_points_msgs);
        
        geometry_msgs::PointStamped pout;

        try {
            // this->listener.waitForTransform("camera_color_optical_frame", "target_marker",
            //               ros::Time(0), ros::Duration(3.0));
            this->listener.transformPoint("target_marker", ros::Time(0),  camera_stamped_points_msgs, this->camera_frame, pout);
        } catch ( tf::TransformException &ex ) {

            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        this->m_test_point_stamped_publisher.publish(pout);

        cout<<" pout published ! "<<endl;

        // this->m_point_stamped_publisher.publish(pout);
        // cout<< "x: " <<pout.point.x << "y: " << pout.point.y << "z: " << pout.point.z <<endl;
    }
}

void Handeye::callbackBoundingBoxes( const darknet_ros_msgs::BoundingBoxesConstPtr& msg )
{
    //cout<<"isworking:"<<this->isworking<<endl;
    if ( workstate ) {
        return;
    }

    if ( this->ifdepthget == 0 ) return;

    this->pixel_points.clear();
    this->camera_points.clear();
    this->target_points.clear();
    this->assortment.clear();

    //cout<<"检测到的物体的个数是: "<<msg->bounding_boxes.size()<<endl;

    //遍历所有检测到的物体的中心点
    int counter_cell_phone = 0;
    int counter_yunnaobaiyao = 0;

    for ( int i = 0; i < msg->bounding_boxes.size(); i++ ) {

        if( counter_yunnaobaiyao == 0 ) this->assortment.push_back( "yunnaobaiyao" );
        counter_yunnaobaiyao++;
        //如果是scissors
        //计算出像素中心,和相机坐标
        if ( msg->bounding_boxes[i].Class == "yunnaobaiyao" ) {

            cv::Point2i point( (int)(( msg->bounding_boxes[i].xmax + msg->bounding_boxes[i].xmin )*0.5 ),
            (int)(( msg->bounding_boxes[i].ymax + msg->bounding_boxes[i].ymin )*0.5)
            );//得到像素坐标

            this->pixel_points["yunnaobaiyao"].push_back(point);
            this->cameraAxisCalculation( "yunnaobaiyao", point);
        }

        else if ( msg->bounding_boxes[i].Class == "cell phone" ) {

            if( counter_cell_phone == 0 ) this->assortment.push_back( "cell phone" );
            counter_cell_phone++;
            //cout<<"cell phone detected!"<<endl;

        cv::Point2i point( (int)(( msg->bounding_boxes[i].xmax + msg->bounding_boxes[i].xmin )*0.5),
          (int)(( msg->bounding_boxes[i].ymax + msg->bounding_boxes[i].ymin )*0.5)
          );

          this->pixel_points["cell phone"].push_back(point);
          this->cameraAxisCalculation( "cell phone", point);
        }
        //cout<<this->pixel_points["scissors"][0]<<endl
}
}

// this function is really very important!!!!!!
void Handeye::cameraAxisCalculation(string cLass, cv::Point2i point )
{
    //cout<<"calculate!!"<<endl;
    
    //for realsense
    float Zc = this->depth_align_picture.at< unsigned short >( point.y, point.x );
    Zc = Zc/1000;
    //cout<<"深度值:　"<<Zc<<endl;

    //for ZED2
    // int centerIdx = (int)(point.x) + 1280 * (int)(point.y);
    // float* depths_ptr = (float*)(&this->depths[0]);
    // float Zc = (float)(depths_ptr[centerIdx]);
    // cout<<"depth of the "<<cLass<<" is "<<Zc<<endl;
 

    double Xcam = ( point.x - this->camera_matrix.at<double>(0,2) )*Zc*( 1/this->camera_matrix.at<double>(0,0) );
    double Ycam = ( point.y - this->camera_matrix.at<double>(1,2) )*Zc*( 1/this->camera_matrix.at<double>(1,1) );
    float  Zcam = Zc;

    this->camera_points[cLass].push_back( cv::Point3f( Xcam, Ycam, Zcam ) );

    ros::Time now = ros::Time::now();
    geometry_msgs::PointStamped pin;
    pin.header.frame_id = this->camera_frame;


    pin.header.stamp = now;

    //cout<<"camera point: "<< Xcam <<" : "<< Ycam <<" : "<< Zcam <<endl;

    pin.point.x = Xcam;
    pin.point.y = Ycam;
    pin.point.z = Zcam;
    this->m_point_stamped_publisher.publish(pin);

    geometry_msgs::PointStamped pout;

    try {

        // this->listener.waitForTransform("camera_color_optical_frame", "target_marker",
        //               ros::Time(0), ros::Duration(3.0));
        this->listener.transformPoint("magician_origin", ros::Time(0),  pin, this->camera_frame, pout);

    } catch ( tf::TransformException &ex ) {

        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();

    }

    this->target_points[cLass].push_back( cv::Point3d( pout.point.x, pout.point.y, pout.point.z ) );

   // this->main_points[cLass].push_back(pout);
    //this->m_point_stamped_publisher.publish(pout);
}




