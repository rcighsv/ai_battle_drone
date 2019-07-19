/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//#include <apriltags2_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


using namespace std;

mavros_msgs::State current_state;
ros::Publisher image_pub;
ros::Publisher image_info_pub;
geometry_msgs::PoseStamped pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void camera_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    //cout << "Inside camera_cb" << endl;
    image_pub.publish(msg);
}

void camera_info_cb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    //cout << "Inside camera_cb" << endl;
    image_info_pub.publish(msg);
}


void coord_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  cout << "inside coord_cb" << endl;
  pose.pose.position.x = msg->pose.position.x;
  pose.pose.position.y = msg->pose.position.y;
  pose.pose.position.z = msg->pose.position.z;
}


void tag_detect_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    cout << "inside tag_detect_cb" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber camera_sub = nh.subscribe<sensor_msgs::Image>
            ("iris_1/camera_down/image_raw", 10, camera_cb);

    ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>
            ("iris_1/camera_down/camera_info", 10, camera_info_cb);

    ros::Subscriber tag_detect_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>
            ("apriltag_ros/AprilTagDetectionArray", 10, &tag_detect_cb);
//ros::Subscriber apriltag_sub = nodeHandle.subscribe<apriltag_ros::AprilTagDetectionArray>
//            ("apriltag_ros/AprilTagDetectionArray", 10, &apriltag_cb);

    ros::Subscriber coord_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("keyboard_input/coord_msg", 10, coord_cb);


    image_pub = nh.advertise<sensor_msgs::Image>
            ("/camera_rect/image_rect", 10);
    image_info_pub = nh.advertise<sensor_msgs::CameraInfo>
            ("/camera_rect/camera_info", 10);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }


    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    pose.pose.orientation.x =0;
    pose.pose.orientation.y =0;
    pose.pose.orientation.z =-.99;
    pose.pose.orientation.w =-0.04;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ros::Time last_setpose = ros::Time::now();
    int count = 0;
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        if(ros::Time::now() - last_setpose > ros::Duration(15.0))
        {
//             pose.pose.position.x -= 5.0;
             //pose.pose.position.y += 0.0;
             last_setpose = ros::Time::now();
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
