#include <iostream>
#include <thread>
#include <signal.h>
#include <string.h>
#include <chrono>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "DeltaRobotControl.h"


bool bExit = false;
//define a delta robot object
DeltaRobot::DeltaRobotControl robot;
//joy stick data



/**
 * processSignal function
 */
void processSignal(int sign)
{
    bExit = true;
}


/**
 * velocity control function
 */
void velocitycontrolthread(DeltaRobot::DeltaRobotControl *handles)
{
    while(!bExit)
    {
        if(!handles->VelocityControlLoop())
        {
            ROS_INFO("velocity control failed, it may the robot out of the workspace!");
        }
    }
}


/**
 * callback function
 */
/*void SetTwistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    DeltaRobot::v4DVelocity v4dvel;
    v4dvel.Velocity.Vx = msg->linear.x; v4dvel.Velocity.Vy = msg->linear.y; v4dvel.Velocity.Vz = msg->linear.z;
    v4dvel.YawAngleRate = msg->angular.z;
    //delta velocity control
    robot.VelocityControl(v4dvel);
}*/


/**
 * get joy data callback function
 */
void GetJoyDataCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    double reduce_ratio = 0.3;
    DeltaRobot::v4DVelocity v4dvel;
    std::vector<float> axes = msg->axes;
    v4dvel.Velocity.Vx = reduce_ratio * axes.at(0);
    v4dvel.Velocity.Vy = reduce_ratio * axes.at(1);
    v4dvel.Velocity.Vz = reduce_ratio * axes.at(3);
    v4dvel.YawAngleRate = 5 * reduce_ratio * axes.at(2);
    //delta velocity control
    robot.VelocityControl(v4dvel);
}


/**
 * main function
 */
int main(int argc, char **argv)
{
    //Associate program interrupt signals and call processSignal when you end the program with ctrl-c
    signal(SIGINT, processSignal);
    // ros init
    ros::init(argc, argv, "DeltaRobot");
    //ros handle
    ros::NodeHandle n;

    //open
    robot.Open();
    //printf robot information
    robot.PrintfRobotInfomation();
    //set the robot in position mode
    robot.SetControlMode(DeltaRobot::ROBOT_MODE_POSITION);
    //move robot in zero position
    robot.JointControl(DeltaRobot::v4DJoint(0, 0, 0, 0));

    //subscribe the target twist
    //ros::Subscriber settwist_sub = n.subscribe<geometry_msgs::Twist>("/DeltaRobot/SetTwist", 1000, SetTwistCallback);
    //subscribe joy stick data
    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1000, GetJoyDataCallback);
    //publish the robot joint data
    ros::Publisher jointstate_pub = n.advertise<sensor_msgs::JointState>("/DeltaRobot/JointState", 1000);
    //publish the robot pose data
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/DeltaRobot/Pose", 1000);

    //creat a velocity control thread and start
    std::thread velstl_thread(velocitycontrolthread, &robot);
    velstl_thread.detach();
    ROS_INFO("start the velocity control...");

    //get the data refresh frequenct
    int rate;
    if(!n.param("/startvelocitycontrol/fixed_rate", rate, 50))
    {
        ROS_INFO("cannot get the <fixed_rate> param, use the default value: %d", rate);
    }else
    {
        ROS_INFO("<fixed_rate> param use the setting value: %d", rate);
    }
    
    ros::Rate loop_rate(rate);
    uint32_t count = 0;
    //while loop...
    while(ros::ok())
    {
        //get robot data
        DeltaRobot::v4DJoint v4djoint = robot.GetJointAngles();
        DeltaRobot::v4DJointRate v4djointrate = robot.GetJointAnglesRate();
        DeltaRobot::v4DCurrent v4dcurrent = robot.GetJointCurrent();
        DeltaRobot::v4DPoint v4dpoint = robot.GetPosition();
        //publish the joint data
        sensor_msgs::JointState jointstate;
        jointstate.header.frame_id = "deltarobot";
        jointstate.header.seq = count;
        jointstate.header.stamp = ros::Time::now();
        jointstate.name = {"joint1", "joint2", "joint3", "joint4"};
        jointstate.position = {v4djoint.Joint.th1, v4djoint.Joint.th2, v4djoint.Joint.th3, v4djoint.YawAngle};
        jointstate.velocity = {v4djointrate.JointRate.vth1, v4djointrate.JointRate.vth2, v4djointrate.JointRate.vth3, v4djointrate.YawAngleRate};
        jointstate.effort = {v4dcurrent.JointCurrent1, v4dcurrent.JointCurrent2, v4dcurrent.JointCurrent3, v4dcurrent.JointCurrent4};
        jointstate_pub.publish(jointstate);
        //publish the pose data
        geometry_msgs::Pose pose;
        pose.position.x = v4dpoint.Position.Px; pose.position.y = v4dpoint.Position.Py; pose.position.z = v4dpoint.Position.Pz;
        tf::Quaternion quat = tf::createQuaternionFromYaw(v4dpoint.YawAngle);
        pose.orientation.w = quat.getW(); pose.orientation.x = quat.getX(); pose.orientation.y = quat.getY(); pose.orientation.z = quat.getZ();
        pose_pub.publish(pose);

        //refresh count
        count++;
        //rate sleep
        loop_rate.sleep();
        //ros spin once
        ros::spinOnce();
    }

    //close robot
    robot.Close();

    //return
    return(0);
}