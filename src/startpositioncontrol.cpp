#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/JointState.h"
#include "DeltaRobotControl.h"


//define a delta robot object
DeltaRobot::DeltaRobotControl robot;


/**
 * callback function
 */
void TargetPoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    DeltaRobot::v4DPoint v4dpoint;
    v4dpoint.Position.Px = msg->position.x; v4dpoint.Position.Py = msg->position.y; v4dpoint.Position.Pz = msg->position.z;
    v4dpoint.YawAngle = msg->orientation.w;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    v4dpoint.YawAngle = yaw;
    //control the robot
    robot.PositionControl(v4dpoint);
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
}


/**
 * main function
 */
int main(int argc, char **argv)
{
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

    //subscribe the target position
    ros::Subscriber setpos_sub = n.subscribe<geometry_msgs::Pose>("/DeltaRobot/SetPose", 1000, TargetPoseCallback);
    //publish the robot joint data
    ros::Publisher jointstate_pub = n.advertise<sensor_msgs::JointState>("/DeltaRobot/JointState", 1000);
    //publish the robot pose data
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("/DeltaRobot/Pose", 1000);

    ros::Rate loop_rate(50);
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