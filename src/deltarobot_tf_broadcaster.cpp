#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"



void PoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z));
    tf::Quaternion q;
    q.setW(msg->orientation.w); q.setX(msg->orientation.x); q.setY(msg->orientation.y); q.setZ(msg->orientation.z);
    transform.setRotation(q);
    //broadcast tf
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "end_link"));
}



int main(int argc, char** argv)
{
    //ros init
    ros::init(argc, argv, "deltarobot_tf_broadcaster");
    //ros handle
    ros::NodeHandle n;

    //subscribe the delta robot pose
    ros::Subscriber pose_sub = n.subscribe<geometry_msgs::Pose>("/DeltaRobot/Pose", 1000, PoseCallback);
    ROS_INFO("start to broadcaster root tf...");

    //ros spin
    ros::spin();

    //return
    return(0);
}


