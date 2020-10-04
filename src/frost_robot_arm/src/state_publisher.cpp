#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <frost_robot_arm/ArmCoreToSystem.h>
#include <vector>

const double pi = 3.141592654;
sensor_msgs::JointState joint_state;

double rad2deg(double radian);
double deg2rad(int16_t degree);
void initJointState();
void encoderValuesReceiver_callBack(const frost_robot_arm::ArmCoreToSystem::ConstPtr & msg);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;

    initJointState();

    joint_sub = n.subscribe("arm_core_to_ik_solver", 1, encoderValuesReceiver_callBack);
    joint_pub = n.advertise<sensor_msgs::JointState>("move_group/encoder_values", 1);

    ros::Rate loop_rate(30);

    while (ros::ok()) 
    {
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "world";

        joint_pub.publish(joint_state);

        loop_rate.sleep();
    } 
}

double rad2deg(double radian)
{
    double deg = radian * 180.0/pi;

    return deg;
}

double deg2rad(int16_t degree)
{
    double rad = ((double) degree) * pi/180.0;

    return rad;
}

void initJointState()
{
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.velocity.resize(6);

    joint_state.name[0] ="joint0";
    joint_state.name[1] ="joint1";
    joint_state.name[2] ="joint2";
    joint_state.name[3] ="joint3";
    joint_state.name[4] ="joint4";
    joint_state.name[5] ="joint5";

    joint_state.position[0] = 0;
    joint_state.position[1] = 0;
    joint_state.position[2] = 0;
    joint_state.position[3] = 0;
    joint_state.position[4] = 0;
    joint_state.position[5] = 0;

    joint_state.velocity[0] = 0;
    joint_state.velocity[1] = 0;
    joint_state.velocity[2] = 0;
    joint_state.velocity[3] = 0;
    joint_state.velocity[4] = 0;
    joint_state.velocity[5] = 0;
}

void encoderValuesReceiver_callBack(const frost_robot_arm::ArmCoreToSystem::ConstPtr & msg)
{
    joint_state.position[0] = deg2rad(msg->actualPositions[0])/1000;
    joint_state.position[1] = deg2rad(msg->actualPositions[1])/1000;
    joint_state.position[2] = deg2rad(msg->actualPositions[2])/1000;
    joint_state.position[3] = deg2rad(msg->actualPositions[3])/1000;
    joint_state.position[4] = deg2rad(msg->actualPositions[4])/1000;
    joint_state.position[5] = deg2rad(msg->actualPositions[5])/1000;

    joint_state.velocity[0] = deg2rad(msg->actualVelocities[0])/1000;
    joint_state.velocity[1] = deg2rad(msg->actualVelocities[1])/1000;
    joint_state.velocity[2] = deg2rad(msg->actualVelocities[2])/1000;
    joint_state.velocity[3] = deg2rad(msg->actualVelocities[3])/1000;
    joint_state.velocity[4] = deg2rad(msg->actualVelocities[4])/1000;
    joint_state.velocity[5] = deg2rad(msg->actualVelocities[5])/1000;
}