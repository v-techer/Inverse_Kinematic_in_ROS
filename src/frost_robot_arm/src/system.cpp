#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>

#include "system.h"
#include "globalDataStructures.h"
#include <stdint.h>

#include "frost_robot_arm/ArmCoreToSystem.h"
#include "frost_robot_arm/SystemToArmCore.h"

#include <iostream>
#include <array>


static const std::string PLANNING_GROUP = "frost_arm";

#define MAX_VELOCITY 10
const double pi = 3.141592654;


System::System(int argc, char** argv):
m_positionReached(false)
{
    // //init the ros node
    ros::init(argc, argv, "system");

    m_rosNode = new ros::NodeHandle();

    m_spinner = new ros::AsyncSpinner(1);
    
    sub = m_rosNode->subscribe("arm_core_to_ik_solver", 1000, &System::receiveDataCallback, this);

    m_spinner->start();

    ROS_INFO("I am alive!");

    init();

}

System::~System()
{
    
}

void System::init()
{
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group = 
        move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
}

void System::moveAxe(globalData_enumTypeDef_robotArmAxis axe, int8_t velocity)
{

}

void System::stopMovement()
{
    // ros::Publisher joint_pub = m_rosNode->advertise<sensor_msgs::JointState>("joint_states", 1);
    // std_msgs::Header head;
    // sensor_msgs::JointState jointState;

    // jointState.header.stamp = ros::Time::now();
    // jointState.name.resize(8);
    // jointState.position.resize(8);
    // jointState.name[0] = "joint0";
    // jointState.name[1] = "joint1";
    // jointState.name[2] = "joint2";
    // jointState.name[3] = "joint3";
    // jointState.name[4] = "joint4";
    // jointState.name[5] = "joint5";
    // jointState.name[6] = "left_gripper_joint";
    // jointState.name[7] = "right_gripper_joint";

    // jointState.position[0] = 0;
    // jointState.position[1] = -0.8938;
    // jointState.position[2] = -1.7289;
    // jointState.position[3] = -1.9579;
    // jointState.position[4] = -1.3451;
    // jointState.position[5] = 0.9199;
    // jointState.position[6] = 0.0;
    // jointState.position[7] = 0.0;

    // joint_pub.publish(jointState);
}

void System::driveToTeachedPosition(globalData_enumTypeDef_robotArmTeachedPos value)
{
    bool success = false;
    globalData_typeDef_robotArm_ARM_MOTOR targetValue;

    // get all registerd position for robot arm as list of names
    ros::V_string listOfTargetPositionNames = move_group->getNamedTargets();
    int bla = listOfTargetPositionNames.size();
    //check if position is in range.
    if ( value < listOfTargetPositionNames.size())
    {
        move_group->setNamedTarget(move_group->getNamedTargets()[value]);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        success = (move_group->plan(m_myPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            for (int i = 0; i < m_myPlan.trajectory_.joint_trajectory.points.size(); i++)
            {

                //wait for position reached
                //while(!m_positionReached);

                // targetValue.dataID = GLOBALDATA_ID_ARM;
                // targetValue.Endeffector_open = false;
                // targetValue.operationEnable = true;

                // targetValue.targetArmCoreMode = ROBOTARMMODE_POSITION;
                // trajectory_msgs::JointTrajectoryPoint temTrajecotryPoints = my_plan.trajectory_.joint_trajectory.points.at(i);
                
                // for (int j = 0; j < temTrajecotryPoints.positions.size(); j++)
                // {
                //     // TODO convert rad to deg and save value in int32
                //     targetValue.targetPositions = dynamic_cast<int> (rad2deg(temTrajecotryPoints.positions.at(i)) * 10000);

                //     targetValue.targetVelocities = dynamic_cast<int> (rad2deg(temTrajecotryPoints.velocities.at(i)) * 10000);

                //     targetValue.targetAcceleration = dynamic_cast<int> (rad2deg(temTrajecotryPoints.accelerations.at(i)) * 10000);
                // }
                // sendDataToArmCore(targetValue);
            }
            move_group->execute(m_myPlan.trajectory_);
            move_group->move();
        }
    }
    else
    {
        ROS_INFO("position is out of range!");
    }
    
}

void System::driveToPosition(globalData_typeDef_robotArm_posTransformation value)
{
    
}

bool System::positionWasReached()
{
    
}

globalData_typeDef_robotArm_posTransformation System::calcNewPosition(globalData_typeDef_robotArm_posTransformation value)
{

}

void System::sendDataToArmCore(globalData_typeDef_robotArm_ARM_MOTOR TargetValues)
{

}

void System::setArmCoreMode(globalData_enumTypeDef_robotArmCoreMode armCoreMode)
{
    m_transmitData.targetArmCoreMode = armCoreMode;
}

void System::setEndeffectorClosed(bool state)
{
    m_transmitData.Endeffector_open = (uint8_t) state;
}

void System::setOperationEnable(bool operationState)
{
    m_transmitData.operationEnable = (uint8_t) operationState;
}

void System::sendTargetTrajectoryPoint()
{
    //******************fill this function with life******************
    bool nothing = true;
}

void System::setTargetVelocitiy(uint8_t targetVelocity)
{
    //*****************check if this numbers are legit*****************
    // MAX_VELOCITY 
    // m_transmitData.targetVelocities
}

void System::setTrajectoryPoint()
{
    // move_group->setStartState(move_group->getCurrentPose());
    move_group->execute(m_myPlan);

    int test = m_myPlan.trajectory_.joint_trajectory.points.size();

    if (!m_myPlan.trajectory_.joint_trajectory.points.empty())
    {
        //double test = m_myPlan.trajectory_.joint_trajectory.points.at(1).velocities.at(1);
        m_transmitData.targetPositions.JointAngle1 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(0));
        m_transmitData.targetPositions.JointAngle2 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(1));
        m_transmitData.targetPositions.JointAngle3 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(2));
        m_transmitData.targetPositions.JointAngle4 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(3));
        m_transmitData.targetPositions.JointAngle5 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(4));
        m_transmitData.targetPositions.JointAngle6 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).positions.at(5));
        m_transmitData.targetVelocities.JointVelocity1 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(0));
        m_transmitData.targetVelocities.JointVelocity2 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(1));
        m_transmitData.targetVelocities.JointVelocity3 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(2));
        m_transmitData.targetVelocities.JointVelocity4 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(3));
        m_transmitData.targetVelocities.JointVelocity5 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(4));
        m_transmitData.targetVelocities.JointVelocity6 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).velocities.at(5));
        m_transmitData.targetAcceleration.JointAcceleration1 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(0));
        m_transmitData.targetAcceleration.JointAcceleration2 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(1));
        m_transmitData.targetAcceleration.JointAcceleration3 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(2));
        m_transmitData.targetAcceleration.JointAcceleration4 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(3));
        m_transmitData.targetAcceleration.JointAcceleration5 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(4));
        m_transmitData.targetAcceleration.JointAcceleration6 = 10000*rad2deg(m_myPlan.trajectory_.joint_trajectory.points.at(0).accelerations.at(5));
    }
}

void System::calcNewTrajectory(globalData_enumTypeDef_robotArmTeachedPos teachedPos, bool collisionDetection)
{
    bool success = false;

    moveit::planning_interface::MoveItErrorCode errorCode;

    globalData_typeDef_robotArm_ARM_MOTOR targetValue;

    // get all registerd position for robot arm as list of names
    ros::V_string listOfTargetPositionNames = move_group->getNamedTargets();

    //check if position is in range.
    if ( teachedPos < listOfTargetPositionNames.size())
    {
        move_group->setNamedTarget(move_group->getNamedTargets()[teachedPos]);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.
        errorCode = move_group->plan(m_myPlan);

        success = (errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        // if calcualtion was successfuly save the new trajectory in in m_myPlan 
        if (success)
        {
            ROS_INFO("position planning successfuly!");
        }
        // TODO add a return value with info about calculation. For example moveitErrorCode
    }
    else
    {
        ROS_INFO("position is out of range!");
    }
}

void System::calcNewTrajectory(globalData_typeDef_robotArm_posTransformation transformationVector, bool collisionDetection)
{
    bool success;
    geometry_msgs::PoseStamped target;

    tf2::Quaternion rotation;
    tf2::Quaternion orientation;
    tf2::Quaternion newOrientation;
    moveit::planning_interface::MoveItErrorCode errorCode;

    success = false;

    // get the current positon. The position is neccessery. It seems it conntains important
    // inforamtion for further excuting of positions.
    target = move_group->getCurrentPose();

    // take care and use the setEulerZYX. This calcualates the absoulte positin of the endeffector.
    // with the setRPY it always cals arelative movement!
    orientation.setEuler( deg2rad(transformationVector.target_roll), deg2rad(transformationVector.target_pitch), deg2rad(transformationVector.target_yaw) );

    orientation.normalize();

    target.pose.orientation.x = orientation.getX();
    target.pose.orientation.y = orientation.getY();
    target.pose.orientation.z = orientation.getZ();
    target.pose.orientation.w = orientation.getW();

    target.pose.position.x = ((double) transformationVector.target_x/1000);
    target.pose.position.y = ((double) transformationVector.target_y/1000);
    target.pose.position.z = ((double) transformationVector.target_z/1000);

    move_group->setPoseTarget(target);

    errorCode = move_group->plan(m_myPlan);

    success = (errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    // if calcualtion was successfuly save the new trajectory in in m_myPlan 
    if (success)
    {
        ROS_INFO("position planning successfuly!");
    }
    // TODO add a return value with info about calculation. For example moveitErrorCode
}    


bool System::furtherTrajectoriePoints()
{
    return !(m_myPlan.trajectory_.joint_trajectory.points.empty());
}


bool System::isTrajectoryPointReached()
{
    bool rc = m_trajectoryPointReached;
    m_trajectoryPointReached = false;
    return rc;
}

double System::rad2deg(double radian)
{
    double deg = radian * 180/pi;
}

double System::deg2rad(int16_t degree)
{
    double rad = ((double) degree) * pi/180;
}

void System::receiveDataCallback(const frost_robot_arm::ArmCoreToSystem::ConstPtr& msg)
{
    m_oldReceivedData = m_newReceivedData;

    m_newReceivedData.dataID = msg->dataID;
    m_newReceivedData.operationEnabled = msg->operationEnabled;
    m_newReceivedData.actualArmCoreMode = static_cast<globalData_enumTypeDef_robotArmCoreMode>(msg->actualMode);
    m_newReceivedData.Endeffector_IsOpen = msg->Endeffector_IsOpen;
    m_newReceivedData.actualPositions.JointAngle1 =  msg->actualPositions[0];
    m_newReceivedData.actualPositions.JointAngle2 =  msg->actualPositions[1];
    m_newReceivedData.actualPositions.JointAngle3 =  msg->actualPositions[2];
    m_newReceivedData.actualPositions.JointAngle4 =  msg->actualPositions[3];
    m_newReceivedData.actualPositions.JointAngle5 =  msg->actualPositions[4];
    m_newReceivedData.actualPositions.JointAngle6 =  msg->actualPositions[5];
    m_newReceivedData.actualVelocities.JointVelocity1 = msg->actualVelocities[0];
    m_newReceivedData.actualVelocities.JointVelocity2 = msg->actualVelocities[1];
    m_newReceivedData.actualVelocities.JointVelocity3 = msg->actualVelocities[2];
    m_newReceivedData.actualVelocities.JointVelocity4 = msg->actualVelocities[3];
    m_newReceivedData.actualVelocities.JointVelocity5 = msg->actualVelocities[4];
    m_newReceivedData.actualVelocities.JointVelocity6 = msg->actualVelocities[5];
    m_newReceivedData.HomeOffset.HomeOffset1 = msg->HomeOffset[0];
    m_newReceivedData.HomeOffset.HomeOffset2 = msg->HomeOffset[1];
    m_newReceivedData.HomeOffset.HomeOffset3 = msg->HomeOffset[2];
    m_newReceivedData.HomeOffset.HomeOffset4 = msg->HomeOffset[3];
    m_newReceivedData.HomeOffset.HomeOffset5 = msg->HomeOffset[4];
    m_newReceivedData.HomeOffset.HomeOffset6 = msg->HomeOffset[5];
    m_newReceivedData.targetPositions.JointAngle1 = msg->targetPositions[0];
    m_newReceivedData.targetPositions.JointAngle2 = msg->targetPositions[1];
    m_newReceivedData.targetPositions.JointAngle3 = msg->targetPositions[2];
    m_newReceivedData.targetPositions.JointAngle4 = msg->targetPositions[3];
    m_newReceivedData.targetPositions.JointAngle5 = msg->targetPositions[4];
    m_newReceivedData.targetPositions.JointAngle6 = msg->targetPositions[5];
    m_newReceivedData.targetAcceleration.JointAcceleration1 = msg->targetAcceleration[0];
    m_newReceivedData.targetAcceleration.JointAcceleration2 = msg->targetAcceleration[1];
    m_newReceivedData.targetAcceleration.JointAcceleration3 = msg->targetAcceleration[2];
    m_newReceivedData.targetAcceleration.JointAcceleration4 = msg->targetAcceleration[3];
    m_newReceivedData.targetAcceleration.JointAcceleration5 = msg->targetAcceleration[4];
    m_newReceivedData.targetAcceleration.JointAcceleration6 = msg->targetAcceleration[5];
    m_newReceivedData.PositionReached[0] = msg->PositionReached[0];
    m_newReceivedData.PositionReached[1] = msg->PositionReached[1];
    m_newReceivedData.PositionReached[2] = msg->PositionReached[2];
    m_newReceivedData.PositionReached[3] = msg->PositionReached[3];
    m_newReceivedData.PositionReached[4] = msg->PositionReached[4];
    m_newReceivedData.PositionReached[5] = msg->PositionReached[5];
    m_newReceivedData.dummy[0] = msg->dummy[0];
    m_newReceivedData.dummy[1] = msg->dummy[1];


    if (m_newReceivedData.PositionReached[0] && m_newReceivedData.PositionReached[1] && m_newReceivedData.PositionReached[2] &&
        m_newReceivedData.PositionReached[3] && m_newReceivedData.PositionReached[4] && m_newReceivedData.PositionReached[5])
    {
        m_trajectoryPointReached = true;
    }
}

