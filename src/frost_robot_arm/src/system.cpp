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

#include "system.h"
#include "globalDataStructures.h"
#include <stdint.h>

#include "frost_robot_arm/ArmCoreToSystem.h"
#include "frost_robot_arm/SystemToArmCore.h"

#include <iostream>
#include <array>


static const std::string PLANNING_GROUP = "frost_arm";


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
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            for (int i = 0; i < my_plan.trajectory_.joint_trajectory.points.size(); i++)
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
            move_group->execute(my_plan.trajectory_);
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

double System::rad2deg(double radian)
{
    double deg = radian * 180/pi;
}

void System::receiveDataCallback(const frost_robot_arm::ArmCoreToSystem::ConstPtr& msg)
{
    m_newData.dataID = msg->dataID;
    m_newData.operationEnabled = msg->operationEnabled;
    m_newData.actualArmCoreMode = static_cast<globalData_enumTypeDef_robotArmCoreMode>(msg->actualMode);
    m_newData.Endeffector_IsOpen = msg->Endeffector_IsOpen;
    m_newData.actualPositions.JointAngle1 =  msg->actualPositions[0];
    m_newData.actualPositions.JointAngle2 =  msg->actualPositions[1];
    m_newData.actualPositions.JointAngle3 =  msg->actualPositions[2];
    m_newData.actualPositions.JointAngle4 =  msg->actualPositions[3];
    m_newData.actualPositions.JointAngle5 =  msg->actualPositions[4];
    m_newData.actualPositions.JointAngle6 =  msg->actualPositions[5];
    m_newData.actualVelocities.JointVelocity1 = msg->actualVelocities[0];
    m_newData.actualVelocities.JointVelocity2 = msg->actualVelocities[1];
    m_newData.actualVelocities.JointVelocity3 = msg->actualVelocities[2];
    m_newData.actualVelocities.JointVelocity4 = msg->actualVelocities[3];
    m_newData.actualVelocities.JointVelocity5 = msg->actualVelocities[4];
    m_newData.actualVelocities.JointVelocity6 = msg->actualVelocities[5];
    m_newData.HomeOffset.HomeOffset1 = msg->HomeOffset[0];
    m_newData.HomeOffset.HomeOffset2 = msg->HomeOffset[1];
    m_newData.HomeOffset.HomeOffset3 = msg->HomeOffset[2];
    m_newData.HomeOffset.HomeOffset4 = msg->HomeOffset[3];
    m_newData.HomeOffset.HomeOffset5 = msg->HomeOffset[4];
    m_newData.HomeOffset.HomeOffset6 = msg->HomeOffset[5];
    m_newData.targetPositions.JointAngle1 = msg->targetPositions[0];
    m_newData.targetPositions.JointAngle2 = msg->targetPositions[1];
    m_newData.targetPositions.JointAngle3 = msg->targetPositions[2];
    m_newData.targetPositions.JointAngle4 = msg->targetPositions[3];
    m_newData.targetPositions.JointAngle5 = msg->targetPositions[4];
    m_newData.targetPositions.JointAngle6 = msg->targetPositions[5];
    m_newData.targetAcceleration.JointAcceleration1 = msg->targetAcceleration[0];
    m_newData.targetAcceleration.JointAcceleration2 = msg->targetAcceleration[1];
    m_newData.targetAcceleration.JointAcceleration3 = msg->targetAcceleration[2];
    m_newData.targetAcceleration.JointAcceleration4 = msg->targetAcceleration[3];
    m_newData.targetAcceleration.JointAcceleration5 = msg->targetAcceleration[4];
    m_newData.targetAcceleration.JointAcceleration6 = msg->targetAcceleration[5];
    m_newData.PositionReached[0] = msg->PositionReached[0];
    m_newData.PositionReached[1] = msg->PositionReached[1];
    m_newData.PositionReached[2] = msg->PositionReached[2];
    m_newData.PositionReached[3] = msg->PositionReached[3];
    m_newData.PositionReached[4] = msg->PositionReached[4];
    m_newData.PositionReached[5] = msg->PositionReached[5];
    m_newData.dummy[0] = msg->dummy[0];
    m_newData.dummy[1] = msg->dummy[1];
}

