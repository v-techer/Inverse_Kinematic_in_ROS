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

#include "frost_robot_arm/ArmCoreToSystem.h"
#include "frost_robot_arm/SystemToArmCore.h"
#include "globalDataStructures.h"
#include "system.h"

// data to the robot arm core are transmittet as degres with a persition of 3 number of digits after the decimal point.
// further more are the numbers represented in rotation per minits.

static const std::string PLANNING_GROUP = "frost_arm";

#define MAX_VELOCITY 10.0000
const double pi = 3.141592654;

// Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script

namespace rvt = rviz_visual_tools;

System::System(int argc, char** argv):
m_positionReached(false)
{
    // //init the ros node
    ros::init(argc, argv, "system");

    m_rosNode = new ros::NodeHandle();

    m_spinner = new ros::AsyncSpinner(1);
    
    // initate receiver callback function
    sub = m_rosNode->subscribe("arm_core_to_ik_solver", 1000, &System::receiveDataCallback, this);
    
    // initate ros transmitter function
    pub = m_rosNode->advertise<frost_robot_arm::SystemToArmCore>("ik_solver_to_arm_core", 1);

    // start ros node
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

    visual_tools = new moveit_visual_tools::MoveItVisualTools("world");
}

/***************************** getter functions *******************************
 */
bool System::isTrajectoryPointReached()
{
    bool rc = m_trajectoryPointReached;
    m_trajectoryPointReached = false;
    return rc;
}

/*************************** end getter functions *****************************
 */


/***************************** setter functions *******************************
 */
void System::disableMovement()
{
    m_transmitData.operationEnable = false;
}

void System::enableMovement()
{
    m_transmitData.operationEnable = true;
}

globalData_typeDef_robotArmVelocity System::setAxeVelocity(globalData_enumTypeDef_robotArmAxis axeIndex, int16_t velocityPercentig)
{
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    int32_t tempAxes[ROBOTARMAXIS_MAX_LOCAL_AXIS];
    globalData_typeDef_robotArmVelocity targetVelocity;

    for (uint8_t i = 0; i < ROBOTARMAXIS_MAX_LOCAL_AXIS; i++)
    {
        tempAxes[i] = 0;
        if ( i == axeIndex )
            tempAxes[i] = rad2deg(calcVelocity(velocityPercentig)) * 1000;
    }

    targetVelocity.JointVelocity1 = tempAxes[0];
    targetVelocity.JointVelocity2 = tempAxes[1];
    targetVelocity.JointVelocity3 = tempAxes[2];
    targetVelocity.JointVelocity4 = tempAxes[3];
    targetVelocity.JointVelocity5 = tempAxes[4];
    targetVelocity.JointVelocity6 = tempAxes[5];

    return targetVelocity;
}

void System::setArmCoreMode(globalData_enumTypeDef_robotArmCoreMode armCoreMode)
{
    m_transmitData.targetArmCoreMode = armCoreMode;
}

void System::setEndeffectorClosed(bool state)
{
    m_transmitData.Endeffector_open = (uint8_t) state;
}

void System::setTargetVelocitiy(globalData_typeDef_robotArmVelocity targetVelocity)
{
    m_transmitData.targetVelocities = targetVelocity;
}

void System::setTargetTrajectoryPoint()
{
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

    if (m_trajectoryIterator < m_myPlan.trajectory_.joint_trajectory.points.size())
    {
        trajectory_point = m_myPlan.trajectory_.joint_trajectory.points.at(m_trajectoryIterator);

        m_transmitData.targetPositions.JointAngle1 = 1000*rad2deg(trajectory_point.positions.at(0));
        m_transmitData.targetPositions.JointAngle2 = 1000*rad2deg(trajectory_point.positions.at(1));
        m_transmitData.targetPositions.JointAngle3 = 1000*rad2deg(trajectory_point.positions.at(2));
        m_transmitData.targetPositions.JointAngle4 = 1000*rad2deg(trajectory_point.positions.at(3));
        m_transmitData.targetPositions.JointAngle5 = 1000*rad2deg(trajectory_point.positions.at(4));
        m_transmitData.targetPositions.JointAngle6 = 1000*rad2deg(trajectory_point.positions.at(5));
        m_transmitData.targetVelocities.JointVelocity1 = 1000*rad2deg(trajectory_point.velocities.at(0));
        m_transmitData.targetVelocities.JointVelocity2 = 1000*rad2deg(trajectory_point.velocities.at(1));
        m_transmitData.targetVelocities.JointVelocity3 = 1000*rad2deg(trajectory_point.velocities.at(2));
        m_transmitData.targetVelocities.JointVelocity4 = 1000*rad2deg(trajectory_point.velocities.at(3));
        m_transmitData.targetVelocities.JointVelocity5 = 1000*rad2deg(trajectory_point.velocities.at(4));
        m_transmitData.targetVelocities.JointVelocity6 = 1000*rad2deg(trajectory_point.velocities.at(5));
        m_transmitData.targetAcceleration.JointAcceleration1 = 1000*rad2deg(trajectory_point.accelerations.at(0));
        m_transmitData.targetAcceleration.JointAcceleration2 = 1000*rad2deg(trajectory_point.accelerations.at(1));
        m_transmitData.targetAcceleration.JointAcceleration3 = 1000*rad2deg(trajectory_point.accelerations.at(2));
        m_transmitData.targetAcceleration.JointAcceleration4 = 1000*rad2deg(trajectory_point.accelerations.at(3));
        m_transmitData.targetAcceleration.JointAcceleration5 = 1000*rad2deg(trajectory_point.accelerations.at(4));
        m_transmitData.targetAcceleration.JointAcceleration6 = 1000*rad2deg(trajectory_point.accelerations.at(5));
    }
}

/*************************** end setter functions *****************************
 */

/***************************** public functions *******************************
 */
void System::incrementTrajectoryIterator()
{
    m_trajectoryIterator++;
}

bool System::furtherTrajectoriePoints()
{
    if (m_trajectoryIterator < m_myPlan.trajectory_.joint_trajectory.points.size())
        return true;
}

globalData_typeDef_robotArmVelocity System::calcNewVelocity(globalData_typeDef_robotArm_posTransformation transformationVector)
{
    geometry_msgs::Pose startTarget;
    geometry_msgs::Pose endTarget;
    std::vector<geometry_msgs::Pose> waypoints;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    tf2::Quaternion rotation;
    tf2::Quaternion orientation;
    tf2::Quaternion newOrientation;

    // normailze the the values that get send for the orientation scale it down to 20 percont of input value
    const double scale = 20;

    // get the current positon. The position is neccessery. It seems it conntains important
    // inforamtion for further excuting of positions.
    startTarget = move_group->getCurrentPose().pose;

    // save the first actual state as target
    waypoints.push_back(startTarget);

    // copy all header data and overrite the actual position and orientation
    endTarget = startTarget;

    // change the target orientation into a Quaternion Objetct
    orientation.setX(endTarget.orientation.x);
    orientation.setY(endTarget.orientation.y);
    orientation.setZ(endTarget.orientation.z);
    orientation.setW(endTarget.orientation.w);

    // take care and use the setEulerZYX. This calcualates the absoulte positin of the endeffector.
    // with the setRPY it always cals arelative movement!
    rotation.setRPY( deg2rad((double) transformationVector.target_roll*(scale/100.0)),
                    deg2rad((double) transformationVector.target_pitch*(scale/100.0)),
                    deg2rad((double) transformationVector.target_yaw*(scale/100.0)) );

    // add the actueal orientation with the relatively movement.
    orientation.operator*=(rotation);
    
    orientation.normalize();

    // change the target orientation with new calculated one
    endTarget.orientation.x = orientation.getX();
    endTarget.orientation.y = orientation.getY();
    endTarget.orientation.z = orientation.getZ();
    endTarget.orientation.w = orientation.getW();

    // add the new offeset to the catual position
    endTarget.position.x = endTarget.position.x + ((double) transformationVector.target_x/1000);
    endTarget.position.y = endTarget.position.y + ((double) transformationVector.target_y/1000);
    endTarget.position.z = endTarget.position.z + ((double) transformationVector.target_z/1000);

    waypoints.push_back(endTarget);

    // calculate out of the waypoints the trajectory and store it in m_myPlan.trajectory
    move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, m_myPlan.trajectory_);

    visual_tools->deleteAllMarkers();
    // visual_tools->publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    // for (std::size_t i = 0; i < waypoints.size(); ++i)
    //     visual_tools->publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools->trigger();
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
   
        joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);   
        visual_tools->deleteAllMarkers();
        visual_tools->publishAxisLabeled(move_group->getPoseTargets().back().pose , "goal");
        visual_tools->publishTrajectoryLine(m_myPlan.trajectory_, joint_model_group);
        visual_tools->trigger();
    }

    // TODO add a return value with info about calculation. For example moveitErrorCode
}    

void System::calcNewTrajectory(globalData_enumTypeDef_robotArmTeachedPos teachedPos, bool collisionDetection)
{
    bool success = false;
    std::string targetName;

    moveit::planning_interface::MoveItErrorCode errorCode;

    globalData_typeDef_robotArm_ARM_MOTOR targetValue;

    // get all registerd position for robot arm as list of names
    ros::V_string listOfTargetPositionNames = move_group->getNamedTargets();

    //check if position is in range.
    if ( teachedPos < listOfTargetPositionNames.size())
    {
        targetName = move_group->getNamedTargets()[teachedPos];

        move_group->setNamedTarget(targetName);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group
        // to actually move the robot.

        errorCode = move_group->plan(m_myPlan);

        success = (errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        // if calcualtion was successfuly save the new trajectory in in m_myPlan 
        if (success)
        {
            ROS_INFO("position planning successfuly!");

            joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP); 
            visual_tools->deleteAllMarkers();
            visual_tools->publishTrajectoryLine(m_myPlan.trajectory_, joint_model_group);
            visual_tools->trigger();
            // reset up trajectory iterator to 0.
            m_trajectoryIterator = 0;
        }
        // TODO add a return value with info about calculation. For example moveitErrorCode
    }
    else
    {
        ROS_INFO("position is out of range!");
    }
}

/*************************** end public functions *****************************
 */

/************************** local private functions ***************************
 */

double System::rad2deg(double radian)
{
    double deg = radian * 180.0/pi;

    return deg;
}

double System::deg2rad(double degree)
{
    double rad = ((double) degree) * pi/180.0;

    return rad;
}

double System::calcVelocity(int16_t velocityPercentig)
{
    double rc = 0.0;

    rc = MAX_VELOCITY * (((double) velocityPercentig)/100.0) * pi * 2.0;

    return rc;
}

/************************ end local private functions *************************
 */

/**************************** transmitter receiver ****************************
 */

void System::sendDataToArmCore()
{
    frost_robot_arm::SystemToArmCore msg;
    
    msg.dataID = GLOBALDATA_ID_ARM_CORE;
    msg.operationEnable = m_transmitData.operationEnable;
    msg.targetlArmCoreMode = m_transmitData.targetArmCoreMode;
    msg.Endeffector_open = m_transmitData.Endeffector_open;
    msg.targetPositions[0] = m_transmitData.targetPositions.JointAngle1;
    msg.targetPositions[1] = m_transmitData.targetPositions.JointAngle2;
    msg.targetPositions[2] = m_transmitData.targetPositions.JointAngle3;
    msg.targetPositions[3] = m_transmitData.targetPositions.JointAngle4;
    msg.targetPositions[4] = m_transmitData.targetPositions.JointAngle5;
    msg.targetPositions[5] = m_transmitData.targetPositions.JointAngle6;
    msg.targetVelocities[0] = m_transmitData.targetVelocities.JointVelocity1;
    msg.targetVelocities[1] = m_transmitData.targetVelocities.JointVelocity2;
    msg.targetVelocities[2] = m_transmitData.targetVelocities.JointVelocity3;
    msg.targetVelocities[3] = m_transmitData.targetVelocities.JointVelocity4;
    msg.targetVelocities[4] = m_transmitData.targetVelocities.JointVelocity5;
    msg.targetVelocities[5] = m_transmitData.targetVelocities.JointVelocity6;
    msg.targetAcceleration[0] = m_transmitData.targetAcceleration.JointAcceleration1;
    msg.targetAcceleration[1] = m_transmitData.targetAcceleration.JointAcceleration2;
    msg.targetAcceleration[2] = m_transmitData.targetAcceleration.JointAcceleration3;
    msg.targetAcceleration[3] = m_transmitData.targetAcceleration.JointAcceleration4;
    msg.targetAcceleration[4] = m_transmitData.targetAcceleration.JointAcceleration5;
    msg.targetAcceleration[5] = m_transmitData.targetAcceleration.JointAcceleration6;

    pub.publish(msg);

    if(m_transmitData.operationEnable)
        move_group->execute(m_myPlan);
    else
        move_group->stop();
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

    // check if positionReached got deprecated
    if (m_newReceivedData.PositionReached[0] && m_newReceivedData.PositionReached[1] && m_newReceivedData.PositionReached[2] &&
        m_newReceivedData.PositionReached[3] && m_newReceivedData.PositionReached[4] && m_newReceivedData.PositionReached[5])
    {
        if (!std::equal(std::begin(m_newReceivedData.PositionReached), std::end(m_newReceivedData.PositionReached), std::begin(m_oldReceivedData.PositionReached)))
            m_trajectoryPointReached = true;
    }
}

/************************** end transmitter receiver **************************
 */