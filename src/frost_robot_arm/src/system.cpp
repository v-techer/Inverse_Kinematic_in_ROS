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


static const std::string PLANNING_GROUP = "frost_arm";

#define MAX_VELOCITY 10.0000

/*  set the correct resolution of the encoder with constant
    this is used to convert double numbers with 2 digits after
    the comma seperator*/
const int encoder_resolution_factor = 100;
const int groundstation_resolution_factor = 1000;
const double pi = 3.141592654;

// Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script

namespace rvt = rviz_visual_tools;

System::System(int argc, char** argv)
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

    m_trajectoryIterator = 0;

    m_transmitData.targetPositions.JointAngle1 = 0;
    m_transmitData.targetPositions.JointAngle2 = 0;
    m_transmitData.targetPositions.JointAngle3 = 0;
    m_transmitData.targetPositions.JointAngle4 = 0;
    m_transmitData.targetPositions.JointAngle5 = 0;
    m_transmitData.targetPositions.JointAngle6 = 0;
    m_transmitData.targetVelocities.JointVelocity1 = 0;
    m_transmitData.targetVelocities.JointVelocity2 = 0;
    m_transmitData.targetVelocities.JointVelocity3 = 0;
    m_transmitData.targetVelocities.JointVelocity4 = 0;
    m_transmitData.targetVelocities.JointVelocity5 = 0;
    m_transmitData.targetVelocities.JointVelocity6 = 0;
    m_transmitData.targetAcceleration.JointAcceleration1 = 0;
    m_transmitData.targetAcceleration.JointAcceleration2 = 0;
    m_transmitData.targetAcceleration.JointAcceleration3 = 0;
    m_transmitData.targetAcceleration.JointAcceleration4 = 0;
    m_transmitData.targetAcceleration.JointAcceleration5 = 0;
    m_transmitData.targetAcceleration.JointAcceleration6 = 0;
    m_newReceivedData.RobotArmPositionReached = 0;
    m_oldReceivedData.RobotArmPositionReached = 1;
    m_newReceivedData.PositionReached[0] = 0;
    m_newReceivedData.PositionReached[1] = 0;
    m_newReceivedData.PositionReached[2] = 0;
    m_newReceivedData.PositionReached[3] = 0;
    m_newReceivedData.PositionReached[4] = 0;
    m_newReceivedData.PositionReached[5] = 0;
    m_oldReceivedData.PositionReached[0] = 1;
    m_oldReceivedData.PositionReached[1] = 1;
    m_oldReceivedData.PositionReached[2] = 1;
    m_oldReceivedData.PositionReached[3] = 1;
    m_oldReceivedData.PositionReached[4] = 1;
    m_oldReceivedData.PositionReached[5] = 1;
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
    }

    switch (axeIndex)
    {
    case 0:
        //out of robot description file urdf called frost_arm.xacro
        tempAxes[0] = rad2deg(calcVelocity(velocityPercentig, 1.048)) * encoder_resolution_factor;
        break;
    case 1:
        tempAxes[1] = rad2deg(calcVelocity(velocityPercentig, 0.348)) * encoder_resolution_factor;
        break;
    case 2:
        tempAxes[2] = rad2deg(calcVelocity(velocityPercentig, 0.522)) * encoder_resolution_factor;
        break;
    case 3:
        tempAxes[3] = rad2deg(calcVelocity(velocityPercentig, 0.942)) * encoder_resolution_factor;
        break;
    case 4:
        tempAxes[4] = rad2deg(calcVelocity(velocityPercentig, 1.256)) * encoder_resolution_factor;
        break;
    case 5:
        tempAxes[5] = rad2deg(calcVelocity(velocityPercentig, 1.000)) * encoder_resolution_factor;
        break;
    default:
        break;
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

        m_transmitData.targetPositions.JointAngle1 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(0));
        m_transmitData.targetPositions.JointAngle2 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(1));
        m_transmitData.targetPositions.JointAngle3 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(2));
        m_transmitData.targetPositions.JointAngle4 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(3));
        m_transmitData.targetPositions.JointAngle5 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(4));
        m_transmitData.targetPositions.JointAngle6 = encoder_resolution_factor*rad2deg(trajectory_point.positions.at(5));
        m_transmitData.targetVelocities.JointVelocity1 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(0));
        m_transmitData.targetVelocities.JointVelocity2 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(1));
        m_transmitData.targetVelocities.JointVelocity3 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(2));
        m_transmitData.targetVelocities.JointVelocity4 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(3));
        m_transmitData.targetVelocities.JointVelocity5 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(4));
        m_transmitData.targetVelocities.JointVelocity6 = encoder_resolution_factor*rad2deg(trajectory_point.velocities.at(5));
        m_transmitData.targetAcceleration.JointAcceleration1 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(0));
        m_transmitData.targetAcceleration.JointAcceleration2 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(1));
        m_transmitData.targetAcceleration.JointAcceleration3 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(2));
        m_transmitData.targetAcceleration.JointAcceleration4 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(3));
        m_transmitData.targetAcceleration.JointAcceleration5 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(4));
        m_transmitData.targetAcceleration.JointAcceleration6 = encoder_resolution_factor*rad2deg(trajectory_point.accelerations.at(5));
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

void System::calcNewVelocity(globalData_typeDef_robotArm_posTransformation transformationVector)
{
    bool success;
    geometry_msgs::Pose startTarget;
    geometry_msgs::Pose endTarget;
    tf2::Quaternion rotation;
    tf2::Quaternion orientation;
    moveit::planning_interface::MoveItErrorCode errorCode;

    // normailze the the values that get send for the orientation scale it down to 20 percont of input value
    const double scale = 20;

    // get the current positon. The position is neccessery. It seems it conntains important
    // inforamtion for further excuting of positions.
    startTarget = move_group->getCurrentPose().pose;

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

    // calculate out of the endTarget the trajectory and store it in m_myPlan.trajectory
    move_group->setPoseTarget(endTarget);

    errorCode = move_group->plan(m_myPlan);

    success = (errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    uint8_t size = m_myPlan.trajectory_.joint_trajectory.points.size();
    
    // if calcualtion was successfuly save the new trajectory in in m_myPlan 
    if (success)
    {
        ROS_INFO("position planning successfuly!");
   
        // get joint Module Group of move_group current state and update joint_model_group
        joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP); 
        visual_tools->deleteAllMarkers();
        visual_tools->publishAxisLabeled(move_group->getPoseTargets().back().pose , "goal");
        visual_tools->publishTrajectoryLine(m_myPlan.trajectory_, joint_model_group);
        visual_tools->trigger();

        // reset up trajectory iterator to 0.
        m_trajectoryIterator = 0;
        // reset position reached for risng edge detection
        m_trajectoryPointReached = true;
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

    // Roll Pitch and Yaw are rotation around the Roll = X-Axis, Pitch= Y-Axis and Yaw= Z-Axis.
    // be aware it is always a rotation seen from the world origin of coordinate
    orientation.setRPY(deg2rad(transformationVector.target_roll), deg2rad(transformationVector.target_pitch), deg2rad(transformationVector.target_yaw));

    //The magnitude of a quaternion should be one. If numerical errors cause a
    //quaternion magnitude other than one, ROS will print warnings. To avoid these warnings, normalize the quaternion
    orientation.normalize();

    target.pose.orientation.x = orientation.getX();
    target.pose.orientation.y = orientation.getY();
    target.pose.orientation.z = orientation.getZ();
    target.pose.orientation.w = orientation.getW();

    target.pose.position.x = ((double) transformationVector.target_x/groundstation_resolution_factor);
    target.pose.position.y = ((double) transformationVector.target_y/groundstation_resolution_factor);
    target.pose.position.z = ((double) transformationVector.target_z/groundstation_resolution_factor);

    move_group->setPoseTarget(target);

    errorCode = move_group->plan(m_myPlan);

    success = (errorCode == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if calcualtion was successfuly save the new trajectory in in m_myPlan 
    if (success)
    {
        ROS_INFO("position planning successfuly!");
   
        // get joint Module Group of move_group current state and update joint_model_group
        joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP); 
        visual_tools->deleteAllMarkers();
        visual_tools->publishAxisLabeled(move_group->getPoseTargets().back().pose , "goal");
        visual_tools->publishTrajectoryLine(m_myPlan.trajectory_, joint_model_group);
        visual_tools->trigger();

        // reset up trajectory iterator to 0.
        m_trajectoryIterator = 0;
        // reset position reached for risng edge detection
        m_trajectoryPointReached = true;
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

            // get joint Module Group of move_group current state and update joint_model_group
            joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP); 

            visual_tools->deleteAllMarkers();
            visual_tools->publishTrajectoryLine(m_myPlan.trajectory_, joint_model_group);
            visual_tools->trigger();

            // reset up trajectory iterator to 0.
            m_trajectoryIterator = 0;
            m_trajectoryPointReached = true;
        }
        // TODO add a return value with info about calculation. For example moveitErrorCode
    }
    else
    {
        ROS_INFO("position is out of range!");
    }
}


globalData_typeDef_robotArm_MOTOR_ARM System::getArmCoreData()
{
    return m_newReceivedData;
}

globalData_typeDef_robotArm_posTransformation System::getCartesianPosition()
{
    geometry_msgs::PoseStamped currentPosition = move_group->getCurrentPose();
    std::vector<double> currentOrientation = move_group->getCurrentRPY();
    globalData_typeDef_robotArm_posTransformation currentPose;
 
    currentPose.target_roll = rad2deg(currentOrientation.at(0));    //roll
    currentPose.target_pitch = rad2deg(currentOrientation.at(1));   //pitch
    currentPose.target_yaw = rad2deg(currentOrientation.at(2));     //yaw
    currentPose.target_x = currentPosition.pose.position.x * groundstation_resolution_factor;
    currentPose.target_y = currentPosition.pose.position.y * groundstation_resolution_factor;
    currentPose.target_z = currentPosition.pose.position.z * groundstation_resolution_factor;

    return currentPose;
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

double System::calcVelocity(int16_t velocityPercentig, double maxvelocity)
{
    double rc = 0.0;

    // 100 stands for 100 Percent
    rc = maxvelocity * (((double) velocityPercentig)/100.0);

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

    // if you want the fake trajectory get executed uncomment this 4 lines
    // if(m_transmitData.operationEnable)
    //     move_group->execute(m_myPlan);
    // else
    //     move_group->stop();
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
    m_newReceivedData.RobotArmPositionReached = msg->RobotArmPositionReached;
    m_newReceivedData.dummy[0] = msg->dummy[0];


    // check if positionReached got deprecated
    if (m_newReceivedData.RobotArmPositionReached)
    {
        if (m_newReceivedData.RobotArmPositionReached != m_oldReceivedData.RobotArmPositionReached)
            m_trajectoryPointReached = true;
    }
}

/************************** end transmitter receiver **************************
 */