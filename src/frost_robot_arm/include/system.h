#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdint.h>


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frost_robot_arm/ArmCoreToSystem.h"
#include "frost_robot_arm/SystemToArmCore.h"

#include "globalDataStructures.h"

class System
{
private:
    void receiveDataCallback(const frost_robot_arm::ArmCoreToSystem::ConstPtr& msg);
    double rad2deg(double rad);
    double deg2rad(double degree);
    double calcVelocity(int16_t velocityPercentig);
    void init();

    globalData_typeDef_robotArm_MOTOR_ARM m_newReceivedData;
    globalData_typeDef_robotArm_MOTOR_ARM m_oldReceivedData;
    globalData_typeDef_robotArm_ARM_MOTOR m_transmitData;
    bool m_positionReached;
    bool m_trajectoryPointReached;
    moveit::planning_interface::MoveGroupInterface* move_group;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan m_myPlan;
    moveit_visual_tools::MoveItVisualTools *visual_tools;
    uint16_t m_trajectoryIterator;
    ros::NodeHandle* m_rosNode;
    ros::AsyncSpinner* m_spinner;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    System(int argc, char** argv);
    ~System();
    globalData_typeDef_robotArmVelocity setAxeVelocity(globalData_enumTypeDef_robotArmAxis axeIndex, int16_t velocityPercentig);
    void enableMovement();
    void disableMovement();
    void setEndeffectorClosed(bool state);
    void setArmCoreMode(globalData_enumTypeDef_robotArmCoreMode armCoreMode);
    void setTargetTrajectoryPoint();
    void setTargetVelocitiy(globalData_typeDef_robotArmVelocity targetVelocity);
    void calcNewTrajectory(globalData_enumTypeDef_robotArmTeachedPos teachedPos, bool collisionDetection);
    void calcNewTrajectory(globalData_typeDef_robotArm_posTransformation transformationVector, bool collisionDetection);
    globalData_typeDef_robotArmVelocity calcNewVelocity(globalData_typeDef_robotArm_posTransformation transformationVector);
    globalData_typeDef_robotArm_MOTOR_ARM getArmCoreData();
    globalData_typeDef_robotArm_posTransformation getCartesianPosition();
    void incrementTrajectoryIterator();
    bool isTrajectoryPointReached();
    bool furtherTrajectoriePoints();
    void sendDataToArmCore();
};