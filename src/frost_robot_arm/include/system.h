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
    void sendDataToArmCore(globalData_typeDef_robotArm_ARM_MOTOR TargetValues);
    void receiveDataCallback(const frost_robot_arm::ArmCoreToSystem::ConstPtr& msg);
    double rad2deg(double rad);
    double deg2rad(int16_t degree);

    globalData_typeDef_robotArm_MOTOR_ARM m_newReceivedData;
    globalData_typeDef_robotArm_MOTOR_ARM m_oldReceivedData;
    globalData_typeDef_robotArm_ARM_MOTOR m_transmitData;
    bool m_positionReached;
    bool m_trajectoryPointReached;
    moveit::planning_interface::MoveGroupInterface* move_group;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::planning_interface::MoveGroupInterface::Plan m_myPlan;
    uint16_t m_trajectoryIterator;
    ros::NodeHandle* m_rosNode;
    ros::AsyncSpinner* m_spinner;
    ros::Subscriber sub;

public:
    void moveAxe(globalData_enumTypeDef_robotArmAxis axe, int8_t velocity);
    void stopMovement();
    void init();
    void setEndeffectorClosed(bool state);
    void setOperationEnable(bool operationState);
    void setArmCoreMode(globalData_enumTypeDef_robotArmCoreMode armCoreMode);
    void calcNewTrajectory(globalData_enumTypeDef_robotArmTeachedPos teachedPos, bool collisionDetection);
    void calcNewTrajectory(globalData_typeDef_robotArm_posTransformation transformationVector, bool collisionDetection);
    void calcNewVelocity(globalData_typeDef_robotArm_posTransformation targetTransformation)
    void sendPositionToArmCore(void);
    void driveToTeachedPosition(globalData_enumTypeDef_robotArmTeachedPos value);
    void driveToPosition(globalData_typeDef_robotArm_posTransformation value);
    void incrementTrajectoryIterator();
    bool positionWasReached();
    bool isTrajectoryPointReached();
    bool furtherTrajectoriePoints();
    void setTargetTrajectoryPoint();
    void setTargetVelocitiy(uint8_t targetVelocity);
    globalData_typeDef_robotArm_posTransformation calcNewVelocity(globalData_typeDef_robotArm_posTransformation value);
    System(int argc, char** argv);
    ~System();
};