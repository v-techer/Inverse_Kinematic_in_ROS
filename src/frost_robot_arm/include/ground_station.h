
#include "globalDataStructures.h"
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frost_robot_arm/GroundstationToRobotarm.h"

class Groundstation
{
private:
    void init();
    void resetData();
    int m_currentMode;
    globalData_enumTypeDef_robotArmAxis m_currentAxe;
    globalData_typeDef_robotArm_GS_ARM m_newData;
    globalData_typeDef_robotArm_GS_ARM m_oldData;
    bool m_newPosition;
    bool m_newTeachedPosition;
    bool m_newJoyMovement;
    void receiveDataCallback(const frost_robot_arm::GroundstationToRobotarm::ConstPtr& msg);
    ros::NodeHandle* m_rosNode;
    ros::AsyncSpinner* m_spinner;
    ros::Subscriber sub;
    ros::Publisher pub;

public:
    bool movementIsEnabled();
    bool modeWasChanged();
    bool newJoyMovement();
    bool isCollisionDetectionEnabled();
    bool newPosition();
    bool newTeachedPosition();
    bool getEndEffectorState();
    int getCurrentMode();
    void sendDataToGroundstation(globalData_typeDef_robotArm_MOTOR_ARM armCoreData, globalData_typeDef_robotArm_posTransformation cartesianPosition);
    globalData_enumTypeDef_robotArmAxis getAxe();
    int16_t getVelocitiyPercentig();
    globalData_enumTypeDef_robotArmTeachedPos getTeachedPosition();
    globalData_typeDef_robotArm_posTransformation getPosition();
    globalData_typeDef_robotArm_posTransformation getJoyMovement();
    Groundstation(int argc, char** argv);
    ~Groundstation();
};