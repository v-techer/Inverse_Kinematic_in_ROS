
#include "globalDataStructures.h"
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frost_robot_arm/GroundstationToRobotarm.h"

class Groundstation
{
private:
    globalData_enumTypeDef_robotArmMode m_currentMode;
    globalData_enumTypeDef_robotArmAxis m_currentAxe;
    globalData_typeDef_robotArm_GS_ARM m_newData;
    bool m_newDataFlag;
    void receiveDataCallback(const frost_robot_arm::GroundstationToRobotarm::ConstPtr& msg);
    ros::NodeHandle* m_rosNode;
    ros::AsyncSpinner* m_spinner;
    ros::Subscriber sub;


public:
    bool movementWasStarted();
    bool modeWasChanged();
    bool newJoyMovement();
    globalData_enumTypeDef_robotArmMode getCurrentMode();
    globalData_enumTypeDef_robotArmAxis getAxe();
    int8_t getVelocitiy();
    globalData_enumTypeDef_robotArmTeachedPos getTeachedPosition();
    globalData_typeDef_robotArm_posTransformation getPosition();
    globalData_typeDef_robotArm_posTransformation getJoyMovement();
    Groundstation(int argc, char** argv);
    ~Groundstation();
};