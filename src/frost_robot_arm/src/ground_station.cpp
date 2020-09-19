
#include "ground_station.h"
#include "globalDataStructures.h"
#include <stdint.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frost_robot_arm/ViewTemplate.h"
#include "frost_robot_arm/GroundstationToRobotarm.h"

    Groundstation::Groundstation(int argc, char** argv):
    m_newPosition(false),
    m_newTeachedPosition(false),
    m_newJoyMovement(false)
    {
        m_currentMode = -1;

        ros::init(argc, argv, "ground_station");

        m_rosNode = new ros::NodeHandle();

        m_spinner = new ros::AsyncSpinner(1);
        
        sub = m_rosNode->subscribe("groundstation_to_ik_solver", 1000, &Groundstation::receiveDataCallback, this);

        m_spinner->start();

        ROS_INFO("I am alive!");
    }

    Groundstation::~Groundstation()
    {

    }

    void Groundstation::initGroundstation()
    {
        m_newData.teachedPos = ROBOTARMTP_UNDEFINED;
        m_newData.mode = ROBOTARMMODE_UNDEFINED;
        m_newData.movementStarted = 0;
        m_newData.activAxis = ROBOTARMAXIS_UNDEFINED;
    }

    bool Groundstation::movementIsEnabled()
    {
        return (bool) m_newData.movementStarted;
    }

    bool Groundstation::modeWasChanged()
    {
        if (m_newData.mode == m_currentMode)
            return false;

        m_currentMode = m_newData.mode;

        return true;
    }

    bool Groundstation::newJoyMovement()
    {
        bool rc = m_newJoyMovement;
        m_newJoyMovement = false;
        return rc; 
    }

    bool Groundstation::newPosition()
    {
        bool rc = m_newPosition;
        m_newPosition = false;
        return rc;
    }

    bool Groundstation::newTeachedPosition()
    {
        bool rc = m_newTeachedPosition;
        m_newTeachedPosition = false;
        return rc;
    }

    int Groundstation::getCurrentMode()
    {
        return m_currentMode;
    }

    globalData_enumTypeDef_robotArmAxis Groundstation::getAxe()
    {
        return m_currentAxe;
    }

    int8_t Groundstation::getVelocitiy()
    {
        return m_newData.axisVelocity;
    }

    globalData_enumTypeDef_robotArmTeachedPos Groundstation::getTeachedPosition()
    {
        return m_newData.teachedPos;
    }

    globalData_typeDef_robotArm_posTransformation Groundstation::getPosition()
    {
        return m_newData.targetCoordinate;
    }

    globalData_typeDef_robotArm_posTransformation Groundstation::getJoyMovement()
    {
        return m_newData.targetCoordinate;
    }

    bool Groundstation::isCollisionDetectionEnabled()
    {
        return m_newData.collisionDetection;
    }


    bool Groundstation::getEndEffectorState()
    {
        return (bool) m_newData.endEffectorState;
    }

    void Groundstation::receiveDataCallback(const frost_robot_arm::GroundstationToRobotarm::ConstPtr& msg)
    {

        /* store the old Data befor overwriting them. this helps to optimies
         * the later inverse kinematik calculation. Only if a change in the 
         * position was recoginzed 
         */
        m_oldData = m_newData;

        m_newData.dataID = msg->dataID;
        m_newData.mode = static_cast<globalData_enumTypeDef_robotArmMode>(msg->mode);
        m_newData.endEffectorState = static_cast<globalData_enumTypeDef_robotArmgripperStatus>(msg->gripper_status);
        m_newData.movementStarted = msg->movementStarted;
        m_newData.dummy0 = msg->dummy0;
        m_newData.dummy1 = msg->dummy1;
        m_newData.dummy2 = msg->dummy2;
        m_newData.teachedPos = static_cast<globalData_enumTypeDef_robotArmTeachedPos>(msg->teached_pos);
        m_newData.activAxis = static_cast<globalData_enumTypeDef_robotArmAxis>(msg->active_axis);
        m_newData.axisVelocity = msg->axis_velocity;
        m_newData.targetCoordinate.target_x = msg->target_x;
        m_newData.targetCoordinate.target_y = msg->target_y;
        m_newData.targetCoordinate.target_z = msg->target_z;
        m_newData.targetCoordinate.target_roll = msg->target_roll;
        m_newData.targetCoordinate.target_pitch = msg->target_pitch;
        m_newData.targetCoordinate.target_yaw = msg->target_yaw;
        m_newData.collisionDetection = msg->collisionDetection;

        /* check if changes hase occured during the Position Mode. If so calculate new position*/
        if (m_newData.mode == ROBOTARMMODE_POSITION)
        {
            if(m_newData.targetCoordinate.target_x != m_oldData.targetCoordinate.target_x ||
                m_newData.targetCoordinate.target_y != m_oldData.targetCoordinate.target_y ||
                m_newData.targetCoordinate.target_z != m_oldData.targetCoordinate.target_z ||
                m_newData.targetCoordinate.target_roll != m_oldData.targetCoordinate.target_roll ||
                m_newData.targetCoordinate.target_pitch != m_oldData.targetCoordinate.target_pitch ||
                m_newData.targetCoordinate.target_yaw != m_oldData.targetCoordinate.target_yaw)
                {
                    m_newPosition = true;
                }
        }

        /* check if changes hase occured during the Position Mode. If so calculate new position*/
        if (m_newData.mode == ROBOTARMMODE_TEACHED_POS)
            if (m_newData.teachedPos != m_oldData.teachedPos)
                m_newTeachedPosition = true;

        /* for Joy movement alle incomming positions are neccessery. if new data are awalyble inform the state machine*/
        m_newJoyMovement = true;

        ROS_INFO("dataID: [%d]", msg->dataID);
        ROS_INFO("mode: [%d]", msg->mode);
        ROS_INFO("teached_pos: [%d]", msg->teached_pos);
        ROS_INFO("active_axis: [%d]", msg->active_axis);
        ROS_INFO("axis_velocity: [%d]", msg->axis_velocity);
        ROS_INFO("target_x: [%d]", msg->target_x);
        ROS_INFO("target_y: [%d]", msg->target_y);
        ROS_INFO("target_z: [%d]", msg->target_z);
        ROS_INFO("target_roll: [%d]", msg->target_roll);
        ROS_INFO("target_pitch: [%d]", msg->target_pitch);
        ROS_INFO("target_yaw: [%d]", msg->target_yaw);
        ROS_INFO("gripper_status: [%d]", msg->gripper_status);
        ROS_INFO("movementStarted: [%d]", msg->movementStarted);
        ROS_INFO("collisionDetection: [%d]", msg->collisionDetection);
        ROS_INFO("dummy0: [%d]", msg->dummy0);
        ROS_INFO("dummy1: [%d]", msg->dummy1);
        ROS_INFO("dummy2: [%d]", msg->dummy2);
    }