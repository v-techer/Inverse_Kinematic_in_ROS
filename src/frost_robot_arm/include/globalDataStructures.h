/*
  *****************************************************************************************************************************************
  * @title  globalDataStructures_CM_MTR.c
  *
  * @author Carsten Mueller & Max Triller
  *
  * @date   Created on: 14.03.2019
  *
  * @brief  Typedef of all data structures for data exchange between ground station and rover !
  * 		Also Mutex and Semaphore definitions.
  * 		- MODULE_typeDef_SUBMODULE_producer_consumer:
  * 			IO 		= I/O Core
  * 			GS 		= ground station
  * 			ARM 	= arm Core
  * 			DRILL	= drill Core
  * 			BMS		= battery management system
  * 		- Use get() and set() functions for save access to data structures
  *
  * @edit	 Max Triller 		20.06.2019	MTR
  * 			- format code & text /
  * 			- edit get() & set() functions /
  * 			- edit Mutex and Semaphores
  * 			- add function globalDataStrucutres_initData();
  **********************************************************************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GLOBALDATASTRUCTURES_H_
#define GLOBALDATASTRUCTURES_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
//#include "FreeRTOS.h" /* removed MTR 18.06.2019*/

/* External variables----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define GLOBAL_DATA_STRUCTURES_MS_WAIT  20 //Time to wait for getting mutex !?!?! not tested

#define GLOBAL_DATA_STRUCTURES_INIT_ZERO 0

/*Define Data IDs to differentiate UDP messages under same IP address */ /*MTR 18.06.2019*/
/*
#define GLOBALDATA_ID_DRIVETRAIN 		 0
#define GLOBALDATA_ID_BMS				 1
#define GLOBALDATA_ID_WEIGHTCELL		 2
#define GLOBALDATA_ID_SECURITY			 3
#define GLOBALDATA_ID_SAVETY			 4
#define GLOBALDATA_ID_ENVIROMENTSENSOR	 5
#define GLOBALDATA_ID_DRILL				 6
#define GLOBALDATA_ID_ARM				 7
#define GLOBALDATA_ID_POWERTRAIN 		 8
#define GLOBALDATA_ID_SYSTEMSTATE		 9
*/
/*Define Data IDs to differentiate UDP messages under same IP address */ /*MTR 20.06.2019*/
typedef enum
{
	GLOBALDATA_ID_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_ID_BMS				=	2,
	GLOBALDATA_ID_WEIGHTCELL		=	3,
	GLOBALDATA_ID_SECURITY			=	4,
	GLOBALDATA_ID_SAFETY			=	5,
	GLOBALDATA_ID_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_ID_DRILL				=	7,
	GLOBALDATA_ID_ARM				=	8,
	GLOBALDATA_ID_POWERTRAIN		=	9,
	GLOBALDATA_ID_SYSTEMSTATE		=	10,
	GLOBALDATA_ID_IMU 				=  	11
}globalData_enumTypeDef_dataID;

typedef enum
{
	GLOBALDATA_FLAG_RX_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_FLAG_RX_BMS				=	2,
	GLOBALDATA_FLAG_RX_WEIGHTCELL		=	3,
	GLOBALDATA_FLAG_RX_SECURITY			=	4,
	GLOBALDATA_FLAG_RX_SAFETY			=	5,
	GLOBALDATA_FLAG_RX_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_FLAG_RX_DRILL			=	7,
	GLOBALDATA_FLAG_RX_ARM				=	8,
	GLOBALDATA_FLAG_RX_POWERTRAIN		=	9,
	GLOBALDATA_FLAG_RX_SYSTEMSTATE		=	10
}globalData_enumTypeDef_signalFlagRx;

typedef enum
{
	GLOBALDATA_FLAG_TX_ERROR 			=	0,
	//GLOBALDATA_ID_DRIVETRAIN		=	1, old but gold the drivetrain !
	GLOBALDATA_FLAG_TX_BMS				=	2,
	GLOBALDATA_FLAG_TX_WEIGHTCELL		=	3,
	GLOBALDATA_FLAG_TX_SECURITY			=	4,
	GLOBALDATA_FLAG_TX_SAFETY			=	5,
	GLOBALDATA_FLAG_TX_ENVIROMENTSENSOR	=	6,
	GLOBALDATA_FLAG_TX_DRILL				=	7,
	GLOBALDATA_FLAG_TX_ARM				=	8,
	GLOBALDATA_FLAG_TX_POWERTRAIN		=	9,
	GLOBALDATA_FLAG_TX_SYSTEMSTATE		=	10
}globalData_enumTypeDef_signalFlagTx;

/* Private enumerations-------------------------------------------------------*/
typedef enum /*Created MTR 18.06.2019*/
{
	GLOBAL_DATA_STRUCT_ERROR 	= 0,
	GLOBAL_DATA_STRUCT_DISABLE 	= 1,
	GLOBAL_DATA_STRUCT_ENABLE 	= 0xFF
}globalData_enumTypeDef_activationStates;

typedef enum /* Created MTR 18.06.2019*/
{
	GLOBAL_DATA_STRUCT_GET_FAIL 	= 0,
	GLOBAL_DATA_STRUCT_SET_OK 		= 1,
	GLOBAL_DATA_STRUCT_SET_FAIL 	= 2,
	GLOBAL_DATA_STRUCT_GET_OK 	    = 0xAA
}globalData_enumTypeDef_functionReturn;

typedef enum
{
	 SYSTEMSTATE_READ_ERROR = 0 /*CHANGE MTR 18.06.2019 */
	,SYSTEMSTATE_ON				/*mode while rover boot-up and idle state */
	,SYSTEMSTATE_CONNECTED		/*mode after receiving heart-beat from GS */
	,SYSTEMSTATE_REMOTE			/*receiving remote control data*/
	,SYSTEMSTATE_AUTO		   /*Autonomous mode -> loads the skynet AI  */
	,SYSTEMSTATE_ERROR	= 0xFF /*activates self-ignition */
}globalData_enumTypeDef_systemState;

/* ADDED DK 23.08.2019 */
typedef enum
{
	KINEMATICSOLVER_ERROR = 1
	,KINEMATICSOLVER_BEST_SOLUTION = 2
	,KINEMATICSOLVER_BEST_POSIBLE_SOLUTION = 3
}globalData_enumTypeDef_KinematicSolverStatusCode;

/* ADDED DK 03.09.2019 */
typedef enum
{
	GREPPERANGLE_VERTICAL = 1
	,GREPPERANGLE_HORIZONTAL = 2
}globalData_enumTypeDef_GrepperAngle;

/* Private typedef -----------------------------------------------------------*/
/* Note:
 * Programmers of processing cores can define their data structure here:
 * Check data direction ->  MODULE_typeDef_SUBMODULE_producer_consumer
 */
typedef struct
{
	uint8_t dataID;
	uint8_t fill1;
	uint8_t	fill2;
	uint8_t fill3;
	float  xEuler;
	float  yEuler;
	float  zEuler;
	float  xAcc;
	float  yAcc;
	float  zAcc;
}globalData_typeDef_boschIMU;

typedef struct
{
	uint8_t dataID;

	uint8_t actualState;				// enable / disable
	int16_t targetVelocityLeft;			// m/s
	int16_t actualVelocityLeft;			// m/s
	int16_t targetVelocityRight;		// m/s
	int16_t actualVelocityRight;		// m/s
}globalData_typeDef_powerTrain_IO_GS;

typedef struct
{
	uint8_t dataID;

	uint8_t maxVelocity;				// percent [0:100]
	int8_t 	xValue;						// -100 ... 100
	int8_t 	yValue;						// -100 ... 100
	uint8_t targetState;				// enable / disable
}globalData_typeDef_powerTrain_GS_IO;

/* ---------- ROBOT ARM ---------- */
typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMMODE_AXES = 0,
	ROBOTARMMODE_JOY,
	ROBOTARMMODE_POSITION,
	ROBOTARMMODE_TEACHED_POS,
	ROBOTARMMODE_MAX_LOCAL_MODES,
	ROBOTARMMODE_UNDEFINED = 0xFF
}globalData_enumTypeDef_robotArmMode;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMTP_HOME = 0,
	ROBOTARMTP_FRONT,
	ROBOTARMTP_LEFT,
	ROBOTARMTP_RIGHT,
	ROBOTARMTP_CONTAINERX,
	ROBOTARMTP_CONTAINERY,
	ROBOTARMTP_CONTAINERZ,
	ROBOTARMTP_MAX_LOCAL_TPS,
	ROBOTARMTP_UNDEFINED = 0xFF
}globalData_enumTypeDef_robotArmTeachedPos;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARMAXIS_1 = 0,
	ROBOTARMAXIS_2,
	ROBOTARMAXIS_3,
	ROBOTARMAXIS_4,
	ROBOTARMAXIS_5,
	ROBOTARMAXIS_6,
	ROBOTARMAXIS_MAX_LOCAL_AXIS,
	ROBOTARMAXIS_UNDEFINED = 0xFF
}globalData_enumTypeDef_robotArmAxis;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	ROBOTARM2BUTTONS_DISABLE	=0
   ,ROBOTARM2BUTTONS_ON_OPEN_LEFT
   ,ROBOTARM2BUTTONS_OFF_CLOSE_RIGHT
}globalData_enumTypeDef_robotArm2Buttons;

typedef enum
{
	 /*CHANGE SE 08.07.2019 */
	GRIPPERSTATE_OPEN	= 0,
   	GRIPPERSTATE_CLOSED = 1,
	GRIPPERSTATE_UNDEFINED = 0xFF
}globalData_enumTypeDef_robotArmgripperStatus;

typedef enum
{
	ROBOTARMSTATE_UNDEFINED
   ,ROBOTARMSTATE_INIT
   ,ROBOTARMSTATE_IDLE
   ,ROBOTARMSTATE_READYTORUN
   ,ROBOTARMSTATE_RUN
   ,ROBOTARMSTATE_STOP
   ,ROBOTARMSTATE_ERROR
}globalData_enumTypeDef_robotArmState;

typedef enum
{
	ROBOTARMERROR_NONE

   ,ROBOTARMERROR_STATE_UNDEFINED
   ,ROBOTARMERROR_STATE_INIT
   ,ROBOTARMERROR_STATE_IDLE
   ,ROBOTARMERROR_STATE_READYTORUN
   ,ROBOTARMERROR_STATE_RUN
   ,ROBOTARMERROR_STATE_STOP

   ,ROBOTARMERROR_INIT_MOTOR1
   ,ROBOTARMERROR_INIT_MOTOR2
   ,ROBOTARMERROR_INIT_MOTOR3
   ,ROBOTARMERROR_INIT_MOTOR4
   ,ROBOTARMERROR_INIT_MOTOR5
   ,ROBOTARMERROR_INIT_MOTORDEFAULT
   ,ROBOTARMERROR_INIT_MOTOREE

   ,ROBOTARMERROR_INIT_ENCODER1
   ,ROBOTARMERROR_INIT_ENCODER2
   ,ROBOTARMERROR_INIT_ENCODER3
   ,ROBOTARMERROR_INIT_ENCODER4
   ,ROBOTARMERROR_INIT_ENCODER5

   ,ROBOTARMERROR_INIT_MICROLINEARACTUATOR

   ,ROBOTARMERROR_PSM_QUICKSTOP
   ,ROBOTARMERROR_PSM_OPERATIONAL_ENABLED

   ,ROBOTARMERROR_VELOCITY_ZERO

   ,ROBOTARMERROR_GLOBAL_DATA_SET



}globalData_enumTypeDef_robotArmError;

typedef enum
{
	 ROBOTARMFLAG_TRUE = 0
	,ROBOTARMFLAG_FALSE
}globalData_enumTypeDef_robotArmFlag;

typedef struct
{
	uint16_t 		angleActual;
	uint16_t 		angleDesired;
}globalData_typeDef_robotArmAngle;

typedef struct
{
	/* basics */
	globalData_enumTypeDef_robotArmState 		state;			// State
	globalData_enumTypeDef_robotArmMode 		mode;

	/* tools */
	uint16_t									angleDesired[5];
	uint8_t 									microLinearActorPercent;
	/* encoder */
	globalData_typeDef_robotArmAngle			encoder[5];
	/* motor */
	int16_t										motorVelocity[6]; // 5 axis robot arm + EE
	/*  */
	uint16_t									ikValues[2][5];
	uint8_t 									flagBerechnungGelaufen[2];


	/* TODO SE add from rx ethernet structure missing values */
}globalData_typeDef_robotArm;

typedef struct /*CHANGE SE 08.07.2019 */
{
	/* new data set '23'-bytes 31/07/2019 */
	/* new 32 bit pack */
	uint8_t dataID;
	globalData_enumTypeDef_robotArmMode mode;				// see enum typedef
	globalData_enumTypeDef_robotArmTeachedPos TeachedPos;  // see enum typedef
	globalData_enumTypeDef_robotArmAxis ArmAxis;			// see enum typedef
	/* new 32 bit pack */
	int8_t 	yValue;
	globalData_enumTypeDef_robotArm2Buttons Axis5State;
	int8_t  targetJointVelocity5;		//
	globalData_enumTypeDef_robotArm2Buttons endEffectorState;
	/* new 32 bit pack */
	// -100 ... 100 Joystick
	int16_t  targetCoordinates_r;		// joystick - y; cylinder coordinates
	int16_t  targetCoordinates_h;		// joystick - ?
	/* new 32 bit pack */
	int16_t  targetCoordinates_phi;		// joystick - x; 11
	int16_t  targetJointVelocity;		// [RPM]
	/* new 32 bit pack */
	uint16_t  maxJointVelocity;		// [RPM]
	uint16_t velocityEndEffector;	//
	/* new 32 bit pack */
	globalData_enumTypeDef_robotArm2Buttons microLinearActorState;
	uint8_t microLinearActorPercent;
	uint8_t grepperOrientation;	// old dummy laserTrigger
	uint8_t laserTrigger;		// old testVar

}globalData_typeDef_robotArm_GS_ARM_copy;

typedef struct
{
	int16_t target_x;
	int16_t target_y;
	int16_t target_z;
	int16_t target_roll;
	int16_t target_pitch;
	int16_t target_yaw;
}
globalData_typeDef_robotArm_posTransformation;

typedef struct /*CHANGE SE 08.07.2019 */
{
	/* new data set '23'-bytes 31/07/2019 */
	/* new 32 bit pack */
	uint8_t dataID;
	globalData_enumTypeDef_robotArmMode mode;				// see enum typedef
	globalData_enumTypeDef_robotArmTeachedPos teachedPos;  // see enum typedef
	globalData_enumTypeDef_robotArmAxis activAxis;			// see enum typedef
	int16_t axisVelocityPercentig;
	globalData_typeDef_robotArm_posTransformation targetCoordinate;
	globalData_enumTypeDef_robotArmgripperStatus endEffectorState;
	uint8_t movementStarted;
	uint8_t collisionDetection;
	uint8_t dummy0;
	uint8_t dummy1;
	uint8_t dummy2;
}
globalData_typeDef_robotArm_GS_ARM;

typedef struct /*CHANGE SE 08.07.2019 */
{
	uint8_t dataID;
	uint8_t status;					// in welchem zustand befinde ich mich
	uint8_t mode;
	uint8_t gripperStatate;
	globalData_typeDef_robotArm_posTransformation targetCoordinate;

	int16_t actualJointAngle1;			// 0,00 ... 360,00� = 0 ... 36000
	int16_t actualJointAngle2;			// 0 ... 360�
	int16_t actualJointAngle3;			// 0 ... 360�
	int16_t actualJointAngle4;			// 0 ... 360�
	int16_t actualJointAngle5;			// 0 ... 360�
	int16_t actualJointAngle6;			// 0 ... 360�
}
globalData_typeDef_robotArm_ARM_GS;

typedef struct /*CHANGE SE 08.07.2019 */
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint16_t actualCoordinates_r;		//
	int16_t actualCoordinates_h;		//
	uint16_t actualCoordinates_phi;		//

	uint16_t actualJointAngle1;			// 0,00 ... 360,00� = 0 ... 36000
	uint16_t actualJointAngle2;			// 0 ... 360�
	uint16_t actualJointAngle3;			// 0 ... 360�
	uint16_t actualJointAngle4;			// 0 ... 360�
	uint16_t actualJointAngle5;			// 0 ... 360�

	uint8_t state;
	int8_t flagJoint1;	//-1 = Warning, -2 = Error, 0 = OK;
	int8_t flagJoint2;
	int8_t flagJoint3;
	int8_t flagJoint4;
	int8_t flagJoint5;

	uint32_t dummy;

}globalData_typeDef_robotArm_ARM_GS_copy;

typedef struct
{
    int32_t HomeOffset1;          // degree*100/s
    int32_t HomeOffset2;          // degree*100/s
    int32_t HomeOffset3;          // degree*100/s
    int32_t HomeOffset4;          // degree*100/s
    int32_t HomeOffset5;          // degree*100/s
    int32_t HomeOffset6;          // degree*100/s
}globalData_typeDef_robotArmHomeOffset;

typedef struct
{
    int32_t JointVelocity1;          // degree*100/s
    int32_t JointVelocity2;          // degree*100/s
    int32_t JointVelocity3;          // degree*100/s
    int32_t JointVelocity4;          // degree*100/s
    int32_t JointVelocity5;          // degree*100/s
    int32_t JointVelocity6;          // degree*100/
}globalData_typeDef_robotArmVelocity;

typedef struct
{
    int32_t JointAcceleration1;          // degree*100/s^2
    int32_t JointAcceleration2;          // degree*100/s^2
    int32_t JointAcceleration3;          // degree*100/s^2
    int32_t JointAcceleration4;          // degree*100/s^2
    int32_t JointAcceleration5;          // degree*100/s^2
    int32_t JointAcceleration6;          // degree*100/s^2
}globalData_typeDef_robotArmAcceleration;

typedef struct
{
    int32_t JointAngle1;          // 0 ... 36000°/100
    int32_t JointAngle2;          // 0 ... 36000°/100
    int32_t JointAngle3;          // 0 ... 36000°/100
    int32_t JointAngle4;          // 0 ... 36000°/100
    int32_t JointAngle5;          // 0 ... 36000°/100
    int32_t JointAngle6;          // 0 ... 36000°/100
}globalData_typeDef_robotArmPositioning;

typedef enum
{
	 /*CHANGE CM 14.07.2020 */
	 ROBOTARMCOREMODE_NONE =0
	,ROBOTARMCOREMODE_POSITION
	,ROBOTARMCOREMODE_VELOCITY
}globalData_enumTypeDef_robotArmCoreMode;

typedef struct
{
	uint8_t dataID;
	uint8_t  operationEnabled;					// enable / disable
	globalData_enumTypeDef_robotArmCoreMode actualArmCoreMode; // see enum typedef
    uint8_t Endeffector_IsOpen;                             // 1: open, 0: close
    globalData_typeDef_robotArmPositioning actualPositions;
    globalData_typeDef_robotArmVelocity actualVelocities;
    globalData_typeDef_robotArmHomeOffset HomeOffset;
    globalData_typeDef_robotArmPositioning targetPositions;
    globalData_typeDef_robotArmVelocity targetVelocities;
    globalData_typeDef_robotArmAcceleration targetAcceleration;
    uint8_t PositionReached[6];
	uint8_t dummy[2];
}globalData_typeDef_robotArm_MOTOR_ARM;    // CM --> DK

typedef struct
{
	uint8_t dataID;
	uint8_t operationEnable;
	globalData_enumTypeDef_robotArmCoreMode targetArmCoreMode; // see enum typedef
    uint8_t Endeffector_open;                               // 1: open, 0: close
    globalData_typeDef_robotArmPositioning targetPositions;
    globalData_typeDef_robotArmVelocity targetVelocities;
	globalData_typeDef_robotArmAcceleration targetAcceleration;
}globalData_typeDef_robotArm_ARM_MOTOR;

/*ADDED DK 29.08.2019*/
typedef struct
{
	int16_t r;
	int16_t h;
	uint16_t phi;
}globalData_typeDef_vector;

/* ---------- ROBOT ARM END ---------- */

typedef struct
{
	uint8_t dataID;

	uint16_t temperature;				// 0.1 �C -40 �C
	uint16_t pressure;					// hPa
	uint8_t  humidity;					// % RH
}globalData_typeDef_environmentSensor_IO_GS;

typedef struct
{
	uint8_t dataID;

	int16_t weight;						// grams

}globalData_typeDef_weightCell_IO_GS;

typedef struct
{
	uint8_t dataID;
	globalData_enumTypeDef_systemState systemState;
	uint8_t chipTemp;

}globalData_typeDef_systemState_IO_GS;

typedef struct
{
	uint8_t dataID;
	//uint8_t jetsonRestart;				// enable / disable
	//uint8_t powerElectronics;				// enable / disable
	uint8_t systemEnable;					// globals System State
	uint8_t systemPowerElectronics;
	uint8_t systemFuseByte;
	uint8_t systemStatusRGB;
	uint8_t dataValidation;
	uint8_t timeoutCounter; 				/*Added MTR 25.06.19*/


}globalData_typeDef_security_GS_IO;

typedef struct
{
	uint8_t dataID;
	uint8_t fill1;
	uint8_t	fill2;
	uint8_t	fill3;
	float voltageCell1;				// 0.01 V
	float voltageCell2;				// 0.01 V
	float voltageCell3;				// 0.01 V
	float voltageCell4;				// 0.01 V
	float voltageCell5;				// 0.01 V
	float voltageCell6;				// 0.01 V
	float  current;					// 0.1 A
	//uint8_t dataValidation;
}globalData_typeDef_BMS_BMS_GS;

typedef struct
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint8_t  automaticMode;				// enable / disable
	uint8_t  lowerDrillUnit;			// lower / raise
	uint8_t	 targetDrillVelocity;		// 0 ... 100 rpm
	uint8_t  targetFeedRate;			// mm/min	*Vorschub*
	uint16_t targetDrillDepth;			// 0 ... 400 mm

}globalData_typeDef_drill_GS_DRILL;

typedef struct
{
	uint8_t dataID;
	uint8_t  enable;					// enable / disable
	uint16_t actualDrillPosition;		// 0 ... 300 mm (from lower/raise)
	uint8_t	 actualDrillVelocity;		// 0 ... 100 rpm
	uint8_t	 actualFeedRate;			// mm/min  	*Vorschub*
	uint16_t actualDrillDepth;			// 0 ... 400 mm

}globalData_typeDef_drill_DRILL_GS;

typedef struct
{
	uint8_t dataID;
	globalData_typeDef_powerTrain_IO_GS 		powerTrain_IO_GS;
	globalData_typeDef_environmentSensor_IO_GS 	environmentSensor_IO_GS;
	globalData_typeDef_weightCell_IO_GS 		weightCell_IO_GS;
	globalData_typeDef_systemState_IO_GS 		systemState_IO_GS;
	globalData_typeDef_BMS_BMS_GS 				BMS_BMS_GS;
	globalData_typeDef_robotArm_ARM_GS 			robotArm_ARM_GS;
	globalData_typeDef_drill_DRILL_GS 			drill_DRILL_GS;

}globalData_typeDef_DataToGS;

typedef struct
{
	uint8_t dataID;
	globalData_typeDef_DataToGS					DataToGS;
	globalData_typeDef_powerTrain_GS_IO 		powerTrain_GS_IO;
	globalData_typeDef_security_GS_IO 			security_GS_IO;
	globalData_typeDef_robotArm_GS_ARM 			robotArm_GS_ARM;
	globalData_typeDef_drill_GS_DRILL 			drill_GS_DRILL;

}globalData_typeDef_allData;

#endif /* GLOBALDATASTRUCTURES_CM_MTR_FROST_H_ */
