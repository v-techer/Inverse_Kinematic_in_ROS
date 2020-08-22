#include "globalDataStructures.h"
#include <stdint.h>

class System
{
public:
    void moveAxe(globalData_enumTypeDef_robotArmAxis axe, int8_t velocity);
    void stopMovement();
    void driveToTeachedPosition(globalData_enumTypeDef_robotArmTeachedPos value);
    void driveToPosition(globalData_typeDef_robotArm_posTransformation value);
    bool positionWasReached();
    globalData_typeDef_robotArm_posTransformation calcNewPosition(globalData_typeDef_robotArm_posTransformation value);
    System();
    ~System();
};