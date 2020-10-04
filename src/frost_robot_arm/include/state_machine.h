
#include "globalDataStructures.h"
#include <stdint.h>

class StateMachine
{
private:
    enum LOCAL_STATES
    {  
        ST_STOPED = 0,
        ST_READY_TO_RUN,
        ST_RUNNING,
        MAX_LOCAL_STATES
    };

    int m_currentState;
    int m_currentMode;

    void enterStateStoped();
    void enterStateRunning();
    void enterStateReadyToRun();
    void init();

public:
    StateMachine();
    ~StateMachine();
    void triggerModeChange(int newMode);
    void triggerStartMovement();
    void triggerStopMovement();
    void triggerPositionReached();
    void triggerNextPosition();
    bool statusIsStopped();
    bool statusIsReadyToRun();
    bool statusIsRunning();
    bool currentModeAxeMode();
    bool currentModeJoyMode();
    bool currentModePosMode();
    bool currentModeTeachedPosMode();
    bool axisHasChanged();
};
