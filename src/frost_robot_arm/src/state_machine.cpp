
#include "main.h"
#include "state_machine.h"
#include "globalDataStructures.h"
#include <stdint.h>


  /* The state machine hase 3 types of states and 4 type of modes.
   * The state machine contains the state Stoped, Run and ReadyToRun.
   * in the first mode: Axes mode does contain the states Stoped and
   * Run. A Transition from Stoped to Run can be acomplished by
   * activating movement enabled. A transtion back will be acomplished
   * if the user disables the movement enable. 
   * 
  */


    StateMachine::StateMachine():
    m_currentMode(-1),
    m_currentState(-1)
    {
        init();
    }
    StateMachine::~StateMachine()
    {

    }

    void StateMachine::init()
    {
        enterStateStoped();
    }

    void StateMachine::enterStateRunning()
    {
        m_currentState = ST_RUNNING;
    }

    void StateMachine::enterStateReadyToRun()
    {
        m_currentState = ST_READY_TO_RUN;
    }

    void StateMachine::enterStateStoped()
    {
        m_currentState = ST_STOPED;
    }

    void StateMachine::triggerModeChange(int newMode)
    {
        m_currentMode = newMode;
    }
    
    void StateMachine::triggerStartMovement()
    {   
        if ((m_currentState == ST_STOPED))
        {
            if (m_currentMode == ROBOTARMMODE_JOY ||
                m_currentMode == ROBOTARMMODE_TEACHED_POS ||
                m_currentMode == ROBOTARMMODE_POSITION)
            {
                enterStateReadyToRun();
            }
            else
            {
                enterStateRunning();
            }
        }
    }

    void StateMachine::triggerStopMovement()
    {
        enterStateStoped();
    }


    void StateMachine::triggerPositionReached()
    {
        if (m_currentState == ST_RUNNING)
        {
            if (m_currentMode == ROBOTARMMODE_JOY ||
            m_currentMode == ROBOTARMMODE_TEACHED_POS ||
            m_currentMode == ROBOTARMMODE_POSITION)
                enterStateReadyToRun();
            else
                enterStateStoped();
        }
    }

    void StateMachine::triggerNextPosition()
    {
        enterStateRunning();
    }


    bool StateMachine::statusIsStopped()
    {
        return (m_currentState == ST_STOPED);
    }

    bool StateMachine::statusIsReadyToRun()
    {
        return (m_currentState == ST_READY_TO_RUN);
    }

    bool StateMachine::statusIsRunning()
    {
        return (m_currentState == ST_RUNNING);
    }

    bool StateMachine::currentModeAxeMode()
    {
        return (m_currentMode == ROBOTARMMODE_AXES);
    }

    bool StateMachine::currentModeJoyMode()
    {
        return (m_currentMode == ROBOTARMMODE_JOY);
    }

    bool StateMachine::currentModePosMode()
    {
        return (m_currentMode == ROBOTARMMODE_POSITION);
    }

    bool StateMachine::currentModeTeachedPosMode()
    {
        return (m_currentMode == ROBOTARMMODE_TEACHED_POS);
    }