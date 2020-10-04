/*********************************************************************
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Daniel Kusmenko */

#include "ros/ros.h"

#include "globalDataStructures.h"
#include "ground_station.h"
#include "state_machine.h"
#include "system.h"


/* main app */

int main(int argc, char** argv)
{
  ROS_INFO("main application started!");

  StateMachine st;
  System sys(argc, argv);
  Groundstation gnd(argc, argv);

  while(1)
  {

    sys.setEndeffectorClosed(gnd.getEndEffectorState());

    /*check if the mode was changed to the modes:
    Joint Move, Cartesian Move, Cartesian Position or Teached Position*/
    if (gnd.modeWasChanged())
    {
      /*if mode was changed update the statemachin for further processing */
      st.triggerModeChange(gnd.getCurrentMode());
    }

    /* go in to current mode Axe Mode*/
    if (st.currentModeAxeMode())
    {
      // set the arm core mode to position mode
      sys.setArmCoreMode(ROBOTARMCOREMODE_VELOCITY);

      if (st.statusIsRunning())
      {

        if (gnd.movementIsEnabled())
        {
          // enabele operation
          sys.enableMovement();

          sys.setTargetVelocitiy(sys.setAxeVelocity(gnd.getAxe(), gnd.getVelocitiyPercentig()));
        }
        else
          st.triggerStopMovement();
      }

      if (st.statusIsStopped())
      {
        if (gnd.movementIsEnabled())
         st.triggerStartMovement();
        
        else
          sys.disableMovement();
      }
    }

    if (st.currentModeTeachedPosMode())
    {
      // set the arm core mode to position mode
      sys.setArmCoreMode(ROBOTARMCOREMODE_POSITION);

      if (st.statusIsReadyToRun() )
      {
        // is movement enabled
        if (gnd.movementIsEnabled() )
        {

          // check if position has changed
          if ( gnd.newTeachedPosition() )
          {
            // calucalte new position
            sys.calcNewTrajectory(gnd.getTeachedPosition(), gnd.isCollisionDetectionEnabled());
              
          }
          else
          {
            // if there is no new position check if list of trajectory has
            // some further entrence
            if ( sys.furtherTrajectoriePoints() )
            {
              // if end position not reached, change to state run and
              // send the current position until reached
              st.triggerNextPosition();
            }
            else
            {
              // if there is no new position and or the tracetrory list is empty
              // change to state stop movement.
              st.triggerStopMovement();
            }
          }
        }
        else
        {
          //if movement is disabled stopp any movement
          st.triggerStopMovement();
        }
      }

      if (st.statusIsRunning())
      {
        
        if (gnd.movementIsEnabled())
        {
          sys.enableMovement();

          // check if position has changed
          if ( gnd.newTeachedPosition() )
          {
            sys.calcNewTrajectory(gnd.getTeachedPosition(), gnd.isCollisionDetectionEnabled());
          }

          if (sys.isTrajectoryPointReached())
          {
            // pop the position out of list if position was reached
            sys.incrementTrajectoryIterator();

            // inform the state machine that the position was reached
            st.triggerPositionReached();
          }
          else
          {
            sys.setTargetTrajectoryPoint();
          }
        }
        else
        {
          st.triggerStopMovement();
        }
      }

      if (st.statusIsStopped())
      {
        if ( gnd.newTeachedPosition() )
        {
          sys.calcNewTrajectory(gnd.getTeachedPosition(), gnd.isCollisionDetectionEnabled());  
        }
        // if movent is enabled change change status to ready to run
        if (gnd.movementIsEnabled())
        {
          st.triggerStartMovement();      
        }
        else
          // if movement is disabled set Operation disable
          sys.disableMovement();
      }
    }

    if (st.currentModePosMode())
    {
      // set the arm core mode to position mode
      sys.setArmCoreMode(ROBOTARMCOREMODE_POSITION);

      if (st.statusIsReadyToRun())
      {
        // is movement enabled
        if (gnd.movementIsEnabled() )
        {
          // check if during the movemnt of the tractory the position has changed
          // in this case the ik solver needs to calc the new position and discard
          // the old trajectories
          if ( gnd.newPosition() )
          {
            // calucalte new position
            sys.calcNewTrajectory(gnd.getPosition(), gnd.isCollisionDetectionEnabled());
          }
          else
          {
            // check if all trajectory poits are reached
            if ( sys.furtherTrajectoriePoints() )
            {
              // if end position not reached, change to state run and
              // send the current position until reached
              st.triggerNextPosition();
            }
            else
            {
              // if there is no new position and or the tracetrory list is empty
              // change to state stop movement.
              st.triggerStopMovement();
            }
          }
        }
        else
        {
          //if movement is disabled stopp any movement
          st.triggerStopMovement();
        }
      }

      if (st.statusIsRunning())
      {
        if (gnd.movementIsEnabled())
        {

          sys.enableMovement();

          // check if position has changed
          if ( gnd.newPosition() )
          {
            sys.calcNewTrajectory(gnd.getPosition(), gnd.isCollisionDetectionEnabled());
          }

          if (sys.isTrajectoryPointReached())
          {
            // pop the position out of list if position was reached
            sys.incrementTrajectoryIterator();

            // inform the state machine that the position was reached
            st.triggerPositionReached();
          }
          else
          {
            sys.setTargetTrajectoryPoint();
          }
        }
        else
        {
          st.triggerStopMovement();
        }
        
      }

      if (st.statusIsStopped())
      {
        // if new position was select calc the new position for representation in Rziz
        if ( gnd.newPosition() )
        {
          sys.calcNewTrajectory(gnd.getPosition(), gnd.isCollisionDetectionEnabled()); 
        }
        // if movent is enabled change change status to ready to run
        if (gnd.movementIsEnabled())
        {
          st.triggerStartMovement();
        }
        // if movement is disabled stop any movement
        else
          sys.disableMovement();
      }
    }

    if (st.currentModeJoyMode())
    {
      // set the arm core mode to position mode
      sys.setArmCoreMode(ROBOTARMCOREMODE_VELOCITY);

      if (st.statusIsRunning())
      {
        if (gnd.movementIsEnabled())
        {
          sys.enableMovement();

          sys.setTargetVelocitiy(sys.calcNewVelocity(gnd.getJoyMovement()));

          if(gnd.newJoyMovement())
          {
            st.triggerNextPosition();
          }
        }
        else
        {
          st.triggerStopMovement();
        }
        
      }

      if (st.statusIsReadyToRun())
      {
        if (gnd.movementIsEnabled())
        {
          if (gnd.newJoyMovement())
          {
            st.triggerNextPosition();
          }
        }
        else
        {
          st.triggerStopMovement();
        }
      }

      if (st.statusIsStopped())
      {
        if (gnd.movementIsEnabled())
        {
          st.triggerStartMovement();
        }
        else
        {
          sys.disableMovement();
        }
      }
    }

    sys.sendDataToArmCore();

  }


  ros::shutdown();

  return 0;
}
