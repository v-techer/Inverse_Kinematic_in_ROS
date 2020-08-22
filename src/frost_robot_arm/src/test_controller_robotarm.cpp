#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <ros/ros.h>



class MoveitJoy
{
private:
    ros::NodeHandle node_handle;
    ros::Subscriber joy_sub_;
    void parsSRDF();
    ros::Publisher plan_group_pub;
public:
    MoveitJoy();
    ~MoveitJoy();
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
};

void MoveitJoy::parsSRDF()
{
    moveit::planning_interface::MoveGroupInterface ri("/robot_description");
    ROS_INFO("IM IN THE MAINFUNCTION");
}

MoveitJoy::MoveitJoy()
{
    ROS_INFO("IM IN THE MAINFUNCTION");
    parsSRDF();
    plan_group_pub = node_handle.advertise<std_msgs::String>("/rviz/moveit/select_planning_group", 5);
    joy_sub_ = node_handle.subscribe<sensor_msgs::Joy>("joy", 10, &MoveitJoy::joyCallback, this);
}

MoveitJoy::~MoveitJoy()
{

}

void MoveitJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ROS_INFO("THIS IS A TEST");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_controller_robotarm");
    MoveitJoy mJoy;
    ros::spin();
}


//   // BEGIN_TUTORIAL
//   //
//   // Setup
//   // ^^^^^
//   //
//   // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
//   // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
//   // are used interchangably.
//   static const std::string PLANNING_GROUP = "panda_arm";

//   // The :move_group_interface:`MoveGroupInterface` class can be easily
//   // setup using just the name of the planning group you would like to control and plan for.
//   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

//   // We will use the :planning_scene_interface:`PlanningSceneInterface`
//   // class to add and remove collision objects in our "virtual world" scene
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

//   // Raw pointers are frequently used to refer to the planning group for improved performance.
//   const robot_state::JointModelGroup* joint_model_group =
//       move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   //
//   // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
//   visual_tools.deleteAllMarkers();

//   // Remote control is an introspection tool that allows users to step through a high level script
//   // via buttons and keyboard shortcuts in RViz
//   visual_tools.loadRemoteControl();

//   // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

//   // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
//   visual_tools.trigger();

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // We can print the name of the reference frame for this robot.
//   ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());

//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

//   // We can get a list of all the groups in the robot:
//   ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
//   std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
//             std::ostream_iterator<std::string>(std::cout, ", "));


// // Start the demo
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Planning to a Pose goal
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // We can plan a motion for this group to a desired pose for the
//   // end-effector.
//   geometry_msgs::Pose target_pose1;
//   target_pose1.orientation.w = -1.0;
//   target_pose1.position.x = 0.28;
//   target_pose1.position.y = -0.2;
//   target_pose1.position.z = 0.5;
//   //move_group.setPoseTarget(target_pose1);
  


//   // Now, we call the planner to compute the plan and visualize it.
//   // Note that we are just planning, not asking move_group
//   // to actually move the robot.
//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   // Visualizing plans
//   // ^^^^^^^^^^^^^^^^^
//   // We can also visualize the plan as a line with markers in RViz.
//   ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
//   visual_tools.publishAxisLabeled(target_pose1, "pose1");
//   visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


//   ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
//   ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
// }


// void joyCB(self, msg)
// {
//   if (len(msg.axes) == 6 && len(msg.buttons) == 17):
//     status = PS3WiredStatus(msg)
//   else if (len(msg.axes) == 8 && len(msg.buttons) == 11):
//     status = XBoxStatus(msg)
//   else if (len(msg.axes) == 20 && len(msg.buttons) == 17):
//     status = PS3Status(msg)
//   else if (len(msg.axes) == 20 && len(msg.buttons) == 17):
//     status = PS4Status(msg)
//   else if (len(msg.axes) == 20 && len(msg.buttons) == 17):
//     status = 3DMouseStatus(msg):
//   else
//     raise Exception("Unknown joystick")
//   self.run(status)
//   self.history.add(status)
// }
// void test()
// {
// }

// class JoyStatus()
// {
//   protected:
//      bool center = false
//      bool select = false
//      bool start = false
//      bool L3 = false
//      bool R3 = false
//      bool square = false
//      bool up = false
//      bool down = false
//      bool left = false
//      bool right = false
//      bool triangle = false
//      bool cross = false
//      bool circle = false
//      bool L1 = false
//      bool R1 = false
//      bool L2 = false
//      bool R2 = false
//      float left_analog_x = 0.0
//      float left_analog_y = 0.0
//      float right_analog_x = 0.0
//      float right_analog_y = 0.0
// };

// class PS3WiredStatus(JoyStatus): public JoyStatus()
// {
//   public:
//     PS3WiredStatus();
//     ~PS3WiredStatus();

//   def __init__(self, msg):
//       JoyStatus.__init__(self)
//       # creating from sensor_msgs/Joy
//       if msg.buttons[16] == 1:
//           self.center = True
//       else:
//           self.center = False
//       if msg.buttons[0] == 1:
//           self.select = True
//       else:
//           self.select = False
//       if msg.buttons[3] == 1:
//           self.start = True
//       else:
//           self.start = False
//       if msg.buttons[1] == 1:
//           self.L3 = True
//       else:
//           self.L3 = False
//       if msg.buttons[2] == 1:
//           self.R3 = True
//       else:
//           self.R3 = False
//       if msg.buttons[15] == 1:
//           self.square = True
//       else:
//           self.square = False
//       if msg.buttons[4] == 1:
//           self.up = True
//       else:
//           self.up = False
//       if msg.buttons[6] == 1:
//           self.down = True
//       else:
//           self.down = False
//       if msg.buttons[7] == 1:
//           self.left = True
//       else:
//           self.left = False
//       if msg.buttons[5] == 1:
//           self.right = True
//       else:
//           self.right = False
//       if msg.buttons[12] == 1:
//           self.triangle = True
//       else:
//           self.triangle = False
//       if msg.buttons[14] == 1:
//           self.cross = True
//       else:
//           self.cross = False
//       if msg.buttons[13] == 1:
//           self.circle = True
//       else:
//           self.circle = False
//       if msg.buttons[10] == 1:
//           self.L1 = True
//       else:
//           self.L1 = False
//       if msg.buttons[11] == 1:
//           self.R1 = True
//       else:
//           self.R1 = False
//       if msg.buttons[8] == 1:
//           self.L2 = True
//       else:
//           self.L2 = False
//       if msg.buttons[9] == 1:
//           self.R2 = True
//       else:
//           self.R2 = False
//       self.left_analog_x = msg.axes[0]
//       self.left_analog_y = msg.axes[1]
//       self.right_analog_x = msg.axes[2]
//       self.right_analog_y = msg.axes[3]
//       self.orig_msg = msg
// }

// PS3WiredStatus::PS3WiredStatus()
// buttons(flase)
// {

// }
// /*******************************************************************/
// /* Setze die target position per name fest. Diese wurde mit dem moveit assissten
//   tool erstellt. We*/
// moveit::planning_interface::MoveGroupInterface().setNamedTarget("ready");
// move_group.setNamedTarget("ready");