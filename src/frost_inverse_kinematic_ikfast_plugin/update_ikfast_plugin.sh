search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=frost.srdf
robot_name_in_srdf=frost
moveit_config_pkg=frost_moveit_config
robot_name=frost
planning_group_name=inverse_kinematic
ikfast_plugin_pkg=frost_inverse_kinematic_ikfast_plugin
base_link_name=link_base
eef_link_name=link_5
ikfast_output_path=/home/daniel/catkin_ws/src/frost_inverse_kinematic_ikfast_plugin/src/frost_inverse_kinematic_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path