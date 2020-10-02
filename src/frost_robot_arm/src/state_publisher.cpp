 #include <string>
    #include <ros/ros.h>
    #include <sensor_msgs/JointState.h>
    #include <tf/transform_broadcaster.h>
    #include <trajectory_msgs/JointTrajectory.h>
    #include <vector>
    int main(int argc, char** argv) {
        ros::init(argc, argv, "state_publisher");
        ros::NodeHandle n;
        ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("test_test/fake_controller_joint_states", 1);
        ros::Rate loop_rate(30);

        while (ros::ok()) 
        {
            sensor_msgs::JointState joint_state;
            joint_state.header.stamp = ros::Time::now();
            joint_state.header.frame_id = "world";
            joint_state.name.resize(6);
            joint_state.position.resize(6);

            joint_state.name[0] ="joint0";
            joint_state.name[1] ="joint1";
            joint_state.name[2] ="joint2";
            joint_state.name[3] ="joint3";
            joint_state.name[4] ="joint4";
            joint_state.name[5] ="joint5";

            joint_state.position[0] =0;
            joint_state.position[1] =0;
            joint_state.position[2] =0;
            joint_state.position[3] =0;
            joint_state.position[4] =0;
            joint_state.position[5] =0;

            joint_pub.publish(joint_state);

            loop_rate.sleep();
        } 
   }