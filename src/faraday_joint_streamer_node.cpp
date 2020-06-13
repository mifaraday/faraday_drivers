#include <ros/ros.h>
#include <sensor_msgs/JointState.h> // for publishing robot current position
#include <faraday_drivers/faraday_joint_streamer.h>
#include <faraday_kinematics/faraday_kinematics.h>

void publishCurrentState(const ros::TimerEvent& timer,
						 ros::Publisher& joint_pub,
						 faraday_drivers::FaradayTrajectoryStreamer& sim)
{
	sensor_msgs::JointState joint_state;
	joint_state.header.frame_id="base_link";
	joint_state.header.stamp=ros::Time::now();
	joint_state.name=sim.getJointNames();

	//compute current position
    // just temporary store current pose
    std::vector<double> current_pose;
	sim.computeTrajectoryPosition(timer.current_real,current_pose);
	sim.pollAction();

    // use inverse kinematics compute joint position
    double joints[7];
    faraday_kinematics::inverse_kinematics(current_pose.data(),joints);
	
    joint_state.position.assign(joints,joints+7);
	// 将pose信息加到joint_state消息的后面，与关节信息一起发出去
	for(int i=0;i<5;i++)
		joint_state.position.push_back(current_pose[i]);

	joint_pub.publish(joint_state);
    
}						

void setCurrentTrajectory(const trajectory_msgs::JointTrajectoryConstPtr& traj,
						  faraday_drivers::FaradayTrajectoryStreamer& sim)
{
	ROS_INFO("Setting new trajctory");
	sim.setTrajectory(*traj);
}						  

int main(int argc,char** argv)
{
    const static double default_position[]={0.0, 0.0, 310.0, 0.0, 0.0};

	ros::init(argc,argv,"faraday_simulator_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //using this style handlenode is able to get private name

	//nh loads joint names if possible
	std::vector<std::string> joint_names;
	if(!nh.getParam("controller_joint_names",joint_names))
	{
		//otherwise, it loads defaults
		joint_names.push_back("joint_1");
		joint_names.push_back("joint_2");
		joint_names.push_back("joint_3");
		joint_names.push_back("joint_4");
		joint_names.push_back("joint_5");
        joint_names.push_back("joint_6");
        joint_names.push_back("joint_7");
		joint_names.push_back("x");
		joint_names.push_back("y");
		joint_names.push_back("z");
		joint_names.push_back("r");
		joint_names.push_back("p");
	}

	//pnh loads configuration parameters
	std::vector<double> seed_position;
	if(!pnh.getParam("initial_position",seed_position))
		seed_position.assign(default_position,default_position+5);

	// ROS_INFO_STREAM("initial_position: "<<seed_position[0]);

	double publish_rate;
	pnh.param<double>("rate",publish_rate,30.0);

	// ROS_INFO_STREAM("publish_rate: "<<publish_rate);

	//instantiate simulation
	faraday_drivers::FaradayTrajectoryStreamer sim(seed_position,joint_names,nh);

	//create pub/subscribers and wire them up 
	ros::Publisher current_joint_pub=nh.advertise<sensor_msgs::JointState>("joint_states",1);
    
	ros::Subscriber command_state_sub=
		nh.subscribe<trajectory_msgs::JointTrajectory>("joint_path_command",
													  1,
													  boost::bind(setCurrentTrajectory,
													  			   _1,
													  			   boost::ref(sim)));
	ros::Timer state_publish_timer=
		nh.createTimer(ros::Duration(1.0/publish_rate),boost::bind(publishCurrentState,
													    _1,
													    boost::ref(current_joint_pub),
													    boost::ref(sim)));

	ROS_INFO("Simulator service spinning");

	ros::spin();

	return 0;
}

