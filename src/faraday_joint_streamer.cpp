#include "faraday_drivers/faraday_joint_streamer.h"
#include <ros/console.h>
#include <ros/assert.h>

namespace faraday_drivers
{
FaradayTrajectoryStreamer::FaradayTrajectoryStreamer(const std::vector<double>& seed_pose,
                              const std::vector<std::string>& joint_names,
                              ros::NodeHandle& nh)
	:joint_names_(joint_names)
	,traj_start_position_(seed_pose)						
	,traj_start_time_(ros::Time::now())
    ,action_server_(nh,"joint_trajectory_action",boost::bind(&FaradayTrajectoryStreamer::goalCB,this,_1),
					boost::bind(&FaradayTrajectoryStreamer::cancelCB,this,_1),false)
	,has_active_goal_(false)
{
	ROS_ASSERT(5==seed_pose.size());
	ROS_ASSERT(12==joint_names.size());
	action_server_.start();
}

bool FaradayTrajectoryStreamer::setTrajectory(const trajectory_msgs::JointTrajectory& new_trajectory)
{
	ROS_INFO("Setting new active trajectory");
	//Compute current state
	ros::Time now=ros::Time::now();
	std::vector<double> position;
	computeTrajectoryPosition(now,position);

	//Rollover to the new trajectory
	traj_start_position_=position;
	traj_=new_trajectory;
	traj_start_time_=now;

    /* 规划的路径包含的路点个数 */
    int point_num = traj_.points.size();
    // trajectory catesian position
    double x_position[point_num];
    double y_position[point_num];
    double z_position[point_num];
    double r_position[point_num];
    double p_position[point_num];

    /* 时间数组 */
    double time_from_start[point_num];

    for(int i=0;i<point_num;i++)
    {
        x_position[i]=traj_.points[i].positions[0];
        y_position[i]=traj_.points[i].positions[1];
        z_position[i]=traj_.points[i].positions[2];
        r_position[i]=traj_.points[i].positions[3];
        p_position[i]=traj_.points[i].positions[4];

        time_from_start[i]=traj_.points[i].time_from_start.toSec();
    }

    spline_x_.loadData(time_from_start, x_position, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    spline_y_.loadData(time_from_start, y_position, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    spline_z_.loadData(time_from_start, z_position, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    spline_r_.loadData(time_from_start, r_position, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);
    spline_p_.loadData(time_from_start, p_position, point_num, 0, 0, cubicSpline::BoundType_First_Derivative);

	return true;
}

static double linearInterpolate(double start,double stop,double ratio)
{
	return start+(stop-start)*ratio;
}

//Compute the robot position at a given time based on the currently active
//trajectory
bool FaradayTrajectoryStreamer::computeTrajectoryPosition(const ros::Time& tm,
								           std::vector<double>& output) const
{
	//Check to see if time is in past of traj
	if(tm<traj_start_time_||traj_.points.empty())
	{
		output=traj_start_position_;
		return true;
	}
	//check to see if time is past end of traj
	else if(tm>traj_start_time_+traj_.points.back().time_from_start)
	{
		output=traj_.points.back().positions;
		return true;
	}

	//Otherwise the traj must be within the trajectory
	ros::Duration dt=tm-traj_start_time_;

    size_t idx=0;
    for(size_t i=0;i<traj_.points.size();++i)
    {
        if(dt<traj_.points[i].time_from_start)
        {
            idx=i;
            break;
        }
    }

    //Grab the two points and interpolate
    const trajectory_msgs::JointTrajectoryPoint& end_pt=traj_.points[idx];

    //output container
    std::vector<double> point;
    double cartesian_position[5];
    point.reserve(traj_start_position_.size());

    if(0==idx)
    {
        //interpolate from start position
        double ratio=dt.toSec()/end_pt.time_from_start.toSec();

        for (int i = 0; i < 5; ++i)
        {
            point.push_back(linearInterpolate(traj_start_position_[i],end_pt.positions[i],ratio));
        }
    }
    else
    {
        double time=dt.toSec();

        spline_x_.getYbyX(time, cartesian_position[0]);
        spline_y_.getYbyX(time, cartesian_position[1]);
        spline_z_.getYbyX(time, cartesian_position[2]);
        spline_r_.getYbyX(time, cartesian_position[3]);
        spline_p_.getYbyX(time, cartesian_position[4]);

        for(int i=0;i<5;i++)
            point.push_back(cartesian_position[i]);
    }

	output=point;

	return true;
}	

void FaradayTrajectoryStreamer::pollAction()												
{
	if(has_active_goal_&&ros::Time::now()>(traj_start_time_+traj_.points.back().time_from_start))
	{
		active_goal_.setSucceeded();
		has_active_goal_=false;
	}
}

void FaradayTrajectoryStreamer::goalCB(JointTrajectoryActionServer::GoalHandle& gh)
{
	ROS_INFO("Receive new goal request");
	if(has_active_goal_)
	{
		ROS_WARN("Receive new goal,canceling current one");
		active_goal_.setAborted();
		has_active_goal_=false;
	}

	gh.setAccepted();
	active_goal_=gh;
	has_active_goal_=true;

	const trajectory_msgs::JointTrajectory& traj=active_goal_.getGoal()->trajectory;
	setTrajectory(traj);
}


void FaradayTrajectoryStreamer::cancelCB(JointTrajectoryActionServer::GoalHandle &gh)
{
	ROS_INFO("cancelling goal");
	if(active_goal_==gh)
	{
		//stop the controller
		//mark the goal as canceled
		active_goal_.setCanceled();
		has_active_goal_=false;
	}
}			

} //end ns faraday_drivers

