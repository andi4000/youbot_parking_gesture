#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <cmath>
#include "ros/time.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gesturedetector");
	
	ros::NodeHandle node;
	
	ros::Rate rate(40);
	
	tf::TransformListener tfListener;
	
	bool hasBegun = false;
	ros::Time begin;
	ros::Duration duration;
	
	bool inTheZone = false;
	
	float timeHoldPose = 1; // seconds
	
	// reference initial values for gesture
	// hand pos calc has to be relative to a fixed point because robot+cam will move
	float ref_a = 0, ref_b = 0, ref_c = 0;
	
	while (node.ok())
	{
		tf::StampedTransform t_right_shoulder;
		tf::StampedTransform t_right_hand;
/**
		tf::StampedTransform t_head;
		tf::StampedTransform t_neck;
		tf::StampedTransform t_torso;

		tf::StampedTransform t_left_shoulder;
		tf::StampedTransform t_left_elbow;
		tf::StampedTransform t_left_hand;

		tf::StampedTransform t_left_hip;
		tf::StampedTransform t_left_knee;
		tf::StampedTransform t_left_foot;
		
		tf::StampedTransform t_right_shoulder;
		tf::StampedTransform t_right_elbow;
		tf::StampedTransform t_right_hand;
		
		tf::StampedTransform t_right_hip;
		tf::StampedTransform t_right_knee;
		tf::StampedTransform t_right_foot;
*/	
		
		float a = 0, b = 0, c = 0;
		
		try
		{
			tfListener.lookupTransform("/openni_depth_frame", "right_shoulder", ros::Time(0), t_right_shoulder);
			//tfListener.lookupTransform("/openni_depth_frame", "right_elbow", ros::Time(0), t_right_elbow);
			tfListener.lookupTransform("/openni_depth_frame", "right_hand", ros::Time(0), t_right_hand);
			
			//ROS_INFO("hand: (%.2f, %.2f, %.2f)", t_right_hand.getOrigin().x(), t_right_hand.getOrigin().y(), t_right_hand.getOrigin().z());
			//ROS_INFO("shou: (%.2f, %.2f, %.2f)", t_right_shoulder.getOrigin().x(), t_right_shoulder.getOrigin().y(), t_right_shoulder.getOrigin().z());
			
			a = t_right_shoulder.getOrigin().x() - t_right_hand.getOrigin().x();
			b = t_right_shoulder.getOrigin().y() - t_right_hand.getOrigin().y();
			c = t_right_shoulder.getOrigin().z() - t_right_hand.getOrigin().z();
			
			a = std::abs(a);
			b = std::abs(b);
			c = std::abs(c);
			
			//ROS_INFO("a b c (%.2f, %.2f, %.2f)", a, b, c);
			
			//TODO: if gesture active, turn off "wave to exit"
			//TODO: needs to be adjusted from person to person since they have different arm length (?)
			//NOTE: a b c are in meters
			if (a < 0.15 && b < 0.25 && c > 0.40)
			{
				if (!hasBegun){
					begin = ros::Time::now();
					ROS_WARN("begin: %f", begin.toNSec());
					hasBegun = true;
				}

				duration = ros::Time::now() - begin;
				
				float time_diff = timeHoldPose - duration.toSec();
				
				if (time_diff > 0.1)
				{
					ROS_INFO("hold position for %.2f more seconds to enter gesture mode", time_diff);
				}
				else if (std::abs(time_diff) < 0.1 && !inTheZone)
				{
					ref_a = a;
					ref_b = b;
					ref_c = c;
					ROS_WARN("hand is IN THE ZONE");
					inTheZone = true;
				} // end if time_diff
				
				if (inTheZone)
				{
					ROS_INFO("rel pos: (%.2f, %.2f, %.2f)", ref_a - a, ref_b - b, ref_c - c);
				} // end if inTheZone
			}
			else // from if a b c
			{
				//ROS_INFO("nothing");
				hasBegun = false;
				inTheZone = false;
				ref_a = 0;
				ref_b = 0;
				ref_c = 0;
			} // end if a b c
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		
	} // while node.ok
	
	return 0;
}
