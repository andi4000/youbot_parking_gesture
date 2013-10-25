#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <cmath>
#include "ros/time.h"

//TODO: do not use global variable!
bool g_activeUserPresent = false;
void userStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
	g_activeUserPresent = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gesturedetector");
	
	ros::NodeHandle node;
	
	tf::TransformListener tfListener;
	
	ros::Subscriber sub_activeUserPresent = node.subscribe("/openni2_user_selection/activeUserPresent", 1000, userStateCallback);
	
	ros::Publisher pub_offset_x = node.advertise<std_msgs::Float32>("/youbotPID/linear_x/offset", 1000);
	ros::Publisher pub_offset_y = node.advertise<std_msgs::Float32>("/youbotPID/linear_y/offset", 1000);
	ros::Rate rate(40);
	
	
	bool hasBegun = false;
	ros::Time begin;
	ros::Duration duration;
	
	bool inTheZone = false;
	
	float timeHoldPose = 2; // seconds
	
	// reference initial values for gesture
	// hand pos calc has to be relative to a fixed point because robot+cam will move
	//TODO: needs to be tested
	float ref_a = 0, ref_b = 0, ref_c = 0;
	
	// ROS message
	std_msgs::Float32 msg_offset_x, msg_offset_y;
	// coordinate transformation: 
	// cam_x = robot_y
	// cam_z = robot_x
	//DONE: sign of above is it positive or negative
	//TODO: offset values should be held (not going back to 0 when no gesture) --> done --> NOPE!
	//DONE: when new user comes, offset should be back to 0!
	msg_offset_x.data = 0.0;
	msg_offset_y.data = 0.0;

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
		
		// to reset offset when active user is lost
		if (!g_activeUserPresent)
		{
			msg_offset_x.data = 0.0;
			msg_offset_y.data = 0.0;
		}
		
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
			
			/**
			a = std::abs(a);
			b = std::abs(b);
			c = std::abs(c);
			*/
			
			//ROS_INFO("a b c (%.2f, %.2f, %.2f)", a, b, c);
			
			//TODO: if gesture active, turn off "wave to exit"
			//TODO: needs to be adjusted from person to person since they have different arm length (?)
			//NOTE: a b c are in meters
			
			// Gesture area
			if (std::abs(a) < 0.25 && std::abs(b) < 0.35 && std::abs(c) > 0.30)
			{
				// Activation zone
				if (std::abs(a) < 0.15 && std::abs(b) < 0.25 && std::abs(c) > 0.40)
				{
					float time_diff;
					
					if (!hasBegun && !inTheZone){
						begin = ros::Time::now();
						ROS_WARN("begin: %f", begin.toNSec());
						hasBegun = true;
					}
					
					if (hasBegun && !inTheZone)
					{
						duration = ros::Time::now() - begin;
						time_diff = timeHoldPose - duration.toSec();						
					}
					
					if (hasBegun && !inTheZone && time_diff > 0.1)
					{
						ROS_INFO("hold position for %.2f s", time_diff);
					}
					else if (hasBegun && !inTheZone && std::abs(time_diff) < 0.1)
					{
						ref_a = a;
						ref_b = b;
						ref_c = c;
						ROS_WARN("GESTURE IS ACTIVE");
						inTheZone = true;
					} // end if time_diff
				}
				else // activation zone
				{
					hasBegun = false;
				}
				// end activation zone
				
				if (inTheZone)
				{
					ROS_INFO("rel pos: (%.2f, %.2f, %.2f)", ref_a - a, ref_b - b, ref_c - c);
					msg_offset_x.data = ref_c - c;
					msg_offset_y.data = ref_a - a;
					//TODO: plus or minus????
				}
				else if (!hasBegun)
				{
					msg_offset_x.data = 0;
					msg_offset_y.data = 0;
					ROS_INFO("inside gesture area but not active");
				}// end if inTheZone
			}
			else // if outside gesture area
			{
				//ROS_INFO("nothing");
				inTheZone = false;
				hasBegun = false;
				ref_a = 0;
				ref_b = 0;
				ref_c = 0;
				
				msg_offset_x.data = 0;
				msg_offset_y.data = 0;
			}
			// end if gesture area
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}
		
		// ROS message
		pub_offset_x.publish(msg_offset_x);
		pub_offset_y.publish(msg_offset_y);
		ros::spinOnce();
		rate.sleep();
	} // while node.ok
	
	return 0;
}
