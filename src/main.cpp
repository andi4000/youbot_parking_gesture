/*
 * References:
 * Vector operation from API: http://docs.ros.org/fuerte/api/tf/html/c++/Vector3_8h.html
 * 
 */

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <cmath>
#include "ros/time.h"

#define GESTURE_INACTIVE 0
#define GESTURE_ACTIVE_ONE_HAND 1
#define GESTURE_ACTIVE_TWO_HANDS 2


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
	
	ros::Publisher pub_offset_x = node.advertise<std_msgs::Float32>("/youbotStalker/gesture_processor/offset_linear_x", 1000);
	ros::Publisher pub_offset_y = node.advertise<std_msgs::Float32>("/youbotStalker/gesture_processor/offset_linear_y", 1000);
	ros::Publisher pub_gesture_state = node.advertise<std_msgs::Int32>("/youbotStalker/gesture_processor/state", 1000);
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
	std_msgs::Int32 msg_state;
	// coordinate transformation: 
	// cam_x = robot_y
	// cam_z = robot_x
	//DONE: sign of above is it positive or negative
	//TODO: offset values should be held (not going back to 0 when no gesture) --> done --> NOPE!
	//DONE: when new user comes, offset should be back to 0!
	msg_offset_x.data = 0.0;
	msg_offset_y.data = 0.0;
	msg_state.data = GESTURE_INACTIVE;

	while (node.ok())
	{	
		tf::StampedTransform t_right_shoulder;
		tf::StampedTransform t_right_hand;
		
		tf::StampedTransform t_left_shoulder;
		tf::StampedTransform t_left_hand;
		
		tf::Vector3 v_right_shoulder, v_right_hand, v_left_shoulder, v_left_hand;
		
/**
		// complete list
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
		
		float hand_dist;
		tf::Vector3 crossProductRH; // right hand
		tf::Vector3 crossProductLH; // left hand
		
		// to reset offset when active user is lost
		// actually this is not necessary, just to make sure
		if (!g_activeUserPresent)
		{
			msg_offset_x.data = 0.0;
			msg_offset_y.data = 0.0;
			msg_state.data = GESTURE_INACTIVE;
		}
		
		try
		{
			tfListener.lookupTransform("/openni_depth_frame", "right_shoulder", ros::Time(0), t_right_shoulder);
			tfListener.lookupTransform("/openni_depth_frame", "right_hand", ros::Time(0), t_right_hand);
			tfListener.lookupTransform("/openni_depth_frame", "left_shoulder", ros::Time(0), t_left_shoulder);
			tfListener.lookupTransform("/openni_depth_frame", "left_hand", ros::Time(0), t_left_hand);
			
			v_right_shoulder = t_right_shoulder.getOrigin();
			v_right_hand = t_right_hand.getOrigin();
			v_left_shoulder = t_left_shoulder.getOrigin();
			v_left_hand = t_left_hand.getOrigin();
			
			hand_dist = tf::tfDistance(v_right_hand, v_left_hand);
			
			crossProduct_left = tf::tfCross(v_left_shoulder, v_left_hand);
			crossProduct_right = tf::tfCross(v_right_shoulder, v_right_hand);
			//ROS_INFO("cross result = (%.2f, %.2f, %.2f)", cross_hand_shoulder.x(), cross_hand_shoulder.y(), cross_hand_shoulder.z());
			
			
			
			// this might become obsolete
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
					ROS_INFO("hand dist = %.2f", hand_dist);

					
					msg_offset_x.data = ref_c - c;
					msg_offset_y.data = ref_a - a;
					msg_state.data = GESTURE_ACTIVE_ONE_HAND;
					//TODO: plus or minus????
				}
				else if (!hasBegun)
				{
					msg_offset_x.data = 0;
					msg_offset_y.data = 0;
					msg_state.data = GESTURE_INACTIVE;
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
				msg_state.data = GESTURE_INACTIVE;
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
		pub_gesture_state.publish(msg_state);
		ros::spinOnce();
		rate.sleep();
	} // while node.ok
	
	return 0;
}
