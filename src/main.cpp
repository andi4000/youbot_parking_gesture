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

#define RIGHTHAND true
#define LEFTHAND false

//TODO: do not use global variable!
bool g_activeUserPresent = false;
bool g_activeUserVisible = false;
void userPresentCallback(const std_msgs::Bool::ConstPtr& msg)
{
	g_activeUserPresent = msg->data;
}
void userVisibleCallback(const std_msgs::Bool::ConstPtr& msg)
{
	g_activeUserVisible = msg->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gesturedetector");
	
	ros::NodeHandle node;
	
	tf::TransformListener tfListener;
	
	ros::Subscriber sub_activeUserPresent = node.subscribe("/openni2_user_selection/activeUserPresent", 1000, userPresentCallback);
	ros::Subscriber sub_activeUserVisible = node.subscribe("/openni2_user_selection/activeUserVisible", 1000, userVisibleCallback);
	
	ros::Publisher pub_offset_x = node.advertise<std_msgs::Float32>("/youbotStalker/gesture_processor/offset_linear_x", 1000);
	ros::Publisher pub_offset_y = node.advertise<std_msgs::Float32>("/youbotStalker/gesture_processor/offset_linear_y", 1000);
	ros::Publisher pub_gesture_state = node.advertise<std_msgs::Int32>("/youbotStalker/gesture_processor/state", 1000);
	ros::Rate rate(40);
	
	bool hasBegun = false;
	ros::Time begin;
	ros::Duration duration;
	
	bool inTheZone = false;
	
	float timeToHoldPose = 2; // seconds
	
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

	bool rh_inArea = false; // right hand
	bool lh_inArea = false; // left hand
	bool activeHand = RIGHTHAND; // or true
	bool gestureActive = false;
	tf::Vector3 gestureReference;

	while (node.ok())
	{	
		tf::StampedTransform tRightShoulder;
		tf::StampedTransform tRightHand;
		
		tf::StampedTransform tLeftShoulder;
		tf::StampedTransform tLeftHand;
		
		tf::Vector3 vRightShoulder, vRightHand, vLeftShoulder, vLeftHand;
				
		float a = 0, b = 0, c = 0;
		
		float handDistance;
		tf::Vector3 crossProductRH; // right hand
		tf::Vector3 crossProductLH; // left hand
		tf::Vector3 substractionRH;
		tf::Vector3 substractionLH;
		
		// to reset offset when active user is lost
		// actually this is not necessary, just to make sure
		if (!g_activeUserPresent)
		{
			msg_offset_x.data = 0.0;
			msg_offset_y.data = 0.0;
			msg_state.data = GESTURE_INACTIVE;
		}
		
		//TODO: why is this not working?????
		//if (!g_activeUserVisible)
			//continue;
			
		try
		{
			tfListener.lookupTransform("/openni_depth_frame", "right_shoulder", ros::Time(0), tRightShoulder);
			tfListener.lookupTransform("/openni_depth_frame", "right_hand", ros::Time(0), tRightHand);
			tfListener.lookupTransform("/openni_depth_frame", "left_shoulder", ros::Time(0), tLeftShoulder);
			tfListener.lookupTransform("/openni_depth_frame", "left_hand", ros::Time(0), tLeftHand);
			
			vRightShoulder = tRightShoulder.getOrigin();
			vRightHand = tRightHand.getOrigin();
			vLeftShoulder = tLeftShoulder.getOrigin();
			vLeftHand = tLeftHand.getOrigin();
			
			handDistance = tf::tfDistance(vRightHand, vLeftHand);

			crossProductRH = tf::tfCross(vRightShoulder, vRightHand);			
			crossProductLH = tf::tfCross(vLeftShoulder, vLeftHand);
			
			substractionRH = vRightShoulder - vRightHand;
			substractionLH = vLeftShoulder - vLeftHand;
			ROS_INFO("crossRH = (%.2f, %.2f, %.2f)", crossProductRH.x(), crossProductRH.y(), crossProductRH.z());
			ROS_INFO("crossLH = (%.2f, %.2f, %.2f)", crossProductLH.x(), crossProductLH.y(), crossProductLH.z());
			
			float crossLimit = 0.45;
			if (g_activeUserVisible && std::abs(crossProductRH.x()) < crossLimit && std::abs(crossProductRH.y()) < crossLimit)
				rh_inArea = true;
			else
				rh_inArea = false;

			if (g_activeUserVisible && std::abs(crossProductLH.x()) < crossLimit && std::abs(crossProductLH.y()) < crossLimit)
				lh_inArea = true;
			else
				lh_inArea = false;
							
			// activation area == gesture area
			if (rh_inArea || lh_inArea)
			{
				// timer begin
				float time_diff;
				if (!hasBegun && !gestureActive)
				{
					begin = ros::Time::now();
					ROS_WARN("begin: %f", begin.toNSec());
					hasBegun = true;
				}
				
				if (hasBegun && !gestureActive)
				{
					duration = ros::Time::now() - begin;
					time_diff = timeToHoldPose - duration.toSec();
				}
				
				if (hasBegun && !gestureActive && time_diff > 0.1)
				{
					ROS_INFO("hold position for %.2f s", time_diff);
				} 
				else if (hasBegun && !gestureActive && std::abs(time_diff) < 0.1)
				{
					ROS_WARN("Gesture Mode is ACTIVE");
					gestureActive = true;
					activeHand = rh_inArea; // if both hand are present, right hand will be chosen
					if (activeHand)
						gestureReference = substractionRH;
					else
						gestureReference = substractionLH;
				}
				// timer end
			}
			else // activation area
			{
				hasBegun = false;
				gestureActive = false;
				gestureReference = tf::Vector3(0,0,0);
				msg_state.data = GESTURE_INACTIVE;
				ROS_INFO("nothing");
			}
			
			// gesture sending
			if (gestureActive)
			{
				if (activeHand == RIGHTHAND)
				{
					ROS_INFO("RH x y offset = (%.2f, %.2f)", gestureReference.z() - substractionRH.z(), crossProductRH.y());
				}
				else if (activeHand == LEFTHAND)
				{
					ROS_INFO("LH x y offset = (%.2f, %.2f)", gestureReference.z() - substractionLH.z(), crossProductLH.y());
				}
				
				if (handDistance < 0.3)
				{	
					ROS_INFO("double hands mode!");
					msg_state.data = GESTURE_ACTIVE_TWO_HANDS;
				}
				else
					msg_state.data = GESTURE_ACTIVE_ONE_HAND;
			} // if gestureActive
			
			// =========================================================
			
			/**
			// this might become obsolete
			a = tRightShoulder.getOrigin().x() - tRightHand.getOrigin().x();
			b = tRightShoulder.getOrigin().y() - tRightHand.getOrigin().y();
			c = tRightShoulder.getOrigin().z() - tRightHand.getOrigin().z();
						
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
						time_diff = timeToHoldPose - duration.toSec();						
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
					ROS_INFO("hand dist = %.2f", handDistance);

					
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
			 
			*/
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
