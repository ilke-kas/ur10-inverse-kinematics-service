#include "ros/ros.h"
#include <cstdlib>
#include "ik_service/PoseIK.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_ik_client");

  ros::NodeHandle n;
  
  ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");
  
  //Declare an ik_service::PoseIK variable
  ik_service::PoseIK ik_pose;
  
  // Declare a geometry_msgs::Pose variable
  geometry_msgs::Pose part_pose;
  
  // Set the position.x field of the geometry_msgs::Pose variable to 0.5.
  part_pose.position.x = 0.5;
  // Set the other positions yourself 
  part_pose.position.y = 0.0;
  part_pose.position.z = 0.0;
  
  
  // Set the request field of the ik_service::PoseIK variable equal to the geometry_msgs::Pose variable.
  ik_pose.request.part_pose = part_pose;
  
  // Update the client.call() to use the ik_service::PoseIK variable.
  if (client.call(ik_pose))
  {
   // Finally, update the ROS_INFO() messages to indicate that the client.call() returned request.num_sols solutions
    ROS_INFO("Call to ik_service returned [%i] solutions", ik_pose.response.num_sols);
    
    // Add to the ROS_INFO() to show those set of joint angles
    for(int i = 0 ; i < ik_pose.response.num_sols ; i++){
    	ROS_INFO("Solution Num: %d", i+1);
   	   for(int j = 0 ; j < 6 ; j++){
   		ROS_INFO("Joint angle [%d]: %.4f ", j+1, ik_pose.response.joint_solutions[i].joint_angles[j]);
   	   }
   }
   
  }
  else
  {
  // Finally, update the ROS_INFO() messages to indicate that the client.call() failed
    ROS_ERROR("Failed to call service ik_service");
  }

  return 0;
}
