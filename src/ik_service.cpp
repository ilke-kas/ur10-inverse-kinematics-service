#include "ros/ros.h"
#include "ik_service/PoseIK.h"
#include "ur_kinematics/ur_kin.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool pose_ik(ik_service::PoseIK::Request  &req,
        ik_service::PoseIK::Response &res)
{
 //Set the req.num_sols equal to -1 for testing purposes.
 // res.num_sols = -1;
  ROS_INFO("pose_ik service was called.");

  // Get the part pose in tf2:Tramsform type upon which math may be performed.
  tf2::Transform t1;

  // This takes the geometry_msgs/Pose structure and puts it in the tf2:Transform structure.
  tf2::fromMsg(req.part_pose, t1);

  // This is the rotation to orient the vacuum gripper in the desired reference pose.
  tf2::Quaternion q_rot(-0.5, 0.5, 0.5, 0.5);

  // The order of multiplication matters for quaternions.
  t1.setRotation(t1.getRotation() * q_rot);
  
 // Second, 2D way to define the same T Matrix
  double T[4][4] = {{t1.getBasis()[0][0], t1.getBasis()[0][1], t1.getBasis()[0][2], t1.getOrigin()[0]}, \
  		    {t1.getBasis()[1][0], t1.getBasis()[1][1], t1.getBasis()[1][2], t1.getOrigin()[1]}, \
		    {t1.getBasis()[2][0], t1.getBasis()[2][1], t1.getBasis()[2][2], t1.getOrigin()[2]}, \
		    {0.0, 0.0, 0.0, 1.0}};
        
 // Variable to receive the number of solutions returned
 int num_sol;
 // Allocate space for up to eight solutions of six joint angles, make it double
 double q_sols[8][6];
 
 // Inverse kinematic solution(s)
 num_sol = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0);
 
 if(num_sol == 0){
    ROS_ERROR("Inverse Kinematics algorithm could not find a solution for Target position: (%.2f, %.2f, %.2f)", req.part_pose.position.x, req.part_pose.position.y, req.part_pose.position.z );
    return false; 
 }
 else{
   res.num_sols = num_sol;
   
   for(int i = 0 ; i < num_sol ; i++){
   	   for(int j = 0 ; j < 6 ; j++){
   		res.joint_solutions[i].joint_angles[j] = q_sols[i][j];
   	   }
   }
   
   ROS_INFO("Number of solutions inverse kinematics solutions: %d", num_sol);
   return true;
  
 }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_ik_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("pose_ik", pose_ik);
  ROS_INFO("Ready to pose ik.");
  ros::spin();

  return 0;
}
