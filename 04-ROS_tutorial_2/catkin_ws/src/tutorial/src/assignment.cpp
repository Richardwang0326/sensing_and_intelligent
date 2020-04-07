#include <ros/ros.h>
#include <tutorial/assignment.h>
#include <tf/transform_listener.h>
#include <string>
#include <cmath>

/* 
   Lab4 assignment
   Fill your code in Student Implementation part
   Last editted: 10/1, 2018
   Editor: Sean Lu
*/

// Check if given two IDs are in range
bool id_in_range(int id1, int id2){
  bool result;
  /************************************************************
  //                                                         //
  //                                                         //
  //                Student Implementation                   //
  //                                                         //
  //                                                         //
  ************************************************************/
  if((id1>=0 && id1<=15) && (id2>=0 && id2<=15))
  	result = true;
  else 
	result = false;
  return result;
}

// Calculate the distance with given transform
double calculateDistance(tf::Transform transform){
  double dist;
  /************************************************************
  //                                                         //
  //                                                         //
  //                Student Implementation                   //
  //                                                         //
  //                                                         //
  ************************************************************/
  double x,y,z;
  x = transform.getOrigin().x();
  y = transform.getOrigin().y();
  z = transform.getOrigin().z();
  dist = sqrt(x * x + y * y + z * z);
  return dist;
}

// Service callback
bool serviceCb(tutorial::assignment::Request  &req, 
               tutorial::assignment::Response &res){
  if(!id_in_range(req.tag1_id, req.tag2_id)){
    res.result = "Tag ID out of range. Should be 0 through 15.";
    return 1;
  }
  else{
    std::string tag1_str = "tag_", tag2_str = "tag_", parent = "camera_link";
    tag1_str += std::to_string(req.tag1_id);
    tag2_str += std::to_string(req.tag2_id);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
      if(listener.waitForTransform(tag1_str, tag2_str, 
                                   ros::Time::now(), 
                                   ros::Duration(5.0)))
         {
         /************************************************************
  	 //                                                         //
  	 //                                                         //
  	 //                Student Implementation                   //
 	 //                                                         //
 	 //                                                         //
 	 ************************************************************/
  	 listener.lookupTransform(tag1_str, tag2_str, ros::Time(0), transform);
	 }

      else{
        res.result = "Times out.";
        return 1;
      }
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      res.result = "Exception.";
      return 1;
    }
    res.distance = calculateDistance(transform);
    res.result = "Success.";
    return 1;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "assignment_node");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("assignment", serviceCb);
  ROS_INFO("Start to find the distance between two given tags:");
  ros::spin();

  return 0;
}
