#include "cas726/mapper.hh"
#include <iostream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h> 

void cas726::Mapper::laser_callback(const sensor_msgs::msg::LaserScan &scan) {
  //RCLCPP_INFO(this->get_logger(), "Got laser message with %ld ranges", scan.ranges.size());

  //TODO: implement laser callback
  // 0. lookup transform on TF
  // 1. calculate transform as a Pose2 object
  // 2. iterate through 
  // 	a. all map cells and project them to the laser frame
  // or b. all rays and trace through map
  // 3. update map cells 

}

//create a message and publish to map update topic      
void cas726::Mapper::map_update_callback() {
  //RCLCPP_INFO(this->get_logger(), "Updating map");

  //update header
  map_msg_.info.map_load_time = now();
  map_msg_.header.stamp = now();
  this->update_map_msg();
  map_publisher_->publish(map_msg_);
}

//update map to message
void cas726::Mapper::update_map_msg() {
  int8_t FREE=0;
  int8_t OCC=100;
  int8_t UNKN=-1;
  map_msg_.data.clear();
  //TODO here iterate through map cells and push them into message 

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node = std::make_shared<cas726::Mapper>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
