/*
 * Copyright (c) 2009, CoroWare
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Stanford U. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>
#include <HectorMapTools.h>
#include <math.h>
#include <tf/transform_datatypes.h>

using namespace std;

//resolution of the map
double resolution = 0.05; 

/**
 * @brief This node provides images as occupancy grid maps.
 */
class MapAsImageProvider
{

public:

  // Publisher for the map
  ros::Publisher map_publisher;

  //Subscriber to the image topic
  image_transport::Subscriber image_transport_subscriber_map;

  image_transport::ImageTransport* image_transport_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;


  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    //subscribe to the image topic representing the map
    image_transport_subscriber_map = image_transport_->subscribe("map_image_raw", 1, &MapAsImageProvider::mapCallback,this); 
    // publish the map as an occupancy grid
    map_publisher = n_.advertise<nav_msgs::OccupancyGrid>("/map_from_jpeg", 50); 


    ROS_INFO("Image to Map node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }


  //The map->image conversion runs every time a new map is received
  void mapCallback(const sensor_msgs::ImageConstPtr& image)
  {

    nav_msgs::OccupancyGrid map;
    //fill in some map parameters
    map.header.stamp = image->header.stamp;
    map.header.frame_id = "map";
    map.info.width = image->width;
    map.info.height = image->height;
    map.info.origin.orientation.w = 1;
    map.info.resolution = resolution;
    map.info.origin.position.x = -((image->width + 1) * map.info.resolution * 0.5f);
    map.info.origin.position.y = -((image->height + 1) * map.info.resolution * 0.5f);

    //read the pixels of the image and fill the map table
    int data;
    for(int i=image->height -1; i>=0; i--)
    {
	for (unsigned int j=0; j<image->width;j++)
	{
	    data = image->data[i*image->width + j];
	    if(data >=123 && data <= 131){
	    	map.data.push_back(-1);
	    }
	    else if (data >=251 && data <= 259){
		map.data.push_back(0);
	    }
	    else
		map.data.push_back(100);
	}
    }
    //publish the map
    map_publisher.publish(map); 
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_to_map_node");

  ros::NodeHandle nh("~");
  nh.param("resolution", resolution, 0.05); 

  // The 
  MapAsImageProvider map_image_provider;

  ros::spin();

  return 0;
}
