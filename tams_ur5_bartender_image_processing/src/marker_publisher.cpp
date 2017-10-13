/*
Copyright (c) 2017, Daniel Ahlers, Lars Henning Kayser, Jeremias Hartz, Maham Tanveer, Oke Martensen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <object_recognition_msgs/ObjectType.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <shape_msgs/SolidPrimitive.h>
#include <pr2016_msgs/BarCollisionObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

class MarkerPublisher {
public:

    MarkerPublisher() : mapFrameId_("/table_top"), objFramePrefix_("object") {
        ros::NodeHandle pnh("~");
        pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

        // Subscribe to recgnized Objects topics
        sub_ = nh.subscribe("recognizedObjects", 10, &MarkerPublisher::objectsCallback, this);
    }

    //Publish markers for bottles

    void objectsCallback(const pr2016_msgs::BarCollisionObjectArrayConstPtr & msg) {
        visualization_msgs::MarkerArray markerArr;
        if (msg->objects.size()) {
            for (unsigned int i = 0; i < msg->objects.size(); i += 1) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = mapFrameId_;
                marker.header.stamp = ros::Time(0);
                marker.ns = "my_namespace";
                marker.id = 9 + i * 10;
                
                //timeout duration set to 5 sec
                ros::Duration dur(2.0);
                marker.lifetime = dur;

                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

                marker.text = msg->objects[i].id;

                marker.pose.position.x = msg->objects[i].primitive_poses[0].position.x;
                marker.pose.position.y = msg->objects[i].primitive_poses[0].position.y;
                marker.pose.position.z = msg->objects[i].primitive_poses[0].position.z+0.2;
                marker.pose.orientation.x = msg->objects[i].primitive_poses[0].orientation.x;
                marker.pose.orientation.y = msg->objects[i].primitive_poses[0].orientation.y;
                marker.pose.orientation.z = msg->objects[i].primitive_poses[0].orientation.z;
                marker.pose.orientation.w = msg->objects[i].primitive_poses[0].orientation.w;
                marker.scale.z = 0.06;
                
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                
                markerArr.markers.push_back(marker);
                marker.text = "";

                for (unsigned int j = 0; j < msg->objects[i].primitives.size(); j += 1) {
                    //create marker
                    marker.id = j + i * 10;
                    
                    marker.color.a = 0.75;
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;

                    marker.type = msg->objects[i].primitives[j].type;
                    marker.action = visualization_msgs::Marker::ADD;

                    //set marker position & orientation
                    marker.pose.position.x = msg->objects[i].primitive_poses[j].position.x;
                    marker.pose.position.y = msg->objects[i].primitive_poses[j].position.y;
                    marker.pose.position.z = msg->objects[i].primitive_poses[j].position.z;
                    marker.pose.orientation.x = msg->objects[i].primitive_poses[j].orientation.x;
                    marker.pose.orientation.y = msg->objects[i].primitive_poses[j].orientation.y;
                    marker.pose.orientation.z = msg->objects[i].primitive_poses[j].orientation.z;
                    marker.pose.orientation.w = msg->objects[i].primitive_poses[j].orientation.w;
                    marker.scale.x = 2 * msg->objects[i].primitives[j].dimensions[1];
                    marker.scale.y = 2 * msg->objects[i].primitives[j].dimensions[1];
                    marker.scale.z = msg->objects[i].primitives[j].dimensions[0];

                    markerArr.markers.push_back(marker);
                }


            }
        }
        marker_pub_.publish(markerArr);
    }
private:
    std::string mapFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber sub_;
    ros::Publisher marker_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "TestMarkerPublisher");

    MarkerPublisher sync;

    ros::Rate loop_rate(10);
    ros::spin();
}
