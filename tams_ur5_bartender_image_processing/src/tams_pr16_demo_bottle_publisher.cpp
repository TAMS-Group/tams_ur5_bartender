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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>
#include <shape_msgs/SolidPrimitive.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <pr2016_msgs/BarCollisionObjectArray.h>

class Bottle {
public:

    Bottle() {
    }

    Bottle(std::string name, double height_bottle, double radius_bottle, double height_neck, double radius_neck, double height_label, double pos_x, double pos_y) :
    name_(name),
    height_bottle_(height_bottle),
    radius_bottle_(radius_bottle),
    height_neck_(height_neck),
    radius_neck_(radius_neck),
    height_label_(height_label),
    pos_x_(pos_x),
    pos_y_(pos_y){
    }

    std::string getName() const {
        return name_;
    }

    double getHeight_bottle() const {
        return height_bottle_;
    }

    double getRadius_bottle() const {
        return radius_bottle_;
    }

    double getHeight_neck() const {
        return height_neck_;
    }

    double getRadius_neck() const {
        return radius_neck_;
    }

    double getHeight_label() const {
        return height_label_;
    }

    double getPos_x() const {
        return pos_x_;
    }

    double getPos_y() const {
        return pos_y_;
    }

private:
    std::string name_;
    double height_bottle_;
    double radius_bottle_;
    double height_neck_;
    double radius_neck_;
    double height_label_;
    double pos_x_;
    double pos_y_;

};

class BottleRecognition {
public:

    BottleRecognition() : mapFrameId_("/table_top"), objFramePrefix_("object") {
        ros::NodeHandle pnh("~");
        pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        object_pub_ = nh.advertise<pr2016_msgs::BarCollisionObjectArray>("recognizedObjects", 1);

        pnh.getParam("bottles", bottles);

        try {
            for (int32_t i = 0; i < bottles.size(); ++i) {
                XmlRpc::XmlRpcValue& b = bottles[i].begin()->second;
                Bottle b1(b["name"],
                        double(b["height_bottle"]),
                        double(b["radius_bottle"]),
                        double(b["height_neck"]),
                        double(b["radius_neck"]),
                        double(b["height_label"]),
			double(b["pos_x"]),
			double(b["pos_y"]));
                data_bottles[int(b["id"])] = b1;
            }
        } catch (XmlRpc::XmlRpcException & e) {
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    void publishDemoObjects() {
        pr2016_msgs::BarCollisionObjectArray recObjectArr;
        //HEADER BAUEN
        recObjectArr.header.stamp = ros::Time::now();
        recObjectArr.header.frame_id = mapFrameId_;
        for (unsigned int i = 1; i < 3; i += 1) {
            moveit_msgs::CollisionObject Object;
            object_recognition_msgs::ObjectType Type;
            geometry_msgs::PoseStamped PoseStamped_bottle;
            geometry_msgs::Pose Pose_bottle, Pose_neck;
            geometry_msgs::Point Point_bottle, Point_neck;
            geometry_msgs::Quaternion Orient;
            shape_msgs::SolidPrimitive cylinder_bottle, cylinder_neck;

            //header is the geometric frame of the camera
            Object.header = recObjectArr.header;

            //set primitives
            cylinder_bottle.type = shape_msgs::SolidPrimitive::CYLINDER;
            cylinder_neck.type = shape_msgs::SolidPrimitive::CYLINDER;
            cylinder_bottle.dimensions.resize(2);
            cylinder_neck.dimensions.resize(2);

            int bottleID = i;
            //Bottle Group id i.e. 1,2,3 -> 0 
            std::string bottle_name;
            double height_bottle, radius_bottle, height_neck, radius_neck, height_label, pos_x, pos_y;

            if (data_bottles.find(bottleID) == data_bottles.end()) {
                ROS_ERROR("Can't find bottle in database");
                bottle_name = "unknown";
                height_bottle = 0.19;
                radius_bottle = 0.075 / 2;
                height_neck = 0.285;
                radius_neck = 0.025 / 2;
                height_label = 0.095;
            } else {
                bottle_name = data_bottles[bottleID].getName();
                height_bottle = data_bottles[bottleID].getHeight_bottle();
                radius_bottle = data_bottles[bottleID].getRadius_bottle();
                height_neck = data_bottles[bottleID].getHeight_neck();
                radius_neck = data_bottles[bottleID].getRadius_neck();
                height_label = data_bottles[bottleID].getHeight_label();
                pos_x = data_bottles[bottleID].getPos_x();
                pos_y = data_bottles[bottleID].getPos_y();
            }

            //set type
            Type.key = bottle_name;
            Object.type = Type;
            Object.id = bottle_name;

            //generate cylinders
            cylinder_bottle.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height_bottle;
            cylinder_bottle.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius_bottle;
            cylinder_neck.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height_neck;
            cylinder_neck.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius_neck;
            //add cylinders to object
            Object.primitives.push_back(cylinder_bottle);
            Object.primitives.push_back(cylinder_neck);

            //calculate bottle offset
            //BOTTLE POSE & ORIENT
            Pose_bottle.orientation.w = 1;
            Pose_bottle.position.x = pos_x;
            Pose_bottle.position.y = pos_y;
            Pose_bottle.position.z = height_bottle / 2;
            PoseStamped_bottle.pose = Pose_bottle;

            //move bottle neck up so it only sticks out ontop of the bottle
	    Pose_neck.orientation.w = 1;
            Point_neck.x = Pose_bottle.position.x;
            Point_neck.y = Pose_bottle.position.y;
            Point_neck.z = height_bottle + height_neck / 2;
            Pose_neck.position = Point_neck;

            //add modelpositions to Object
            Object.primitive_poses.push_back(Pose_bottle);
            Object.primitive_poses.push_back(Pose_neck);

            //add object to vector
            recObjectArr.objects.push_back(Object);
        }
        object_pub_.publish(recObjectArr);

    }

private:
    std::string mapFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber sub_;
    tf::TransformListener tfListener_;
    ros::Publisher object_pub_;
    XmlRpc::XmlRpcValue bottles;
    std::map <int, Bottle> data_bottles;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "recognizedObjectsListener");
    BottleRecognition sync;
    ros::Rate loop_rate(1);
    while (ros::ok()) {
        sync.publishDemoObjects();
        loop_rate.sleep();
    }
}
