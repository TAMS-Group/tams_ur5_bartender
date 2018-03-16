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
#include <tams_ur5_bartender_msgs/BarCollisionObjectArray.h>

class Bottle {
public:

    Bottle() {
    }

    Bottle(std::string name, double height_bottle, double radius_bottle, double height_neck, double radius_neck, double height_label) :
    name_(name),
    height_bottle_(height_bottle),
    radius_bottle_(radius_bottle),
    height_neck_(height_neck),
    radius_neck_(radius_neck),
    height_label_(height_label) {
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

private:
    std::string name_;
    double height_bottle_;
    double radius_bottle_;
    double height_neck_;
    double radius_neck_;
    double height_label_;

};

class BottleRecognition {
public:

    BottleRecognition() : mapFrameId_("/table_top"), objFramePrefix_("object") {
        ros::NodeHandle pnh("~");
        pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        object_pub_ = nh.advertise<tams_ur5_bartender_msgs::BarCollisionObjectArray>("recognizedObjects", 1);

        sub_ = nh.subscribe("objectsStamped", 1, &BottleRecognition::objectsRecognizedCallback, this);

        pnh.getParam("bottles", bottles);

        bottle_cache_time = 10;

        try {
            for (int32_t i = 0; i < bottles.size(); ++i) {
                XmlRpc::XmlRpcValue& b = bottles[i].begin()->second;
                Bottle b1(b["name"],
                        double(b["height_bottle"]),
                        double(b["radius_bottle"]),
                        double(b["height_neck"]),
                        double(b["radius_neck"]),
                        double(b["height_label"]));
                data_bottles[int(b["id"])] = b1;
            }
        } catch (XmlRpc::XmlRpcException & e) {
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    void objectsRecognizedCallback(const find_object_2d::ObjectsStampedConstPtr & msg) {

        //Topic publishing here------------------------------------------------
        ROS_DEBUG_STREAM_COND(msg->objects.data.size() <= 0, "No bottles detected by find_object_2D");
        if (msg->objects.data.size()) {
            recObjectArr_.header = msg->header;
            recObjectArr_.header.frame_id = mapFrameId_;
            for (unsigned int i = 0; i < msg->objects.data.size(); i += 12) {
                moveit_msgs::CollisionObject Object;
                object_recognition_msgs::ObjectType Type;
                geometry_msgs::PoseStamped PoseStamped_bottle;
                geometry_msgs::Pose Pose_bottle, Pose_neck;
                geometry_msgs::Point Point_bottle, Point_neck;
                geometry_msgs::Quaternion Orient;
                shape_msgs::SolidPrimitive cylinder_bottle, cylinder_neck;

                //header is the geometric frame of the camera
                Object.header = msg->header;
                Object.header.frame_id = mapFrameId_;

                //set primitives
                cylinder_bottle.type = shape_msgs::SolidPrimitive::CYLINDER;
                cylinder_neck.type = shape_msgs::SolidPrimitive::CYLINDER;
                cylinder_bottle.dimensions.resize(2);
                cylinder_neck.dimensions.resize(2);

                int bottleID = (int) msg->objects.data[i];
                //Bottle Group id i.e. 1,2,3 -> 0 
                bottleID = bottleID / 10;
                std::string bottle_name;
                double height_bottle, radius_bottle, height_neck, radius_neck, height_label;

                if (data_bottles.find(bottleID) == data_bottles.end()) {
                    ROS_ERROR("Can't find bottle in database");
                    bottle_name = "unknown";
                    height_bottle = 0.19;
                    radius_bottle = 0.075 / 2;
                    height_neck = 0.095;
                    radius_neck = 0.025 / 2;
                    height_label = 0.095;
                } else {
                    bottle_name = data_bottles[bottleID].getName();
                    height_bottle = data_bottles[bottleID].getHeight_bottle();
                    radius_bottle = data_bottles[bottleID].getRadius_bottle();
                    height_neck = data_bottles[bottleID].getHeight_neck();
                    radius_neck = data_bottles[bottleID].getRadius_neck();
                    height_label = data_bottles[bottleID].getHeight_label();
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

                //set transform frame name for lookup
                std::ostringstream os;
                os << "object_";
                os << (int) msg->objects.data[i];

                //calculate bottle offset
                PoseStamped_bottle.header = msg->header;
                PoseStamped_bottle.header.frame_id = os.str();
                Point_bottle.x = -1.5*radius_bottle;
                Point_bottle.y = 0;
                Point_bottle.z = height_bottle / 2 - height_label;
                Orient.w = 1.0;
                Pose_bottle.orientation = Orient;
                Pose_bottle.position = Point_bottle;
                PoseStamped_bottle.pose = Pose_bottle;

                //calculate transformed pose to table top
                geometry_msgs::PoseStamped transformed_pose;
                try {
                    tfListener_.waitForTransform(mapFrameId_, os.str(), msg->header.stamp, ros::Duration(0, 5));
                } catch (tf::TransformException & ex) {
                    ROS_WARN("Wait for Transform Error: %s", ex.what());
                    continue;
                }
                try {
                    tfListener_.transformPose(mapFrameId_, ros::Time(0), PoseStamped_bottle, os.str(), transformed_pose);
                } catch (tf::TransformException & ex) {
                    ROS_WARN("Transform Pose Error: %s", ex.what());
                    continue;
                }
                //eliminate rotation around X and Y
                double roll, pitch, yaw;
                tf::Quaternion transformedQaternion(transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);
                tf::Matrix3x3 transformedMatrix(transformedQaternion);
                transformedMatrix.getRPY(roll, pitch, yaw);
                transformedQaternion.setRPY(0.0, 0.0, yaw);
                //transformed_pose.pose.orientation.x = transformedQaternion.getAxis().getX();
                //transformed_pose.pose.orientation.y = transformedQaternion.getAxis().getY();
                //transformed_pose.pose.orientation.z = transformedQaternion.getAxis().getZ();
                //transformed_pose.pose.orientation.w = transformedQaternion.getW();
                //remove rotation
                transformed_pose.pose.orientation.x = 0.0;
                transformed_pose.pose.orientation.y = 0.0;
                transformed_pose.pose.orientation.z = 0.0;
                transformed_pose.pose.orientation.w = 1.0;

                //Workaround due to colliding bottles with the table
                transformed_pose.pose.position.z = height_bottle / 2;

                Pose_bottle = transformed_pose.pose;
                Pose_neck = transformed_pose.pose;

                //move bottle neck up so it only sticks out ontop of the bottle
                Point_neck.x = transformed_pose.pose.position.x;
                Point_neck.y = transformed_pose.pose.position.y;
                Point_neck.z = transformed_pose.pose.position.z + (height_neck + height_bottle) / 2;
                Pose_neck.position = Point_neck;

                //add modelpositions to Object
                Object.primitive_poses.push_back(Pose_bottle);
                Object.primitive_poses.push_back(Pose_neck);

                //Check if object is already in array or colliding with cached bottle
                updateBottle(Object);
            }
        }
        //cleanup bottles before publish
        cleanupBottles();
        //Publish bottles
        object_pub_.publish(recObjectArr_);
        ROS_DEBUG_STREAM_COND((msg->objects.data.size() / 12) > recObjectArr_.objects.size(), "Find_object_2D bottles does not match recognized bottles. Bottles are missing");
        ROS_DEBUG_STREAM_COND((msg->objects.data.size() / 12) < recObjectArr_.objects.size(), "Find_object_2D bottles does not match recognized bottles. Using cached Bottles");

    }

    //update bottle in cache

    void updateBottle(moveit_msgs::CollisionObject Object) {
        bool new_bottle = true;
        //Check cached bottles
        std::stringstream ss;
        ss << Object.type.key;
        std::string name = ss.str();
        for (int32_t i = 0; i < recObjectArr_.objects.size(); ++i) {
            if (recObjectArr_.objects[i].type.key == Object.type.key) {
                //if names are the same update object
                recObjectArr_.objects[i] = Object;
                bottle_timeout_[name] = ros::Time::now();
                new_bottle = false;
            }
                //If new bottle on top of old (cached) bottle, delete cached bottle
            else if (doBottlesCollide(recObjectArr_.objects[i], Object)) {
                recObjectArr_.objects.erase(recObjectArr_.objects.begin() + i);
                ROS_INFO_STREAM("Deleted cached bottle " << recObjectArr_.objects[i].type.key << " due to collision with " << Object.type.key << ".");
            }
        }
        //Else create new object
        if (new_bottle) {
            recObjectArr_.objects.push_back(Object);
            bottle_timeout_[name] = ros::Time::now();
        }
    }

    //delete old cache items

    void cleanupBottles() {
        for (int i = 0; i < recObjectArr_.objects.size(); ++i) {
            std::stringstream ss;
            ss << recObjectArr_.objects[i].type.key;
            std::string name = ss.str();
            if (bottle_timeout_[name].toSec() + bottle_cache_time < ros::Time::now().toSec()) {
                recObjectArr_.objects.erase(recObjectArr_.objects.begin() + i);
                ROS_INFO_STREAM("Timeout: Bottle " << recObjectArr_.objects[i].type.key << " deleted.");
            }
        }
    }

    //check for collision

    bool doBottlesCollide(moveit_msgs::CollisionObject oldObject, moveit_msgs::CollisionObject newObject) {
        bool collision = false;
        float minimal_distance;
        //Check if bottles are on the same position
        float x1, y1, z1, x2, y2, z2;
        x1 = oldObject.primitive_poses[0].position.x;
        y1 = oldObject.primitive_poses[0].position.y;
        z1 = oldObject.primitive_poses[0].position.z;
        x2 = newObject.primitive_poses[0].position.x;
        y2 = newObject.primitive_poses[0].position.y;
        z2 = newObject.primitive_poses[0].position.z;
        minimal_distance = oldObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] + newObject.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
        if (minimal_distance >= sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0) + pow(z1 - z2, 2.0))) {
            collision = true;
        }
        return collision;
    }

private:
    std::string mapFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber sub_;
    tf::TransformListener tfListener_;
    ros::Publisher object_pub_;
    XmlRpc::XmlRpcValue bottles;
    std::map <int, Bottle> data_bottles;
    tams_ur5_bartender_msgs::BarCollisionObjectArray recObjectArr_;
    std::map <std::string, ros::Time> bottle_timeout_;
    int bottle_cache_time;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "recognizedObjectsListener");

    BottleRecognition sync;

    ros::spin();
}
