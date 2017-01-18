#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <object_recognition_msgs/ObjectType.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <shape_msgs/SolidPrimitive.h>
#include <pr2016_msgs/BarCollisionObjectArray.h>
#include <visualization_msgs/Marker.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

//TO-DO hardcoded array of need IDS - find in recognized objects and print results for testing!

class Cocktail{
public:
    Cocktail(){}
    Cocktail(int id, std::string name, std::map<std::string, double>  ingredients):
    id_(id),
    name_(name),
    ingredients_(ingredients)
    {}
    
    int getId() const {return id_;}
    std::string getName() const {return name_;}
    std::map<std::string, double>  getIngredients() const {return ingredients_;}

private:
    int id_;
    std::string name_;
    std::map <std::string, double> ingredients_;

};

class DrinkChooser {
public:

    DrinkChooser() : mapFrameId_("/table_top"), objFramePrefix_("object") {
        ros::NodeHandle pnh("~");
        pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
        pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

        ros::NodeHandle nh;
        //        marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

        sub_ = nh.subscribe("recognizedObjects", 10, &DrinkChooser::objectsCallback, this);
        
        pnh.getParam("cocktails", cocktails);
        
        try {
            for(int32_t i = 0; i < cocktails.size(); ++i) {
                XmlRpc::XmlRpcValue& c = cocktails[i].begin()->second;
                std::map <std::string, double> incr;
                for(int32_t j = 0; j < c["ingredients"].begin()->second.size(); ++j) {
                    XmlRpc::XmlRpcValue& in = c["ingredients"].begin()->second[i].begin()->second;
                    incr[in["type"]] = double(in["amount"]);
                    //ROS_WARN_STREAM(<< in["type"]);
                    ROS_WARN("%f", double(in["amount"]));
                }
                
                Cocktail c1(i,c["name"],incr);
                cocktails_db.push_back(c1);
            }
        }
        catch(XmlRpc::XmlRpcException & e){
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    void objectsCallback(const pr2016_msgs::BarCollisionObjectArrayConstPtr & msg) {
        unsigned int balance[] = {10, 2, 3, 7, 5};
        if (msg->objects.size()) {
            for (int z = 0; z < 10; z++) {
                for (unsigned int i = 0; i < msg->objects.size(); i += 1) {
                    for (unsigned int j = 0; j < msg->objects[i].primitives.size(); j += 1) {
                        ROS_INFO("%s","Test");
//                        if (n[ z ] ==) {
//                            marker.header.frame_id = mapFrameId_;
//                            marker.id = j + i * 10;
//                            marker_pub_.publish(marker);
//                            n[ i ] = i + 100;
//                        }
                    }
                }
            }
        }
    }
private:
    std::string mapFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber sub_;
    XmlRpc::XmlRpcValue cocktails;
    std::vector <Cocktail> cocktails_db;
    //    ros::Publisher marker_pub_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "DrinkChooser");

    DrinkChooser sync;

    ros::Rate loop_rate(10);
    ros::spin();
}
