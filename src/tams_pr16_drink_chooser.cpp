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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <project16_coordinator/cocktailAction.h>
#include <project16_manipulation/PourBottleAction.h>


//TO-DO hardcoded array of need IDS - find in recognized objects and print results for testing!

class Cocktail {
public:

    Cocktail() {
    }

    Cocktail(int id, std::string name, std::map<std::string, double> ingredients) :
    id_(id),
    name_(name),
    ingredients_(ingredients) {
    }

    int getId() const {
        return id_;
    }

    std::string getName() const {
        return name_;
    }

    std::map<std::string, double> getIngredients() const {
        return ingredients_;
    }

private:
    int id_;
    std::string name_;
    std::map <std::string, double> ingredients_;

};

class DrinkChooser {
public:

    DrinkChooser() {
        ros::NodeHandle pnh("~");

        ros::NodeHandle nh;

        sub_ = nh.subscribe("recognizedObjects", 10, &DrinkChooser::objectsCallback, this);

        pnh.getParam("cocktails", cocktails);

        //Register action service
        as_ = new actionlib::SimpleActionServer<project16_coordinator::CocktailAction>(nh, "cocktail_mixer", boost::bind(&DrinkChooser::mix, this, _1), false);

        as_->start();


        try {
            for (int32_t i = 0; i < cocktails.size(); ++i) {
                XmlRpc::XmlRpcValue& c = cocktails[i].begin()->second;
                std::map <std::string, double> incr;

                for (int32_t j = 0; j < c["ingredients"].size(); ++j) {
                    std::string name = c["ingredients"][i]["incr"]["type"];
                    incr[name] = double(c["ingredients"][i]["incr"]["amount"]);
                }
                Cocktail c1(i, c["name"], incr);
                cocktails_db.push_back(c1);
            }
        } catch (XmlRpc::XmlRpcException & e) {
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    void objectsCallback(const pr2016_msgs::BarCollisionObjectArrayConstPtr & msg) {
        bottles.clear();
        if (msg->objects.size()) {
            for (unsigned int i = 0; i < msg->objects.size(); i += 1) {
                moveit_msgs::CollisionObject obj = msg->objects[i];
                std::stringstream ss;
                ss << obj.type.key;
                std::string key = ss.str();
                bottles.push_back(key);
            }
        }
    }

    void execute(const project16_coordinator::CocktailConstPtr& goal) {
        Cocktail ordered_cocktail;
        bool success = false;

        for (int i = 0; i < cocktails_db.size(); i++) {
            if (cocktails_db[i].getName() == goal->cocktail) {
                ordered_cocktail = cocktails_db[i];
            }
        }

        if (ordered_cocktail == NULL) {
            as_->setAborted();
            return;
        }
        feedback.task_state = "Start mixing " + ordered_cocktail.getName();
        as_->publishFeedback(feedback);

        std::map<std::string, double> incr = ordered_cocktail.getIngredients();
        for (int i = 0; i < incr.size(); i++) {
            

        }

        if (success) {
            feedback.task_state = "Finished mixing " + ordered_cocktail.getName();
            as_->publishFeedback(feedback);
            result_.success = true;
            as_.setSucceeded(result_);
        }
    }
}


private:
std::string mapFrameId_;
std::string objFramePrefix_;
ros::Subscriber sub_;
XmlRpc::XmlRpcValue cocktails;
std::vector <Cocktail> cocktails_db;
std::vector <std::string> bottles;
project16_coordinator::CocktailFeedback feedback_;
project16_coordinator::CocktailResult result_;

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "DrinkChooser");

    DrinkChooser sync;

    ros::Rate loop_rate(1);
    ros::spin();
}
