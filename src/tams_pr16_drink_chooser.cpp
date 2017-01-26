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
#include <project16_coordinator/CocktailAction.h>
#include <project16_manipulation/PourBottleAction.h>
#include <pr2016_msgs/CocktailList.h>


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

        sub_ = nh.subscribe("recognizedObjects", 1000, &DrinkChooser::objectsCallback, this);
        
        object_pub_ = nh.advertise<pr2016_msgs::CocktailList>("availableCocktails", 1000);

        pnh.getParam("cocktails", cocktails_);

        current_incr_ = 0;
        max_incr_ = 0;

        //Register action service
        as_ = new actionlib::SimpleActionServer<project16_coordinator::CocktailAction>(nh, "cocktail_mixer", boost::bind(&DrinkChooser::mix, this, _1), false);

        as_->start();


        try {
            for (int32_t i = 0; i < cocktails_.size(); ++i) {
                XmlRpc::XmlRpcValue& c = cocktails_[i].begin()->second;
                std::map <std::string, double> incr;

                for (int32_t j = 0; j < c["ingredients"].size(); ++j) {
                    std::string name = c["ingredients"][j]["incr"]["type"];
                    incr[name] = double(c["ingredients"][j]["incr"]["amount"]);
                }
                Cocktail c1(i, c["name"], incr);
                cocktails_db_.push_back(c1);
            }
        } catch (XmlRpc::XmlRpcException & e) {
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    void objectsCallback(const pr2016_msgs::BarCollisionObjectArrayConstPtr & msg) {
        bottles_.clear();
        if (msg->objects.size()) {
            for (unsigned int i = 0; i < msg->objects.size(); i += 1) {
                moveit_msgs::CollisionObject obj = msg->objects[i];
                std::stringstream ss;
                ss << obj.type.key;
                std::string key = ss.str();
                bottles_.push_back(key);
            }
        }
    }

    void mix(const project16_coordinator::CocktailGoalConstPtr& goal) {
        Cocktail ordered_cocktail;
        bool success = false;
        
        feedback_.task_state = "Received order for cocktail " + goal->cocktail;
        ROS_INFO_STREAM("Received order for cocktail " << goal->cocktail);
        as_->publishFeedback(feedback_);

        for (int i = 0; i < cocktails_db_.size(); i++) {
            if (cocktails_db_[i].getName() == goal->cocktail) {
                ordered_cocktail = cocktails_db_[i];
            }
        }

        if (ordered_cocktail.getName() == "") {
            ROS_ERROR_STREAM("Cocktail '" << goal->cocktail << "' not found");
            as_->setAborted();
            return;
        }

        actionlib::SimpleActionClient<project16_manipulation::PourBottleAction> ac("pour_bottle", true);
        ac.waitForServer();
        feedback_.task_state = "Start mixing " + ordered_cocktail.getName();
        ROS_INFO_STREAM("Start mixing " << ordered_cocktail.getName());
        as_->publishFeedback(feedback_);

        std::map<std::string, double> incr = ordered_cocktail.getIngredients();
        std::map<std::string, double>::iterator it;


        success_ = true;
        max_incr_ = incr.size();
        for (it = incr.begin(); (it != incr.end()) && success_; it++) {
            project16_manipulation::PourBottleGoal goal;
            current_incr_ = std::distance(incr.begin(), incr.find(it->first)) + 1;

            std::stringstream ss;
            ss << it->first;
            std::string name = ss.str();
            goal.bottle_id = name;
            goal.portion_size = double(it->second);

            ROS_INFO_STREAM("Mixing " << goal.portion_size << " cl " << goal.bottle_id);
            ac.sendGoal(goal,
                    boost::bind(&DrinkChooser::doneCB, this, _1, _2),
                    boost::bind(&DrinkChooser::actCB, this),
                    boost::bind(&DrinkChooser::feedCB, this, _1));
            ac.waitForResult();
        }

        if (success_) {
            feedback_.task_state = "Finished mixing " + ordered_cocktail.getName();
            ROS_INFO_STREAM("Finished mixing " << ordered_cocktail.getName());
        } else {
            feedback_.task_state = "Failed mixing " + ordered_cocktail.getName();
            ROS_INFO_STREAM("Failed mixing " << ordered_cocktail.getName());
        }
        as_->publishFeedback(feedback_);
        result_.success = success_;
        as_->setSucceeded(result_);
    }

    void doneCB(const actionlib::SimpleClientGoalState& state, const project16_manipulation::PourBottleResultConstPtr& result) {
        success_ = result->success;
    }

    void actCB() {

    }

    void feedCB(const project16_manipulation::PourBottleFeedbackConstPtr& feed) {
        std::stringstream ss;
        ss << current_incr_;
        ss << "/";
        ss << max_incr_;
        std::string step = ss.str();
        ROS_INFO_STREAM("" << step << " " << feed->task_state);
        feedback_.task_state = "" + step + " " + feed->task_state;
        as_->publishFeedback(feedback_);
    }

    void sendCocktailList() {
        pr2016_msgs::CocktailList clist;

        for (int i = 0; i < cocktails_db_.size(); i++) {
            bool avail = true;
            std::map<std::string, double> incr = cocktails_db_[i].getIngredients();
            std::map<std::string, double>::iterator it;
            
            for (it = incr.begin(); (it != incr.end()) && avail; it++) {
                std::stringstream ss;
                ss << it->first;
                std::string name = ss.str();
                bool incrAvail = false;
                for (int32_t j = 0; j < bottles_.size(); ++j) {
                    if (name == bottles_[j]) {
                        incrAvail = true;
                    }
                }
                if (!incrAvail) {
                    avail = false;
                }
            }
            clist.cocktails.push_back(cocktails_db_[i].getName());
            clist.available.push_back(avail);        
        }

        object_pub_.publish(clist);
    }

private:
    std::string mapFrameId_;
    std::string objFramePrefix_;
    ros::Subscriber sub_;
    ros::Publisher object_pub_;
    XmlRpc::XmlRpcValue cocktails_;
    std::vector <Cocktail> cocktails_db_;
    std::vector <std::string> bottles_;
    bool success_;
    actionlib::SimpleActionServer<project16_coordinator::CocktailAction>* as_;
    project16_coordinator::CocktailFeedback feedback_;
    project16_coordinator::CocktailResult result_;
    int current_incr_;
    int max_incr_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "DrinkChooser");

    DrinkChooser sync;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        sync.sendCocktailList();
        ros::spinOnce();
        loop_rate.sleep();        
    }
    ros::spin();
}
