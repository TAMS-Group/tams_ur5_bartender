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


// Cocktail Object
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
        
        //register for recognized Objects 
        sub_ = nh.subscribe("recognizedObjects", 1000, &DrinkChooser::objectsCallback, this);
        
        //register publisher for available cocktails
        object_pub_ = nh.advertise<pr2016_msgs::CocktailList>("availableCocktails", 1, true);

        pnh.getParam("cocktails", cocktails_);

        current_ingr_ = 0;
        max_ingr_ = 0;

        //Register action service
        as_ = new actionlib::SimpleActionServer<project16_coordinator::CocktailAction>(nh, "cocktail_mixer", boost::bind(&DrinkChooser::mix, this, _1), false);

        as_->start();

        //put all cocktails to internal DB
        try {
            for (int32_t i = 0; i < cocktails_.size(); ++i) {
                XmlRpc::XmlRpcValue& c = cocktails_[i].begin()->second;
                std::map <std::string, double> ingr;

                for (int32_t j = 0; j < c["ingredients"].size(); ++j) {
                    std::string name = c["ingredients"][j]["ingr"]["type"];
                    ingr[name] = double(c["ingredients"][j]["ingr"]["amount"]);
                }
                Cocktail c1(i, c["name"], ingr);
                cocktails_db_.push_back(c1);
            }
        } catch (XmlRpc::XmlRpcException & e) {
            ROS_WARN("%s", e.getMessage().c_str());
        }
    }

    //Save bottles to internal Bottle DB
    void objectsCallback(const pr2016_msgs::BarCollisionObjectArrayConstPtr & msg) {
        //delete old db
        bottles_.clear();
        if (msg->objects.size()) {
            for (unsigned int i = 0; i < msg->objects.size(); i += 1) {
                moveit_msgs::CollisionObject obj = msg->objects[i];
                std::stringstream ss;
                ss << obj.type.key;
                std::string key = ss.str();
                //save bottle name to db
                bottles_.push_back(key);
            }
        }
    }

    // mixing action
    void mix(const project16_coordinator::CocktailGoalConstPtr& goal) {
        Cocktail ordered_cocktail;
        bool success = false;
        
        //feedback received order
        feedback_.task_state = "Received order for cocktail " + goal->cocktail;
        ROS_INFO_STREAM("Received order for cocktail " << goal->cocktail);
        as_->publishFeedback(feedback_);

        //find cocktail in db
        for (int i = 0; i < cocktails_db_.size(); i++) {
            if (cocktails_db_[i].getName() == goal->cocktail) {
                ordered_cocktail = cocktails_db_[i];
            }
        }

        //abort if cocktail not available
        if (ordered_cocktail.getName() == "") {
            ROS_ERROR_STREAM("Cocktail '" << goal->cocktail << "' not found");
            feedback_.task_state = "Cocktail '" + goal->cocktail + "' not found";
            as_->publishFeedback(feedback_);
            as_->setAborted();
            return;
        }

        //call pour bottle action server
        actionlib::SimpleActionClient<project16_manipulation::PourBottleAction> ac("pour_bottle", true);
        ac.waitForServer();
        
        //send feedback
        feedback_.task_state = "Start mixing " + ordered_cocktail.getName();
        ROS_INFO_STREAM("Start mixing " << ordered_cocktail.getName());
        as_->publishFeedback(feedback_);

        std::map<std::string, double> ingr = ordered_cocktail.getIngredients();
        std::map<std::string, double>::iterator it;

        success_ = true;
        max_ingr_ = ingr.size();
        //iterate over ingredients of cocktail
        for (it = ingr.begin(); (it != ingr.end()) && success_; it++) {
            project16_manipulation::PourBottleGoal goal;
            current_ingr_ = std::distance(ingr.begin(), ingr.find(it->first)) + 1;

            std::stringstream ss;
            ss << it->first;
            std::string name = ss.str();
            goal.bottle_id = name;
            goal.portion_size = double(it->second);

            ROS_INFO_STREAM("Mixing " << goal.portion_size << " cl " << goal.bottle_id);
            //feedback_.task_state = "Mixing " + goal.portion_size + " cl " + goal.bottle_id;
            //as_->publishFeedback(feedback_);
            //send mixing goal of actual ingredient
            ac.sendGoal(goal,
                    boost::bind(&DrinkChooser::doneCB, this, _1, _2),
                    boost::bind(&DrinkChooser::actCB, this),
                    boost::bind(&DrinkChooser::feedCB, this, _1));
            //wait for finish
            ac.waitForResult();
        }

        //set status regarding the result
        if (success_) {
            feedback_.task_state = "Finished mixing " + ordered_cocktail.getName();
            ROS_INFO_STREAM("Finished mixing " << ordered_cocktail.getName());
        } else {
            feedback_.task_state = "Failed mixing " + ordered_cocktail.getName();
            ROS_INFO_STREAM("Failed mixing " << ordered_cocktail.getName());
        }
        //publish feedback if finished
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
        //on feedback publish feedback on own feedback topic
        std::stringstream ss;
        ss << current_ingr_;
        ss << "/";
        ss << max_ingr_;
        std::string step = ss.str();
        ROS_INFO_STREAM("" << step << " " << feed->task_state);
        feedback_.task_state = "" + step + " " + feed->task_state;
        as_->publishFeedback(feedback_);
    }

    //generate available cocktail list
    void sendCocktailList() {
        pr2016_msgs::CocktailList clist;
        bool iteratedThroughAllBottles = false;
        //iterate over all cocktails
        for (int i = 0; i < cocktails_db_.size(); i++) {
            bool avail = true;
            std::map<std::string, double> ingr = cocktails_db_[i].getIngredients();
            std::map<std::string, double>::iterator it;
            
            //iterate over all ingredients 
            for (it = ingr.begin(); (it != ingr.end()) && avail; it++) {
                std::stringstream ss;
                ss << it->first;
                std::string name = ss.str();
                bool ingrAvail = false;
                //check if bottle available
                for (int32_t j = 0; j < bottles_.size(); ++j) {
                    if (name == bottles_[j]) {
                        ingrAvail = true;
                    }
                    //Add all available bottles to msg ONCE (on first for-loop)
                    if(!iteratedThroughAllBottles){
                        clist.recognizedBottles.push_back(bottles_[j]);
                    }
                }
                iteratedThroughAllBottles = true;
                if (!ingrAvail) {
                    avail = false;
                }
            }
            //add cocktail to cocktaillist
            clist.cocktails.push_back(cocktails_db_[i].getName());
            //true or false if mixable
            clist.available.push_back(avail);        
        }

        //publish topic
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
    int current_ingr_;
    int max_ingr_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "DrinkChooser");

    DrinkChooser sync;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        //call send cocktail list
        sync.sendCocktailList();
        //spin once so callbacks could be called
        ros::spinOnce();
        //sleep the rest of the time
        loop_rate.sleep();        
    }
    ros::spin();
}
