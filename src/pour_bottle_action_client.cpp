#include <project16_manipulation/PourBottleAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<project16_manipulation::PourBottleAction> Client;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pour_bottle_client");
	Client client("pour_bottle", true); // true -> don't need ros::spin()
	client.waitForServer();
	project16_manipulation::PourBottleGoal goal;
	
	// Fill in goal here
	goal.bottle_id = "bottle";
	goal.portion_size = 10.0;
	
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && client.getResult()->success)
		printf("Yay! The Bottle has been poured!");
    else
		printf("Noo! Something failed!");
	printf("Current State: %s\n", client.getState().toString().c_str());
	return 0;
}
