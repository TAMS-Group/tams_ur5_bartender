#include <project16_manipulation/PourBottleAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<project16_manipulation::PourBottleAction> Client;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pour_bottle_client");
	Client client("pour_bottle", true); // true -> don't need ros::spin()
	client.waitForServer();
	
	project16_manipulation::PourBottleGoal goal;
        project16_manipulation::PourBottleGoal goal2;
		
	// Fill in goal 1 here
	goal.bottle_id = "bacardi";
	goal.portion_size = 10.0;
		
	// Fill in goal 1 here
	goal2.bottle_id = "tequila";
	goal2.portion_size = 10.0;
	
	// Send goal 1
	client.sendGoal(goal);
	client.waitForResult();
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && client.getResult()->success)
		printf("Yay! The Bottle 1 has been poured!");
    	else
		printf("Noo! Something failed with pouring bottle 1!");
	printf("Current State: %s\n", client.getState().toString().c_str());
	
       
	
	// Send goal 2
	client.sendGoal(goal2);
	client.waitForResult();
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && client.getResult()->success)
		printf("Yay! The Bottle 2 has been poured!");
    	else
		printf("Noo! Something failed with pouring bottle 1!");
	printf("Current State: %s\n", client.getState().toString().c_str());


	
	
	return 0;


}
