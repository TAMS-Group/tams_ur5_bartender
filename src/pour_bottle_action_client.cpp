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
		printf("Noo! Something failed with pouring bottle 2!");
	printf("Current State: %s\n", client.getState().toString().c_str());
	
       
	


	
	
	return 0;


}
