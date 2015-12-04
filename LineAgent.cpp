#include "lineAgent.h"
#include "geometry.h"
#include <math.h>

LineAgent::LineAgent(BZRC* team, int index){
	myTeam = team;
	botIndex = index;
	forward = true;
	time = 0;
}

void LineAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	myTeam->shoot(botIndex);
	
	if (time>2000 && (myTanks[botIndex].pos[0] > 390 || myTanks[botIndex].pos[0] < -390 || myTanks[botIndex].pos[1]>390 || myTanks[botIndex].pos[1]< -390))
	{
		if (forward)
			forward = false;
		else
			forward = true;
		time = 0;
	}
	if (forward)
		myTeam->speed(botIndex, 1);
	else
		myTeam->speed(botIndex, -1);
	time++;
}