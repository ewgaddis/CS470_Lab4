#include "crazyAgent.h"
#include "geometry.h"
#include <math.h>

CrazyAgent::CrazyAgent(BZRC* team, int index){
	myTeam = team;
	botIndex = index;
	forward = true;
	time = 0;
	maxTime = 1000;
	curSpeed = 0;
	//Vector* curVector;
	//Vector oldVector;
	oldAngle = 0;
	curAngVel = 0;
}

void CrazyAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	myTeam->shoot(botIndex);

	if (time>maxTime)
	{
		maxTime = rand() % 1000 + 500;
		time = 0;
		curSpeed = (double)(rand() % 201 - 100) / 100.0;
		curAngVel = (double)(rand() % 201 - 100) / 100.0;
		printf("\nSpeed %f AngVel %f", curSpeed, curAngVel);
		if (curSpeed > 0)
			forward = true;
		else
			forward = false;
	}
	myTeam->speed(botIndex, curSpeed);
	myTeam->angvel(botIndex, curAngVel);
	if (forward)
		curSpeed -= (rand() % 10 + 1)/10;
	else
		curSpeed += (rand() % 10 + 1) / 10;
	time++;
}