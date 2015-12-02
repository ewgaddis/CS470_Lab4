#include "kalmanAgent.h"
#include "geometry.h"
#include <math.h>

KalmanAgent::KalmanAgent(BZRC* team, int index){
	myTeam = team;
	botIndex = index;
	curVector = new Vector();
	oldVector = Vector();
	oldAngle = 0.0;
	myTeam->get_othertanks(&otherTanks);
	curTarget = 0;
}

void KalmanAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	Vector aForce;
	//myTeam->shoot(botIndex);
	//otherTanks[curTarget];

	Vector goal = Vector(otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
	Vector myPos = Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]);

	newDirection += Vector(goal.x - myPos.x, goal.y - myPos.y);
	//end frobbing - f = a/dist + b
	//last param = range that obstacles affect bot.

	newDirection += aForce;
	/*if (newDirection.x > 20.0)
		newDirection.x = 20.0;
	if (newDirection.y > 20.0)
		newDirection.y = 20.0;*/

	Vector avgVel = newDirection - oldVector;
	oldVector = newDirection;


	Vector sideVector;
	newDirection.perpendicular(&sideVector);
	double side = curVector->dot(sideVector);
	//finding the angle
	curVector->x = cos(myTanks[botIndex].angle);
	curVector->y = sin(myTanks[botIndex].angle);

	double dot = curVector->dot(newDirection);
	dot /= curVector->length()*newDirection.length();
	double angle = acos(dot);

	double avgAngVel = angle - oldAngle;
	oldAngle = angle;

	if (angle > 0.0000001 || angle < -0.0000001)
	{
		double newVel = (angle / (M_1_PI / 2.0)) + (100.0*avgAngVel);
		if (side < 0){
			myTeam->angvel(botIndex, newVel);
		}
		else{
			myTeam->angvel(botIndex, -newVel);
		}
	}
	else{
		myTeam->shoot(botIndex);
		curTarget++;
		if (curTarget > otherTanks.size()){
			otherTanks.clear();
			myTeam->get_othertanks(&otherTanks);
			curTarget = 0;
		}
	}
}