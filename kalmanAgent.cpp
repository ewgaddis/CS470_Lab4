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
	timerMax = 4500;
	shotTimer = timerMax;
	vector<constant_t> constants;
	team->get_constants(&constants);
	for each(constant_t c in constants){
		if (c.name.compare("shotradius") == 0){
			shotRadius = atof(c.value.c_str());
			printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		else if (c.name.compare("shotrange") == 0){
			shotRange = atof(c.value.c_str());
			printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		else if (c.name.compare("shotspeed") == 0){
			shotSpeed = atof(c.value.c_str());
			printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		else if (c.name.compare("tankspeed") == 0){
			tankSpeed = atof(c.value.c_str());
			printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		//printf("1name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		//printf(c.name.c_str());
	}

	double newX = otherTanks[curTarget].pos[0] + cos(otherTanks[curTarget].angle) * tankSpeed * shotTimer;
	double newY = otherTanks[curTarget].pos[1] + sin(otherTanks[curTarget].angle) * tankSpeed * shotTimer;
	printf("\n%d", otherTanks[curTarget].angle);
	curGoal = Vector(newX, newY);
}

void KalmanAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	Vector aForce;
	//myTeam->shoot(botIndex);
	//otherTanks[curTarget];
	//so, I can get the enemies angle, but not how fast they are going. . .
	//takes about 10 secons sometimes for a turn
	//deltaX/(||vp||+||v||) = time (Difference of positions?)/(magnitude of vector of direction he's going in?)+posmagnitude of 
	//future pos = XofTarget + vt + (1 / 2)at ^ 2 (position of target + velocity(time)+1/2(accel*time^2)
	//so the shot should have moved by 350*100 = 3500 = 3.5 secs?+11000


	//Vector goal = Vector(newX, newY);//otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
	Vector myPos = Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]);

	newDirection += Vector(curGoal.x - myPos.x, curGoal.y - myPos.y);//Vector(goal.x - myPos.x, goal.y - myPos.y);
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

	double dot = curVector->dot(newDirection); //acos(curvector.newdirection/(||curvector||*||newDir||))
	dot /= curVector->length()*newDirection.length();
	double angle = acos(dot);

	double avgAngVel = angle - oldAngle;
	oldAngle = angle;

	if (angle > 0.0001 || angle < -0.0001)//(angle > 0.0000001 || angle < -0.0000001)
	{
		double newVel = (angle / (M_1_PI / 2.0)) + (100.0*avgAngVel); //(100.0*avgAngVel);
		if (side < 0){
			myTeam->angvel(botIndex, newVel);
		}
		else{
			myTeam->angvel(botIndex, -newVel);
		}
	}
	else{
		//Sleep(shotTimer);
		myTeam->angvel(botIndex, 0);
		while (shotTimer >= 0)
			shotTimer--;
		shotTimer = timerMax;
		myTeam->shoot(botIndex);
		curTarget++;
		if (curTarget >= otherTanks.size()){
			otherTanks.clear();
			myTeam->get_othertanks(&otherTanks);
			curTarget = 0;
			//work on out of range targets later.
		}
		double newX = otherTanks[curTarget].pos[0] + cos(otherTanks[curTarget].angle) * tankSpeed * shotTimer;
		double newY = otherTanks[curTarget].pos[1] + sin(otherTanks[curTarget].angle) * tankSpeed * shotTimer;
		printf("\n%d", otherTanks[curTarget].angle);
		curGoal = Vector(newX, newY);
	}
	shotTimer--;
}