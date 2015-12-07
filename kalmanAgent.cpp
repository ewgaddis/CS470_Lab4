#include "kalmanAgent.h"
#include "geometry.h"
#include "gnuplotter.h"
//#include "kalmanFilter.h"

#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;
KalmanAgent::KalmanAgent(BZRC* team, int index){
	myTeam = team;
	botIndex = index;
	curVector = new Vector();
	oldVector = Vector();
	oldAngle = 0.0;
	myTeam->get_othertanks(&otherTanks);
	curTarget = 0;
	timerMax = 6500;
	shotTimer = timerMax;
	vector<constant_t> constants;
	team->get_constants(&constants);
	//filter = new KalmanFilter(800.0, 0.01, 0.01, 0.1, 0.01, 0.01, 25.0);
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
	KalmanFilter filter(800.0, 0.01, 0.01,
		0.1, 0.01, 0.01,
		25.0);

	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	//filter made->now hone in.
	Vector pos(otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
	Vector myPos(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]);

	/*VectorXd z(2);
	z.fill(0.0);
	z(0) = pos.x;
	z(1) = pos.y;*/

	for (int i = 0; i < 20; ++i)
	{
		myPos.x = myTanks[botIndex].pos[0];
		myPos.y = myTanks[botIndex].pos[1];

		VectorXd z(2);
		z.fill(0.0);
		z(0) = pos.x;
		z(1) = pos.y;


		filter.update(z, 0.05, 0.0);

		//cout << "mean:\n" << filter.getMean() << endl;
		//cout << "sigma:\n" << filter.getSigma() << endl;
	}

	//time to make a prediction:
	VectorXd newGoal = VectorXd();
	double time = vectorDistance(myPos, pos) / (shotSpeed + (Vector(filter.getMean()(1),filter.getMean()(4)).length()));
	filter.predict(time+6.5,0,&newGoal,false,false);
	curGoal = Vector(newGoal(0), newGoal(3));
	printf("\n %f %f", curGoal.x, curGoal.y);
}

void KalmanAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	myTeam->get_mytanks(&myTanks);
	Vector aForce;
	//myTeam->shoot(botIndex);
	//takes about 10 secons sometimes for a turn

	//Vector goal = Vector(newX, newY);//otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
	Vector myPos = Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]);

	newDirection += Vector(curGoal.x - myPos.x, curGoal.y - myPos.y);//Vector(goal.x - myPos.x, goal.y - myPos.y);
	//end frobbing - f = a/dist + b
	//last param = range that obstacles affect bot.

	//newDirection += aForce;
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

	Vector oldPos, agentVel, agentAvgVel;

	if (angle > 0.0001 || angle < -0.0001)//(angle > 0.0000001 || angle < -0.0000001)
	{
		double newVel = (angle / (M_1_PI / 1.0)) + (100.0*avgAngVel); //(100.0*avgAngVel);
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
		//while (shotTimer >= 0)
		//	shotTimer--;
		//shotTimer = timerMax;
		myTeam->shoot(botIndex);
		curTarget++;
		bool validTarget = false;
		while (!validTarget){
			if (curTarget >= (int)otherTanks.size()){
				otherTanks.clear();
				myTeam->get_othertanks(&otherTanks);
				curTarget = 0;
			}
			Vector pos(otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
			if (otherTanks[curTarget].color.compare("blue") != 0 || pos.x > 400 || pos.y > 400 || pos.x < -400 || pos.y < -400)
				curTarget++;
			else
				validTarget = true;
		}
		/*KalmanFilter filter(800.0, 0.01, 0.01,
			0.1, 0.01, 0.01,
			25.0);*/
		KalmanFilter filter(1600.0, 800.0, 0.01,
			0.01, 0.01, 0.01,
			100.0);
		//filter made->now hone in.
		Vector pos(otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
		printf("\nPos: %f %f tank: %s", pos.x, pos.y, otherTanks[curTarget].color.c_str());

		/*VectorXd z(2);
		z.fill(0.0);
		z(0) = pos.x;
		z(1) = pos.y;*/

		agentAvgVel = Vector();

		for (int i = 0; i < 30; ++i)
		{
			oldPos = pos;
			pos.x = otherTanks[curTarget].pos[0];
			pos.y = otherTanks[curTarget].pos[1];

			agentAvgVel += (pos - oldPos) * (10.0 / 30.0);

			printf("\nPos: %f %f tank: %s", pos.x, pos.y, otherTanks[curTarget].color.c_str());
			VectorXd z(2);
			z.fill(0.0);
			z(0) = pos.x;
			z(1) = pos.y;

			filter.update(z, 0.1, 0.0);
			//curTarget++;
			otherTanks.clear();
			myTeam->get_othertanks(&otherTanks);
			curTarget = 0;
			bool validTarget = false;
			while (!validTarget){
				if (curTarget >= (int)otherTanks.size()){
					otherTanks.clear();
					myTeam->get_othertanks(&otherTanks);
					curTarget = 0;
				}
				Vector pos(otherTanks[curTarget].pos[0], otherTanks[curTarget].pos[1]);
				if (otherTanks[curTarget].color.compare("blue") != 0 || pos.x > 400 || pos.y > 400 || pos.x < -400 || pos.y < -400)
					curTarget++;
				else
					validTarget = true;
			}
			Sleep(100);
		}

		agentVel = agentAvgVel;

		static int kalmanNum = 0;

		//time to make a prediction:
		//curGoal = Vector(newX, newY);
		VectorXd newGoal = VectorXd();
		double time = 0.3*(vectorDistance(myPos, pos) / (shotSpeed + Vector(filter.getMean()(1), filter.getMean()(4)).length()));
		printf("\ntime: %f", time);
		filter.predict(time+6.0, 0, &newGoal,false,false);
		curGoal = Vector(newGoal(0), newGoal(3));
		printf("\nNew: %f %f", curGoal.x, curGoal.y);
		cout << "\nMean:\n" << filter.getMean() << endl;
		cout << "velocity: " << agentVel << endl;
		cout << "kalman #" << kalmanNum << endl;
		//Vector dir = Vector(1, 1);
		plotKalmanFilter(filter, time + 6.0, 0, &pos, &agentVel);
		myTeam->shoot(botIndex);

		kalmanNum++;
		
	}
	//shotTimer--;
}