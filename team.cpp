#include "team.h"

#include "gnuplotter.h"
#include "kalmanFilter.h"
#include "lineAgent.h"
#include "crazyAgent.h"
#include "kalmanAgent.h"

#include <Eigen/Dense>
#include <iostream>
#include <time.h>
#include <conio.h>

using namespace std;
using namespace Eigen;

BZRC *team;
LineAgent lineAgent(0);
CrazyAgent *crazyAgent;

KalmanAgent *kalmanAgent;

double getRandom(int min, int max)
{
	return (double)(rand() % ((max - min) * 100) + min * 100) / 100.0;
}

void world_init(BZRC *my_team)
{
	team = my_team;

	srand((unsigned int)time(0));

	lineAgent.setTeam(team);
	//crazyAgent = new CrazyAgent(team, 0);

	kalmanAgent = new KalmanAgent(team, 0);
}

void robot_pre_update()
{
}

bool robot_update()
{
	//crazyAgent->Update();
	//lineAgent.Update();
	kalmanAgent->Update();
	/*KalmanFilter filter(1600.0, 800.0, 400.0,
						0.01, 0.01, 25.0,
						100.0);

	Vector pos(300.0, 375.0);
	Vector vel(-10.0, -20.0);
	Vector accel(0.0, 0.0);

	double elapsedTime = 1.0;
	double c = 0.0;

	int iterations = 40;

	for(int i = 0; i < iterations; ++i)
	{
		pos += vel * elapsedTime + accel * (elapsedTime * elapsedTime / 2.0);
		vel += accel * elapsedTime;

		VectorXd z(2);
		z.fill(0.0);
		z(0) = pos.x + getRandom(-5, 5);
		z(1) = pos.y + getRandom(-5, 5);

		filter.update(z, elapsedTime, c);

		plotKalmanFilter(filter, 1.0, c, &pos, &vel);
	}*/

	if(_kbhit())
	{
		//delete crazyAgent;
		delete kalmanAgent;
		return false;
	}

	return true;
}

void robot_post_update()
{
}