#include "team.h"

#include "gnuplotter.h"
#include "potentialFields.h"
#include "graph.h"
#include "graphAlgorithms.h"
#include "dumbAgent.h"
#include "pdAgent.h"
#include "searchAgent.h"
//#include "gridWindow.h"
//#include "occGrid.h"
#include "scoutAgent.h"
#include "lineAgent.h"
#include "crazyAgent.h"
#include "geometry.h"
#include "kalmanAgent.h"
#include "kalmanFilter.h"

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

BZRC *team;
//OCCGrid *grid;
LineAgent *line1;
CrazyAgent *crazy1;
KalmanAgent *kalman1;

//double trueNeg;
//double truePos;

void world_init(BZRC *my_team)
{
	team = my_team;
	//line1 = new LineAgent(team, 0);
	//crazy1 = new CrazyAgent(team, 0);
	kalman1 = new KalmanAgent(team, 0);
	/*vector<constant_t> constants;
	team->get_constants(&constants);
	for each(constant_t c in constants){
		/*if (c.name.compare("truenegative") == 0){
		trueNeg = atof(c.value.c_str());
		//printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		else if (c.name.compare("truepositive") == 0){
		truePos = atof(c.value.c_str());
		//printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		printf("\n1name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		printf(c.name.c_str());
	}
	/*scout1 = new ScoutAgent(team, 0, "upper");
	scout2 = new ScoutAgent(team, 1, "lower");
	scout3 = new ScoutAgent(team, 2, "lower");
	scout4 = new ScoutAgent(team, 3, "lower");
	scout5 = new ScoutAgent(team, 4, "upper");

	grid = new OCCGrid(team, 800, 0.5, truePos, trueNeg);
	*/
}

void robot_pre_update()
{
}

bool robot_update()
{
	//line1->Update();
	//crazy1->Update();
	kalman1->Update();
	/*grid->update(0);
	grid->update(1);
	grid->update(2);
	grid->update(3);
	grid->update(4);

	vector<obstacle_t> obstacles;
	grid->getObstacles(&obstacles, 0.9, 20, 20);

	updateGridWindow(grid->getGridSize(),
					 grid->getGrid(),
					 &obstacles);
	*/
	Vector v(2.4, -1.9);

	Vector2d v2;
	v.getEVector(&v2);

	//cout << v2 << endl;

	v.setEVector(v2);

	/*scout1->Update(obstacles, grid);
	scout2->Update(obstacles, grid);
	scout3->Update(obstacles, grid);
	scout4->Update(obstacles, grid);
	scout5->Update(obstacles, grid);*/
	//cout << v << endl;

	/*KalmanFilter filter(100.0, 0.1, 0.1,
						0.1, 0.1, 100.0,
						25.0);

	Vector pos(40.0, 60.0);

	VectorXd z(2);
	z.fill(0.0);
	z(0) = pos.x;
	z(1) = pos.y;

	GNUPlotter plotter;

	plotter.createFile("./Data/test1.gpi", "Gaussian");
	plotter.drawCircle(pos, 10, 255, 0, 0);
	plotter.drawGaussian(filter.getMean(), filter.getSigma());
	plotter.finishFile();

	cout << "Updating filter..." << endl;

	for(int i = 0; i < 5; ++i)
	{
		filter.update(z, 0.25, 0.0);

		cout << "mean:\n"  << filter.getMean()  << endl;
		cout << "sigma:\n" << filter.getSigma() << endl;
	}

	cout << "Finished updating filter" << endl;

	plotter.createFile("./Data/test2.gpi", "Gaussian");
	plotter.drawCircle(pos, 10, 255, 0, 0);
	plotter.drawGaussian(filter.getMean(), filter.getSigma());
	plotter.finishFile();*/

	//return false;
	return true;
}

void robot_post_update()
{
}