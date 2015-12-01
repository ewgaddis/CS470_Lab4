#include "team.h"

#include "geometry.h"

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

BZRC *team;

void world_init(BZRC *my_team)
{
	team = my_team;
}

void robot_pre_update()
{
}

bool robot_update()
{
	Vector v(2.4, -1.9);

	Vector2d v2;
	v.getEVector(&v2);

	cout << v2 << endl;

	v.setEVector(v2);

	cout << v << endl;

	return false;
}

void robot_post_update()
{
}