#include "gnuplotter.h"

#include <iostream>
#include <deque>

using namespace Eigen;
using namespace std;

GNUPlotter::GNUPlotter() : file(0)
{
}

void GNUPlotter::createFile(const char *fileName, const char *title)
{
	file = fopen(fileName, "w");

	if(!file)
	{
		cout << "Failed to create file \"" << fileName << "\" for gnuplot" << endl;
		return;
	}

	fprintf(file, "set title \"%s\"\n", title);
	fprintf(file, "set xrange [-400.0: 400.0]\n");
	fprintf(file, "set yrange [-400.0: 400.0]\n");
	fprintf(file, "set pm3d\n");
	fprintf(file, "set view map\n");
	fprintf(file, "unset key\n");
	fprintf(file, "set size square\n");
	fprintf(file, "unset arrow\n");
}

void GNUPlotter::drawArrow(double x, double y, const Vector &v, int color)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set arrow from %f, %f to %f, %f size 25, 20 front lt %i\n",
			x - v.x / 2.0, y - v.y / 2.0,
			x + v.x / 2.0, y + v.y / 2.0,
			color);
}

void GNUPlotter::drawArrow(const Vector &t, const Vector &h, int color)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set arrow from %f, %f to %f, %f size 25, 20 front lt %i\n",
			t.x, t.y, h.x, h.y, color);
}

void GNUPlotter::drawArrow(const Vector &t, const Vector &h,
						   unsigned char r, unsigned char g, unsigned char b)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set arrow from %f, %f to %f, %f size 25, 20 front lt rgb \"#%02X%02X%02X\"\n",
			t.x, t.y, h.x, h.y, r, g, b);
}

void GNUPlotter::drawLine(double x1, double y1, double x2, double y2, int color)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set arrow from %f, %f to %f, %f nohead front lt %i\n",
			x1, y1, x2, y2, color);
}

void GNUPlotter::drawCircle(const Vector &c, int radius,
							unsigned char r, unsigned char g, unsigned char b)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set object circle at %f,%f size %i front fillcolor rgb \"#%02X%02X%02X\" fillstyle solid\n",
			c.x, c.y, radius, r, g, b);
}

void GNUPlotter::drawText(double x, double y, const char *text)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set label \"%s\" at %f,%f\n", text, x, y);
}

void GNUPlotter::drawObstacles(const vector<obstacle_t> &obstacles)
{
	if(!file)
	{
		return;
	}

	vector<obstacle_t>::const_iterator itObstacle = obstacles.begin();
	while(itObstacle != obstacles.end())
	{
		const obstacle_t *o = &(*itObstacle);

		for(int c = 0; c < o->numCorners; ++c)
		{
			int e0 = c;
			int e1 = (c + 1 == o->numCorners ? 0 : c + 1);

			drawLine(o->o_corner[e0][0], o->o_corner[e0][1],
					 o->o_corner[e1][0], o->o_corner[e1][1],
					 3);
		}

		++itObstacle;
	}
}

void GNUPlotter::drawGraph(const Graph &graph)
{
	if(!file)
	{
		return;
	}

	for(int i = 0; i < graph.getNumberNodes(); ++i)
	{
		for(int j = i; j < graph.getNumberNodes(); ++j)
		{
			if(graph.edgeExist(i, j))
			{
				const Vector & pos1 = graph.getNodePos(i);
				const Vector & pos2 = graph.getNodePos(j);

				drawLine(pos1.x, pos1.y, pos2.x, pos2.y, -1);
			}
		}
	}
}

void GNUPlotter::drawGraphSearch(const Graph &graph,
								 const GraphSearch *graphSearch)
{
	if(!file)
	{
		return;
	}

	for(int i = 0; i < graph.getNumberNodes(); ++i)
	{
		int parent = graphSearch->getParent(i);

		if(parent != -1)
		{
			const Vector & pos1 = graph.getNodePos(i);
			const Vector & pos2 = graph.getNodePos(parent);

			drawLine(pos1.x, pos1.y, pos2.x, pos2.y, 5);
		}
	}

	deque<int> path;
	graphSearch->getPath(&path);

	if(path.size() > 1)
	{
		double totalCost = 0.0;

		deque<int>::iterator itNode = path.begin() + 1;
		while(itNode != path.end())
		{
			Vector pos1 = graph.getNodePos(*(itNode - 1));
			Vector pos2 = graph.getNodePos(*(itNode    ));

			drawArrow(pos1, pos2, 1);

			totalCost += vectorDistance(pos1, pos2);

			++itNode;
		}

		char cost[64];
		sprintf(cost, "Cost: %.3f", totalCost);
		drawText(-400, -320, cost);
	}

	const bool *visited = graphSearch->getVisited();

	for(int i = 0; i < graph.getNumberNodes(); ++i)
	{
		if(visited[i])
		{
			drawCircle(graph.getNodePos(i), 10,
					   255, 0, 0);
		}
	}

	vector<int> frontier;
	graphSearch->getFrontier(&frontier);

	vector<int>::iterator itFrontNode = frontier.begin();
	while(itFrontNode != frontier.end())
	{
		drawCircle(graph.getNodePos(*itFrontNode), 12,
				   0, 255, 0);

		++itFrontNode;
	}
}

void GNUPlotter::drawGaussian(const VectorXd &mean,
							  const MatrixXd &sigma)
{
	if(!file)
	{
		return;
	}

	fprintf(file, "set palette model RGB functions 1-gray, 1-gray, 1-gray\n");
	fprintf(file, "set isosamples 100\n");

	fprintf(file, "mx = %f\n", mean(0));
	fprintf(file, "my = %f\n", mean(3));

	fprintf(file, "sigma_x = %f\n", sigma(0, 0));
	fprintf(file, "sigma_y = %f\n", sigma(3, 3));
	fprintf(file, "rho = %f\n", sigma(3, 0));

	fprintf(file, "splot 1.0 / (2.0 * pi * sigma_x * sigma_y * sqrt(1.0 - rho ** 2)) \\\n");
	fprintf(file, "* exp(-1.0 / (2.0 * (1.0 - rho ** 2)) * ((x - mx) ** 2 / sigma_x ** 2 + (y - my) ** 2 / sigma_y ** 2 \\\n");
	fprintf(file, "- 2.0 * rho * (x - mx) * (y - my) / (sigma_x * sigma_y))) with pm3d\n");
}

void GNUPlotter::finishFile()
{
	if(file)
	{
		fclose(file);
		file = 0;
	}
}