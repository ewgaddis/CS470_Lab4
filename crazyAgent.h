#ifndef CRAZY_AGENT_H
#define CRAZY_AGENT_H

#include "team.h"
#include "potentialFields.h"

class CrazyAgent {
	BZRC* myTeam;
	int botIndex;
	bool forward;
	int time;
	int maxTime;
	double curSpeed;
	//Vector* curVector;
	//Vector oldVector;
	double oldAngle;
	double curAngVel;
public: CrazyAgent(BZRC* team, int index);

		void Update();
private:
};
#endif