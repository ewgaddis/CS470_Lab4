#ifndef KALMAN_AGENT_H
#define KALMAN_AGENT_H

#include "team.h"
#include "potentialFields.h"

class KalmanAgent {
	BZRC* myTeam;
	int botIndex;
	Vector* curVector;
	Vector oldVector;
	double oldAngle;
	vector <otank_t> otherTanks;
	int curTarget;
public: KalmanAgent(BZRC* team, int index);

		void Update();
private:
};
#endif