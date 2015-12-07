#ifndef KALMAN_AGENT_H
#define KALMAN_AGENT_H

#include "team.h"
#include "potentialFields.h"
#include "kalmanFilter.h"

class KalmanAgent {
	BZRC* myTeam;
	int botIndex;
	Vector* curVector;
	Vector oldVector;
	double oldAngle;
	vector <otank_t> otherTanks;
	int curTarget;
	double shotRadius;
	double shotRange;
	double shotSpeed;
	double tankSpeed;//maybe not needed?
	int shotTimer;
	int timerMax;
	Vector curGoal;
	//KalmanFilter * filter;
public: KalmanAgent(BZRC* team, int index);

		void Update();
private:
};
#endif