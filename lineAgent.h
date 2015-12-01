#ifndef LINE_AGENT_H
#define LINE_AGENT_H

#include "team.h"
#include "potentialFields.h"

class LineAgent {
	BZRC* myTeam;
	int botIndex;
	bool forward;
	int time;
public: LineAgent(BZRC* team, int index);

		void Update();
private:
};
#endif