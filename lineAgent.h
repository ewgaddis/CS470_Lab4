#ifndef LINE_AGENT_H
#define LINE_AGENT_H

#include "team.h"
#include "potentialFields.h"

class LineAgent {
	BZRC* myTeam;
	int botIndex;
	bool forward;
	int time;
public: LineAgent(int index);

		void setTeam(BZRC *team) { myTeam = team; }
		void Update();
private:
};
#endif