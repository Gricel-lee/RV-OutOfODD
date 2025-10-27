#ifndef SIM_UTILS
#define SIM_UTILS
#include <box2d>
#include <normal_agent.h>
#include <mass_agent.h>

void simDesign(int TT, vector<NormalAgent*> colregs, vector<MassAgent*> MASS, vector<vector<double>> goal_locations, b2WorldId worldID);
void simDeploy(int TT, vector<NormalAgent*> colregs, vector<MassAgent*> MASS, vector<vector<double>> goal_locations, b2WorldId worldID);
void setGoal(agent* ego, vector<vector<double>> goals, int selection)
void checkNeigh(NormalAgent* ego, agent* neigh)
void checkNeighMass(MassAgent* ego, agent* neigh)
bool updateMassSituation(MassAgent* ego, agent* neigh)
void resetSim(agent* ego, b2WorldId worldID, vector<vector<double>> goal_locations)

#endif