#include "box2d/box2d.h"
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <random>
#include <chrono>
#include <armadillo>
#include <tuple>
#include "agent.h"

class NormalAgent: public agent 
{
  public:
    NormalAgent();
    NormalAgent(std::string yaml_file, std::string results_file, double seed) : agent(yaml_file, results_file, seed){};
    void updateVel(agent* neighbour);
  
    bool oncomingUpdate(double other_targ_relative_pos_x, double other_target_relative_pos_y, double dist, double other_radius, int time_collision);
    bool overTakingUpdate(double o_pos_x, double o_pos_y, double dist, double o_radius, double other_vel_mag, int time_collision);
    bool makeWayUpdate(double o_relative_theta, double other_radius);
    bool crossingUpdate(double other_relative_pos_x, double other_relative_pos_y, double dist, double other_radius, int time_collision);

    double newAngleWorldFrame(double other_pos_x, double other_pos_y, double other_radius, double frame_theta);
    // double targAngle();
    // double egoAngle();
};
