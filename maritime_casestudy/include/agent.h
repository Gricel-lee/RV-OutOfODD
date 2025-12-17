#ifndef AGENT_H
#define AGENT_H

#include <box2d/box2d.h>
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <regex.h>
#include <fstream>
#include <random>
#include <chrono>
// #include <armadillo>
#include <tuple>

using namespace std;

class agent
{
  public:
    agent();
    agent(std::string yaml_file, std::string results_file, double seed);
    void initialiseAgent();

    b2BodyDef* getBodyDef();
    // void setBody(b2Body* _body);
    void setBodyID(b2BodyId _bodyID);
    // b2Body* getBody();
    b2BodyId getBodyID();
    double getRange();

    float getVelX();
	  float getVelY();
    double getRadius();
    double getMaxVel();

    void setBodyDefPose(float _x_pos, float _y_pos, double _theta);
    void setBodyPose(float _x_pos, float _y_pos, float _theta);

    double getVelMag();
    void brownian();

    void setTheta(double _theta);
    double getTheta();

    void setThetaAcc(double _theta_acc);
    double getThetaAcc();

    void updateTheta();
    double goalTheta();//double x_pos, double y_pos, double x_targ, double y_targ);
    void setTarget(double _targ_x, double _var_x, double _targ_y, double _var_y);
    void setVel(double _vel_x, double _vel_y);
    void setVelMag(double _vel_mag);
    void updateVelMag();

    double getRelativeGoal(double pos, double targ_pos, double var_pos);
    bool checkGoal();
    
    agent copy();

    void recordStep(int t);

    double targAngle();
    double egoAngle();

    void agentNeighReset();

    int getAgentType();

    string parseYAMLENV(string yaml_line);

    int checkFuture(int lookahead_time, double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius);

    // Neighbour information
    int no_neigh=0;
    double sum_neigh_dist=0.0;
    // Goal variables
    double targ_x;
    double var_x;
    double targ_y;
    double var_y;

  protected:
    // To initialise agent
    std::string yaml_file;
    double seed;

  	// Agent definitions
    b2BodyDef body_def;
    // b2Body* body;
    b2BodyId bodyID;
    double radius;
    double sense;
    // double thresh;
    double range;
    int agent_type; // For visual classification purposes (and plotting/analysis)

    // Potential field values
    // double comf_dist;
    int prev_act;
    double prev_grad;
    int time;
    double min_dist;

    std::vector<std::vector<double>> forbidden_zones;

    double cmf_dist;

    // Velocity rules
    double velX;
    double velY;
    double new_vel_mag; // Used for updating with consisent values with neighbours
    double vel_mag;
    double vel_max;
    double vel_theta_max;

    // angular information
    double theta=0.0;
    double theta_acc=0.0;
    double new_theta_acc=0.0;
    double d_theta=0.5;
    std::tuple<double, double> rotate(double theta);


    // Recording
    std::string results_dir;
    std::string filename;
    std::ofstream outfile;
};

#endif