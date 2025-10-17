#include "box2d/box2d.h"
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


class agent
{
  public:
    agent();
    agent(double maxX, double maxY, double mxV);
    agent(double _radius, double pos_x, double pos_y, double angle, double mxV, double rng, int _agent_type, std::string results_file);
    b2BodyDef getBodyDef();
    void setBody(b2Body* _body);
    b2Body* getBody();
    double getRange();
	  void updateVelPot(int robot, std::vector<b2Body*> robots, std::vector<b2Body*> oil);
    // void updateVel(double c_theta);
    void updateVel(double other_pos_x, double other_pos_y, double other_vel_mag, double dist, double other_radius, double other_theta);
    double getVelX();
	  double getVelY();
    double getRadius();
    double getMaxVel();

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

    void recordStep();

  private:
  	// Agent definitions
    b2BodyDef body_def;
    b2Body* body;
    double radius;
    double sense;
    // double thresh;
    double range;
    int agent_type; // For plotting and analysis purposes
    std::uniform_real_distribution<double> distribution_size;
    std::uniform_real_distribution<double> distribution_x;
    std::uniform_real_distribution<double> distribution_y;

    // Potential field values
    double comf_dist;
    int prev_act;
    double prev_grad;
    int time;


    // Goal variables
    double targ_x;
    double var_x;
    double targ_y;
    double var_y;
    double cmf_dist;

    // Velocity rules
    double velX;
    double velY;
    double new_vel_mag; // Used for updating with consisent values with neighbours
    double vel_mag;
    double vel_max;

    // angular information
    double theta=0.0;
    double theta_acc=0.0;
    double new_theta_acc=0.0;
    double d_theta=0.5;
    std::tuple<double, double> rotate(double theta);

    // Recording
    std::string filename;
    std::ofstream outfile;
};
