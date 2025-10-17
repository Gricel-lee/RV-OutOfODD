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
    agent(double _radius, double maxX, double maxY, double mxV, double cmf_dst, double sns, double rng);
    b2BodyDef getBodyDef();
    void setBody(b2Body* _body);
    b2Body* getBody();
    double getRange();
	  void updateVelPot(int robot, std::vector<b2Body*> robots, std::vector<b2Body*> oil);
    void updateVel();
    double getVelX();
	  double getVelY();
    double getRadius();
    void setVel(double _vel_x, double _vel_y);
    double getVelMag();
    void brownian();

  private:
  	// Agent definitions
    b2BodyDef body_def;
    b2Body* body;
    double radius;
    double sense;
    // double thresh;
    double range;

    // RD values
    double u;
    double prev_u;
    double v;
    double du;
    double tempU;
    double tempV;
    double muhnf_avg;
    double max_u_heard;
    double max_u_heard_now;
    double max_u_heard_now_flat;
    double prev_max_u_heard_now_flat;
    double neigh_max_heard;
    int neighbours;
    double avg_neigh;
    int neighbours_dist;
    int max_neighbours;
    int brownian_count=0;
    double neigh_ratio;

    // POI for RD
    std::vector<std::vector<double> > poi;

    // Potential field values
    double comf_dist;
    int prev_act;
    double prev_grad;
    int time;

    // Velocity rules
    double velX;
    double velY;
    double maxVel;
    double gamma=1.0;
    int choice;

	std::tuple<double, double> rotate(double theta);

};
