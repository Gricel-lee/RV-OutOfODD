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
#include <regex>
#include "agent.h"

#define PI 3.14159265

using namespace std;

agent::agent()
{
  // EMPTY CONSTRUCTOR
}

agent::agent(std::string _yaml_file, std::string results_file, double _seed)
{
  yaml_file=_yaml_file;
  seed=_seed;
  initialiseAgent();
 // Reset/create file
  filename=results_dir+"/"+results_file;
  outfile.open(filename);
  outfile.close();
}

void agent::initialiseAgent()
{
 // Read from yaml file
  YAML::Node config = YAML::LoadFile(yaml_file);
 // Initialise agent's position
  const double x_min=config["initial pose"]["x"]["min"].as<double>();
  const double x_max=config["initial pose"]["x"]["max"].as<double>();
  const double y_min=config["initial pose"]["y"]["min"].as<double>();
  const double y_max=config["initial pose"]["y"]["max"].as<double>();
  const double theta_min=config["initial pose"]["theta"]["min"].as<double>();
  const double theta_max=config["initial pose"]["theta"]["max"].as<double>();
  std::uniform_real_distribution<double> distribution_x(x_min, x_max);
  std::uniform_real_distribution<double> distribution_y(y_min, y_max);
  std::uniform_real_distribution<double> distribution_theta(theta_min, theta_max);

  std::default_random_engine gen;
  gen.seed(seed);
  float pos_x=distribution_x(gen);
  float pos_y=distribution_y(gen);
  double _angle=distribution_theta(gen);

 // Box2D particle parameters
  body_def=b2DefaultBodyDef();
  body_def.type=b2_dynamicBody;
  body_def.position=(b2Vec2){pos_x, pos_y};
  // body_def.angle=_angle;
  body_def.rotation=b2MakeRot(_angle);

 // Initialise agent's size, vel, and range
  double size_min=config["size"]["min"].as<double>();
  double size_max=config["size"]["max"].as<double>();
  std::uniform_real_distribution<double> distribution_size(size_min, size_max);
  radius=distribution_size(gen);

 // Robot sensor parameters
  range=radius*config["range-size ratio"].as<double>();
  vel_max=radius*config["vel-size ratio"]["linear"].as<double>();
  new_vel_mag=vel_max;
  vel_mag=vel_max;
  vel_theta_max=config["vel-size ratio"]["angular"].as<double>()/radius;
  agent_type=config["agent type"].as<int>();

 // Set target
  targ_x=config["goal pose"]["x"]["target"].as<double>();
  var_x=config["goal pose"]["x"]["tolerance"].as<double>();
  targ_y=config["goal pose"]["y"]["target"].as<double>();
  var_y=config["goal pose"]["y"]["tolerance"].as<double>();

  cmf_dist=0.5*radius;

 // Recording output
  results_dir=config["results directory"].as<std::string>();
  results_dir=parseYAMLENV(results_dir);
}


string agent::parseYAMLENV(string yaml_line)
{
  auto const env_regex=regex("\\$\\{.*\\}");
  smatch env_match;
  bool match=regex_search(yaml_line, env_match, env_regex);
  if (!match) return yaml_line;
  string env_var=env_match[0];
  env_var=env_var.substr(2, strlen(env_var.data())-3);
  string env_val=getenv(env_var.data());
  string parsed_line;
  std::regex_replace(std::back_inserter(parsed_line), yaml_line.begin(), yaml_line.end(), env_regex, env_val);
  return parsed_line;
}

double agent::targAngle()
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  return atan2(targ_y-position.y, targ_x-position.x);
}

double agent::egoAngle()
{
  b2Rot rot=b2Body_GetRotation(getBodyID());
  return b2Rot_GetAngle(rot);
}


b2BodyDef* agent::getBodyDef()
{
  return &body_def;
}

// void agent::setBody(b2Body* _body)
void agent::setBodyID(b2BodyId _bodyID)
{
  bodyID=_bodyID;
}

void agent::setBodyDefPose(float _x_pos, float _y_pos, double _theta)
{ 
  body_def.position=(b2Vec2){_x_pos, _y_pos};
  body_def.rotation=b2MakeRot(_theta);
}

void agent::setBodyPose(float _x_pos, float _y_pos, float _theta)
{
  b2Vec2 position=(b2Vec2){_x_pos, _y_pos};//b2Body_GetPosition(getBodyID());
  // b2Rot rotation=b2Body_GetRotation(getBodyID());
  b2Rot heading=b2MakeRot(_theta);
  printf("\t\t-Setting transform:  %f  %f  %f\n", _x_pos, _y_pos, _theta);
  getBodyID();
  printf("\t\t-Tested bodyID\n");
  b2Body_SetTransform(bodyID,position,heading);
  printf("\t\t-Transform set!\n");
}

// b2Body* agent::getBody()
b2BodyId agent::getBodyID()
{
  return bodyID;
}

float agent::getVelX()
{
  // return velX;
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  theta=b2Rot_GetAngle(rotation);
  return cos(theta)*vel_mag;
}

float agent::getVelY()
{
  // return velY;
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  theta=b2Rot_GetAngle(rotation);
  return sin(theta)*vel_mag;
}

double agent::getMaxVel()
{
  return vel_max;
}

void agent::updateTheta()
{
  double gamma=0.9;
  if (new_theta_acc>0.0 || new_theta_acc<0.0)
  { 
   // Update theta_acc command
    theta_acc=new_theta_acc;//gamma*(new_theta_acc)+(1-gamma)*goalTheta();
    if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
    else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  }
  else
  {
    theta_acc=goalTheta();
    if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
    else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  }
  new_theta_acc=0;
}

double agent::getRelativeGoal(double pos, double targ_pos, double var_pos)
{
  double gamma=0.2; // Shift towards centre of lane
  double new_targ_pos;
  if (pos<(targ_pos-var_pos))
  {
    new_targ_pos=targ_pos-var_pos;
  }
  else if (pos>targ_pos+var_pos)
  {
    new_targ_pos=targ_pos+var_pos;
  }
  else
  {
    new_targ_pos=(1-gamma)*pos+gamma*(targ_pos);
  }
  return new_targ_pos;
}

double agent::goalTheta()//double x_pos, double y_pos, double x_targ, double y_targ)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation);

  double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);

  double theta_diff=atan2(relative_targ_y-pos_y, relative_targ_x-pos_x);
  double goal_theta=theta_diff-heading;
  if (goal_theta<-PI) goal_theta+=2*PI;
  else if (goal_theta>PI) goal_theta-=2*PI;

  // double vel_x=getVelX();
  // double vel_y=getVelY();

  // double dot=vel_x*relative_targ_x + vel_y*relative_targ_y;
  // double det=vel_x*relative_targ_y - relative_targ_x*vel_y;
  // double goal_theta=atan2(det, dot);
  // printf("Goal theta:\n\tposition: %f  %f\n\tgoal position: %f  %f\n\tgoal heading: %f\n", pos_x, pos_y, relative_targ_x, relative_targ_y, goal_theta);
  return goal_theta;
}

void agent::setTarget(double _targ_x, double _var_x, double _targ_y, double _var_y)
{
  targ_x=_targ_x;
  var_x=_var_x;

  targ_y=_targ_y;
  var_y=_var_y;
}

bool agent::checkGoal()
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  if (pos_x<targ_x+var_x && pos_x>targ_x-var_x && pos_y<targ_y+var_y && pos_y>targ_y-var_y) return true;
  else return false;
} 

void agent::setTheta(double _theta)
{
  theta=_theta;
}

void agent::setThetaAcc(double _theta_acc)
{
  theta_acc=_theta_acc;
}

double agent::getThetaAcc()
{
  return theta_acc;
}

double agent::getTheta()
{
  return theta;
}

double agent::getVelMag()
{
  return vel_mag;
}

void agent::updateVelMag()
{
  if (new_vel_mag>vel_max) vel_mag=vel_max;
  else if (new_vel_mag<0) vel_mag=0;
  else vel_mag=new_vel_mag;
  new_vel_mag=vel_max;
  min_dist=2*range;
  // printf("New vel mag:  %f / %f\n", vel_mag, vel_max);
}

void agent::setVelMag(double _vel_mag)
{
  vel_mag=_vel_mag;
  // updateVel();
}

void agent::setVel(double _vel_x, double _vel_y)
{
  theta=atan2(_vel_y, _vel_x);
  // updateVel(theta);
}

double agent::getRadius()
{
  return radius;
}

double agent::getRange()
{
  return range;
}

void agent::agentNeighReset()
{
  no_neigh=0;
  sum_neigh_dist=range;
}

int agent::getAgentType()
{
  return agent_type;
}

int agent::checkFuture(int lookahead_time, double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double vel_x=getVelX();
  double vel_y=getVelY();

  double neigh_vel_x=cos(neigh_theta)*neigh_vel_mag;
  double neigh_vel_y=sin(neigh_theta)*neigh_vel_mag;
  double dt=1/60.0;

  double prev_dist=range;

  for (int t=1; t<lookahead_time; ++t)
  {
    // printf("Sanity check at time step %i at vel (%f, %f):\n", t, vel_x, vel_y);
    // printf("\t- Starting pos:  (%f,%f)\n",pos_x,pos_y);
    pos_x+=dt*vel_x;
    pos_y+=dt*vel_y;
    neigh_pos_x+=dt*neigh_vel_x;
    neigh_pos_y+=dt*neigh_vel_y;
    // printf("\t- Next pos:  (%f,%f)\n", pos_x,pos_y);
    // printf("\t- Original pos:  (%f,%f)\n", getBody()->GetPosition().x,getBody()->GetPosition().y);

    double dist_x=(neigh_pos_x-pos_x);
    double dist_y=(neigh_pos_y-pos_y);
    double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
    if (dist-radius-neigh_radius<0) return t;
    // else if (prev_dist-dist<0) break;
    prev_dist=dist;
  }
  return -1;
}

void agent::recordStep(int t)
{
  // TODO: save output as csv file; easier to add without needing to modify python visualising/data analysis
  // Recording:
  //   - 0: x
  //   - 1: y
  //   - 2: \theta
  //   - 3: radius (size)
  //   - 4: sensor/communication range
  //   - 5: number of neighbours
  //   - 6: average distance to neighbours
  //   - 7: x-target
  //   - 8: x-target tolerance
  //   - 9: y-target
  //   - 10: y-target tolerance
  //   - 11: current world timestep
  //   - 12: agent type (for plotting and analysis)
  double avg_neigh_dist=range;
  if (no_neigh>0) avg_neigh_dist=sum_neigh_dist/no_neigh;
  // if (avg_neigh_dist!=avg_neigh_dist)
  // {
  //   printf("\tProblem:\n\t\t- %f\n\t\t- %f\n\n", sum_neigh_dist, no_neigh);
  // }
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation);

  std::string output_file=results_dir+'/'+filename;

  outfile.open(output_file, std::ios_base::app);
  outfile << pos_x  << " " 
          << pos_y << " " 
          << heading << " "
          << radius << " "
          << range << " "
          << no_neigh << " "
          << avg_neigh_dist << " "
          << targ_x << " "
          << var_x << " "
          << targ_y << " "
          << var_y << " "
          << t << " "
          << agent_type << " ";
  outfile << std::endl; 
  outfile.close();
}