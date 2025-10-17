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
#include "normal_agent.h"

#define PI 3.14159265

NormalAgent::NormalAgent()
{
  // EMPTY CONSTRUCTOR
}

void NormalAgent::updateVel(agent* neighbour)
{
 // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y; 
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation);

  double targ_theta=atan2(targ_y-pos_y, targ_x-pos_x);

 // Extract useful information from neighbour
  b2Vec2 neigh_position=b2Body_GetPosition(neighbour->getBodyID());
  double neigh_pos_x=neigh_position.x;
  double neigh_pos_y=neigh_position.y; 
  b2Rot neigh_rotation=b2Body_GetRotation(neighbour->getBodyID());
  double neigh_heading=b2Rot_GetAngle(rotation);
  double neigh_theta=atan2(neigh_pos_y, neigh_pos_x);
  
  double neigh_vel_mag=neighbour->getVelMag();
  double neigh_radius=neighbour->getRadius();

  double dist_x=neigh_pos_x-pos_x;
  double dist_y=neigh_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  dist-=(radius+neigh_radius);

  // if (dist>min_dist) return;

 // Other vessel's relative position in ego's reference frame
  double neigh_ego_relative_pos_x=(cos(-heading)*dist_x-sin(-heading)*dist_y);
  double neigh_ego_relative_pos_y=(sin(-heading)*dist_x+cos(-heading)*dist_y);
  double neigh_ego_relative_theta=atan2(neigh_ego_relative_pos_y, neigh_ego_relative_pos_x);

 // Other vessel's relative position in ego's reference frame rotated by angle to goal
  double neigh_targ_relative_pos_x=cos(-targ_theta)*dist_x-sin(-targ_theta)*dist_y;
  double neigh_targ_relative_pos_y=sin(-targ_theta)*dist_x+cos(-targ_theta)*dist_y;
  double neigh_targ_relative_theta=atan2(neigh_targ_relative_pos_y, neigh_targ_relative_pos_x);

 // Check if ship is oncoming
  double oncoming_thresh_min=2*PI*(3/8.0);
  double oncoming_thresh_max=2*PI*(5/8.0);
  
  double heading_norm=heading;
  double neigh_heading_norm=neigh_heading;
  double targ_theta_norm=targ_theta;

  if (heading_norm<0) heading_norm=2*PI+heading_norm;
  if (neigh_heading_norm<0) neigh_heading_norm=2*PI+neigh_heading_norm;
  if (targ_theta_norm<0) targ_theta_norm=2*PI+targ_theta_norm;
  
  double heading_diff_ego_frame=neigh_heading_norm-heading_norm;
  if (heading_diff_ego_frame<0) heading_diff_ego_frame=2*PI+heading_diff_ego_frame;

  double heading_diff_targ_frame=neigh_heading_norm-targ_theta_norm;
  if (heading_diff_targ_frame<0) heading_diff_targ_frame=2*PI+heading_diff_targ_frame;

  int lookahead_time=600;

  bool updated=false;

  int time_collision=checkFuture(lookahead_time, neigh_pos_x, neigh_pos_y, neigh_vel_mag, neigh_theta, neigh_radius);

 // Situation identification
  // Overtaking
  // if (!updated && neigh_ego_relative_theta<67.5*(PI/180.0) && neigh_ego_relative_theta>-67.5*(PI/180.0) && radius>neigh_radius
  if (neigh_ego_relative_theta<67.5*(PI/180.0) && neigh_ego_relative_theta>-67.5*(PI/180.0) && radius>neigh_radius 
          && (heading_diff_ego_frame<PI/4 || heading_diff_ego_frame>2*PI*3/4.0))
  {
    updated=overTakingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, neigh_vel_mag, time_collision);
    // updated=true;
  } 

  // if (time_collision<0) return;

  // Oncoming scenario 
  if (heading_diff_ego_frame>oncoming_thresh_min && heading_diff_ego_frame<oncoming_thresh_max)
  {
    updated=oncomingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, time_collision);
    // updated=true;
  }

  // Crossing
  // if (!updated && neigh_ego_relative_pos_x>0 && neigh_ego_relative_pos_y<neigh_radius && (heading_diff_ego_frame>PI*1/4.0 && heading_diff_ego_frame<PI*3/4.0))
  if (neigh_ego_relative_pos_x>0 && neigh_ego_relative_pos_y<neigh_radius && (heading_diff_ego_frame>PI*1/4.0 && heading_diff_ego_frame<PI*3/4.0))
  {
    // if (time_collision>0) printf("In crossing event collision will occur in %i timesteps\n", time_collision);
    updated=crossingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, time_collision);
    // if (updated && time_collision>1) printf("In crossing event collision will occur in %i timesteps\n", time_collision);
  }

  // if (!updated && neigh_ego_relative_pos_x>-neigh_radius && dist<cmf_dist)
  // if (neigh_ego_relative_pos_x>-neigh_radius && dist<cmf_dist)
  // {
  //   double turn_angle=atan2(dist_y, dist_x);
  //   new_theta_acc=-turn_angle;
  //   updated=true;
  // }

  if (time_collision>1)
  {
    double max_crash_time=600.0;
    double offset_vel=1;
    double temp_vel_mag=(vel_mag*time_collision)/(max_crash_time*offset_vel);
    if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  }
  // else
  // {
  //   double temp_vel_mag=vel_max*(dist/(range-radius-neigh_radius));
  // }
  // if (updated) min_dist=dist;

}

int NormalAgent::checkFuture(int lookahead_time, double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double vel_x=getVelX();
  double vel_y=getVelY();

  double neigh_vel_x=cos(neigh_theta)*neigh_vel_mag;
  double neigh_vel_y=sin(neigh_theta)*neigh_vel_mag;
  double dt=1/60.0;

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
  }
  return -1;
}

bool NormalAgent::oncomingUpdate(double other_targ_relative_pos_x, double other_targ_relative_pos_y, double dist, double other_radius, int time_collision)
{
  if (time_collision<1) return false;
 // Get new target point
  double temp_theta_acc;

  double frame_theta=egoAngle();
  double oncoming_goal_theta=newAngleWorldFrame(other_targ_relative_pos_x, other_targ_relative_pos_y, other_radius, frame_theta);

  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
  temp_theta_acc=(oncoming_goal_theta-heading);

  if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  return true;
}

bool NormalAgent::overTakingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, double other_vel_mag, int time_collision)
{
   // If other ship is smaller, assume you can go faster and perform overtaking procedure
    if (radius>other_radius)
    {
      double frame_theta=egoAngle();
      double overtaking_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);

      b2Rot rotation=b2Body_GetRotation(getBodyID());
      double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
      double temp_theta_acc=(overtaking_goal_theta-heading);

      // if (other_pos_x<radius+other_radius && other_pos_y<0) temp_theta_acc*=-1;    
      if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
      return true;
    }
   // If going faster than larger ship slow appropriately
    else if (new_vel_mag>other_vel_mag) 
    {
      new_vel_mag=other_vel_mag;
      // return false;
    }
    return false;
}

bool NormalAgent::makeWayUpdate(double o_relative_theta, double other_radius)
{
 // Ship is behind, let's get out of way if they are bigger
  if (radius<other_radius)
  {
    double adjusted_theta=(-PI)-o_relative_theta;
    double temp_theta_acc=-d_theta*adjusted_theta;
    if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
    return true;
  }
  return false;
}

bool NormalAgent::crossingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, int time_collision)
{
  if (time_collision<1) return false;

  // double frame_theta=targAngle();
  double frame_theta=egoAngle();
  double crossing_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);
  
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
  double temp_theta_acc=(crossing_goal_theta-heading);
  
  if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  // new_theta_acc=temp_theta_acc;
  return true;
}

double NormalAgent::newAngleWorldFrame(double other_pos_x, double other_pos_y, double other_radius, double frame_theta)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y; 

 // Get new target point
  double new_frame_goal_x=other_pos_x;
  double new_frame_goal_y=other_pos_y-2*(other_radius+radius);

  double new_goal_x=cos(-frame_theta)*new_frame_goal_x-sin(-frame_theta)*new_frame_goal_y+pos_x;
  double new_goal_y=sin(-frame_theta)*new_frame_goal_x+cos(-frame_theta)*new_frame_goal_y+pos_y;
  
  double new_goal_theta=atan2(new_goal_y, new_goal_x);

  return new_goal_theta;
}

  // Check for general problems; is there a ship infront of us (just slow down for now?)
  // else if (neigh_ego_relative_pos_x>-neigh_radius && dist<cmf_dist)
  // {
  //   // double temp_vel_mag=vel_max*((dist-cmf_dist)/(range-radius-neigh_radius));
  //   // temp_vel_mag=std::max(temp_vel_mag, 0.0);
  //   // if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  //   // double force=(cmf_dist/dist);
  //   double pf_pos_x=(pos_x-neigh_pos_x);
  //   double pf_pos_y=(pos_y-neigh_pos_y);

  //   double new_theta=atan2(pf_pos_y, pf_pos_x);
  //   double temp_theta_acc=heading-new_theta;
  //   if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;

  // }
  // else
  // {
  //   double temp_vel_mag=vel_max*((dist-cmf_dist)/(range-radius-neigh_radius));
  //   temp_vel_mag=std::max(temp_vel_mag, 0.0);
  //   if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  // }
 // Distance from forbidden zones 