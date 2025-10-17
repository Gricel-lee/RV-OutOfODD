#include <mass_agent.h>
#include <cmath>
#include <csv_functions.h>
// #include <Python.h>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
// #include <boost/python/panda.hpp>

// #define M_PI 3.14159265

using namespace boost::python;

MassAgent::MassAgent()
{
  // EMPTY CONSTRUCTOR
}

void MassAgent::updateVel()
{
 // Calculate attractor
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation)*(M_PI/180.0);
  double targ_theta=atan2(targ_y-pos_y, targ_x-pos_x);
  double goal_dist=getGoalDist(pos_x, pos_y);
  double pf_x_g=goal_weight*goal_dist*cos(targ_theta);
  double pf_y_g=goal_weight*goal_dist*sin(targ_theta);

 // Combine potential fields
  double pf_x=pf_x_g+pf_x_neigh;
  double pf_y=pf_y_g+pf_y_neigh;

  pf_x_neigh=0;
  pf_y_neigh=0;

 // Update theta acceleration
  targ_theta=atan2(pf_y, pf_x);
  double theta_diff=targ_theta-heading;
  if (theta_diff<-M_PI) theta_diff+=2*M_PI;
  else if (theta_diff>M_PI) theta_diff-=2*M_PI;

  theta_acc=theta_diff;
  if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
  else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  
 // Update velocity magnitude
  vel_mag=vel_max*(1-abs(theta_diff)/M_PI);
  if (vel_mag<0) vel_mag=0;
  else if (vel_mag>vel_max) vel_mag=vel_max;
}

/* 
  Sets situation (state), which compose of
  - Agent type 0 neighbour count (then binned)
  ...
  - Agent type N neighbour count (then binned)
  - Distance of closest neighbour (then binned)
*/
void MassAgent::updateSituation(agent* neighbour)
{
 // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;

 // Extract useful information from neighbour
  b2Vec2 neigh_position=b2Body_GetPosition(neighbour->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;

  double other_radius=neighbour->getRadius();
  double dist_x=other_pos_x-pos_x;
  double dist_y=other_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double o_theta=atan2(dist_y, dist_x);
  dist-=(radius+other_radius);

  int neighbour_type=neighbour->getAgentType();

 // Update situation info
  neighbour_types[neighbour_type]+=1;
  // Go through (sorted) distance zones keys, and find ID
    // Note: should be sorted by default
  
  if (dist<collision_thresh)
  {
    closest_dist_zone=-1;
    printf("COLLISION\n");
    return;
  }

  int temp_dist_zone=0;
  for (std::map<float, int>::iterator it=distance_zones.begin(); it!=distance_zones.end(); it++)
  {
    if (dist<it->first) temp_dist_zone=it->second;
  }
  if (temp_dist_zone<closest_dist_zone) closest_dist_zone=temp_dist_zone;
  return;
}

int MassAgent::situationID()
{
  if (closest_dist_zone==-1) return 0;

  int situation_id=0;
  vector<int> vec_situation;
  for (auto const& [key, val] : neighbour_types)
  {
    int max_val=situation_neigh_comp[key][0]*situation_neigh_comp[key][1];
    situation_id+=(min(max_val,situation_neigh_comp[key][1]*val));
  }
  situation_id+=(closest_dist_zone+1);
  int offset=distance_zones.size();
  situation_id=max(1, situation_id-offset);
  return situation_id;
}

void MassAgent::setPrevSituation()
{
  prev_situation=situationID();
}

void MassAgent::updateNeighPF(agent* neighbour)
{
 // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;

 // Extract useful information from neighbour
  b2Vec2 neigh_position=b2Body_GetPosition(neighbour->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;

  double other_radius=neighbour->getRadius();
  double dist_x=other_pos_x-pos_x;
  double dist_y=other_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double o_theta=atan2(dist_y, dist_x);
  dist-=(radius+other_radius);

  int neighbour_type=neighbour->getAgentType();

 // Update situation info
  neighbour_types[neighbour_type]+=1;
  // Go through (sorted) distance zones keys, and find ID
    // Note: should be sorted by default
  int temp_dist_zone=0;
  for (std::map<float, int>::iterator it=distance_zones.begin(); it!=distance_zones.end(); it++)
  {
    if (dist<it->first) temp_dist_zone=it->second;
  }
  if (temp_dist_zone<closest_dist_zone) closest_dist_zone=temp_dist_zone;

 // Apply PF with corresponding agent type weight
  float neigh_weight=pf_weights[neighbour_type];
  pf_x_neigh+=-neigh_weight*(range-dist)*cos(o_theta);
  pf_y_neigh+=-neigh_weight*(range-dist)*sin(o_theta);
}

double MassAgent::getGoalDist(double pos_x, double pos_y)
{
  double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);
  double x_dist=pos_x-relative_targ_x;
  double y_dist=pos_y-relative_targ_y;
  double dist=sqrt(x_dist*x_dist+y_dist*y_dist);
  return dist;
}

void MassAgent::resetSituation()
{
  for (map<int,int>::iterator it=neighbour_types.begin(); it!=neighbour_types.end(); ++it)
  {
    neighbour_types[it->first]=0;
  }
  closest_dist_zone=size(distance_zones);
}

float MassAgent::getCollisionThresh()
{
  return collision_thresh;
}

void MassAgent::recordStep(int t)
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
  //   - 12: previous state
  //   - 13: next state
  //   - 14: agent type (for plotting and analysis)
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

  int next_state=situationID();
  state_trans_count[prev_situation][next_state]+=1;
          
  // printf("\t - State transition is: %i -> %i\n", prev_situation, next_state);

  outfile.open(filename, std::ios_base::app);
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
          << prev_situation << " "
          << next_state << " "
          << agent_type << " ";
  outfile << std::endl; 
  outfile.close();
}

void MassAgent::saveTransitionMatrix()
{
  /* 
  updating transition info 
    - Update transition matrix
    - Write to csv file
  */
  outfile.open(results_dir+"/"+transition_filename);

 // Write column info
  outfile << ",f1,";
  for(int i=1; i<total_situations-1; ++i) outfile<<"s"+to_string(i)+",";
  outfile << "s"+to_string(total_situations-1) << endl;
 // Create transition matrix, and write row info
  for(int i=0; i<total_situations; ++i)
  {
    string s;
    if (i==0) s="f1,";
    else s="s"+to_string(i)+",";
    outfile << s.data();
    for(int j=0; j<total_situations-1; ++j)
    {
      outfile << to_string(state_trans_count[i][j]) << ",";
    }
    outfile << to_string(state_trans_count[i][-1]) << endl;
  }
  outfile.close();
  printf("PERFORMING MODEL CHECKING\n");
  modelCheck();
}

bool MassAgent::modelCheck()
{
  Py_Initialize();
  printf("\t-Interpreter set\n");
  try
  {
    object model_check_module=import("model_check");
    printf("\t-Module imported\n");
    string csv_filename=results_dir+"/"+transition_filename;
    bool result=extract<bool>(model_check_module.attr("testFunc")(csv_filename));
    printf("\t-Function imported\n");
    string success="no violations\n";
    if (!result) success="violations occurred\n";
    printf("\t-Number of failures:  %s\n", success.data());

    return result;

  }
  catch (const error_already_set&)
  {
    printf("CAUGHT PYTHON ERROR\n");
    PyErr_Print();
    return false;
  // //   // return 1;
  }
  printf("\t - Complete!\n");
}