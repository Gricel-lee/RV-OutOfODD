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
  double heading=b2Rot_GetAngle(rotation);//*(M_PI/180.0);
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
  // printf("\t\t-MASS PF vals:  %f/%f  %f/%f  %f\n", pf_x, pf_x_g, pf_y, pf_y_g, targ_theta);
  // printf("\t\t-MASS update:  %f  %f\n", targ_theta, heading);
  double theta_diff=targ_theta-heading;
  if (theta_diff<-M_PI) theta_diff+=2*M_PI;
  else if (theta_diff>M_PI) theta_diff-=2*M_PI;

  theta_acc=theta_diff;
  if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
  else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  // printf("\t\t-Turning info:\n");
  // printf("\t\t\t- Agent heading:  %f\n", heading);
  // printf("\t\t\t- Target heading:  %f\n", targ_theta);
  // printf("\t\t\t- Theta speed:  %f\n", theta_acc);
  
 // Update velocity magnitude
  vel_mag=vel_max*(1-abs(targ_theta)/M_PI);
  // vel_mag=0;
  if (vel_mag<0) vel_mag=0;
  else if (vel_mag>vel_max) vel_mag=vel_max;
  travel_time+=1;
}

/* 
  Sets situation (state), which compose of
  - Agent type 0 neighbour count (then binned)
  ...
  - Agent type N neighbour count (then binned)
  - Shortest time to collision (TTC) (then binned)
*/

neighLocation MassAgent::getNeighLocation(agent* neighbour)
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

  neighLocation neigh_loc;
  neigh_loc.dist=dist;
  neigh_loc.bearing=o_theta;
  neigh_loc.type=neighbour->getAgentType();
  neigh_loc.x=other_pos_x;
  neigh_loc.y=other_pos_y;
  neigh_loc.heading=neighbour->getTheta();
  neigh_loc.radius=neighbour->getRadius();
  neigh_loc.vel=neighbour->getVelMag();

  return neigh_loc;
}

void MassAgent::updateSituation(agent* neighbour)
{
  neighLocation neigh_loc=getNeighLocation(neighbour);

 // Update situation info
  neighbour_types[neigh_loc.type]+=1;

  if (neigh_loc.dist<collision_thresh)
  {
    shortest_ttc_bin=-2;
    return;
  }

  // int ttc=checkFuture(mass_lookahead_time, other_pos_x, other_pos_y, neigh_vel_mag, neigh_theta, neigh_radius);
  int ttc=checkFuture(mass_lookahead_time, neigh_loc.x, neigh_loc.y, neigh_loc.vel, neigh_loc.heading, neigh_loc.radius);
  // printf("\t\t-Future checked!\n");
  if (ttc<0) ttc=mass_lookahead_time+1;
  if (ttc<ttc_violation) shortest_ttc_bin=-1;
  else
  { 
    int temp_ttc_bin=size(TTC);
    for (std::map<int, int>::iterator it=TTC.begin(); it!=TTC.end(); it++)
    {
      if (ttc<it->first) temp_ttc_bin=it->second;
    }
    if (temp_ttc_bin<shortest_ttc_bin) shortest_ttc_bin=temp_ttc_bin;
  }

  if (ttc<temp_ttc) temp_ttc=ttc;
  return;
}


void MassAgent::recordGoalTime()
{
  if (travel_time==0) return;
  goal_travel_times.push_back(travel_time);

  outfile.open(filename, std::ios_base::app);
  outfile << travel_time << std::endl; 
  outfile.close();
  
  float counts=size(goal_travel_times);
  if (counts>0)
  {
    avg_goal_travel_time=0.0;
    for (auto time : goal_travel_times) avg_goal_travel_time+=(time/counts);
  }
  travel_time=0.0;
  printf("Average traveling time is: %f  (success count is %i)\n", avg_goal_travel_time, int(counts));
  return;
}


int MassAgent::situationID()
{
  if (shortest_ttc_bin==-1) return 0; // TTC violation
  else if (shortest_ttc_bin==-2) return 1; // Collision violation

  int situation_id=0;

  string check_log="";

  check_log+="\t-Breakdown of situation:\n";
  check_log+="\t\t-TTC and bin:  "+to_string(temp_ttc)+"  "+to_string(shortest_ttc_bin)+"\n";
  check_log+="\t\t-Agent A neighbour count:  "+to_string(neighbour_types[0])+"\n";
  check_log+="\t\t-Agent B neighbour count:  "+to_string(neighbour_types[1])+"\n";
  
  bool neigh_present=false;

  for (auto const& [key, val] : neighbour_types)
  {
    int neigh_bin=neigh_count_bins[key].size();
    for (auto const& [num, bin] : neigh_count_bins[key])
    {
      if (val<num && bin<neigh_bin) neigh_bin=bin;
    }
    int max_val=situation_neigh_comp[key][0]*situation_neigh_comp[key][1];
    check_log+="\t\t\t-Neighbouring bin for type "+to_string(key)+":  "+to_string(neigh_bin)+"\n";
    situation_id+=(min(max_val,situation_neigh_comp[key][1]*neigh_bin));
    if (val>0) neigh_present=true;
  }
  check_log+="\t\t-Situation ID neighbours:  "+to_string(situation_id)+"\n";
  situation_id+=2; // Add for fail states
  if (neigh_present) situation_id+=shortest_ttc_bin;
  
  // printf("%i  ", situation_id);
  int offset=TTC.size();
  // int offset=0;
  // printf("%i\n", situation_id-offset);
  check_log+="\t\t-Situation ID TTC:  "+to_string(situation_id)+"\n";
  
  // if (neigh_present) printf("%s", check_log.data()), cin.get();

  situation_id=max(2, situation_id-offset);
  // temp_ttc=mass_lookahead_time;
  return situation_id;
}


void MassAgent::setPrevSituation()
{
  prev_situation=situationID();
}


void MassAgent::updateNeighPF(agent* neighbour)
{
 // Update situation
  updateSituation(neighbour);
 
  neighLocation neigh_loc=getNeighLocation(neighbour);

 // Apply PF with corresponding agent type weight
  float neigh_weight=pf_weights[neigh_loc.type];
  pf_x_neigh+=-neigh_weight*(range-neigh_loc.dist)*cos(neigh_loc.bearing);
  pf_y_neigh+=-neigh_weight*(range-neigh_loc.dist)*sin(neigh_loc.bearing);
}

double MassAgent::getGoalDist(double pos_x, double pos_y)
{
  // printf("\t\t- Positions and goal dist:   \n");
  // printf("\t\t\t- MASS position:  (%f, %f)\n", pos_x, pos_y);
  // printf("\t\t\t- Goal position:  (%f (%f), %f (%f))\n", targ_x, var_x, targ_y, var_y);
  // double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  // double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);
  // printf("\t\t\t- Relative goal position:  (%f, %f)\n", relative_targ_x, relative_targ_y);
  // double x_dist=pos_x-relative_targ_x;
  // double y_dist=pos_y-relative_targ_y;
  
  double x_dist=pos_x-targ_x;
  double y_dist=pos_y-targ_y;
  double dist=sqrt(x_dist*x_dist+y_dist*y_dist);

  // printf("\t\t\t- Distance:  %f\n", dist);
  
  return dist;
}


void MassAgent::resetSituation()
{
  for (map<int,int>::iterator it=neighbour_types.begin(); it!=neighbour_types.end(); ++it)
  {
    neighbour_types[it->first]=0;
  }
  shortest_ttc_bin=size(TTC);
  temp_ttc=mass_lookahead_time;
}


float MassAgent::getCollisionThresh()
{
  return collision_thresh;
}


int MassAgent::getTTCThresh()
{
  return ttc_violation;
}


int MassAgent::getShortestTTC()
{
  return shortest_ttc_bin;
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
  // state_trans_count[prev_situation][next_state]+=1;
          
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


void MassAgent::updateTransitionMatrix()
{
  int next_state=situationID();
  // printf("\t\t-Situation is:  %i\n", next_state);
  state_trans_count[prev_situation][next_state]+=1;
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
  outfile << ",f1,f2,";
  for(int i=2; i<total_situations-1; ++i) outfile<<"s"+to_string(i-1)+",";
  outfile << "s"+to_string(total_situations-2) << endl;
 // Create transition matrix, and write row info
  for(int i=0; i<total_situations; ++i)
  {
    string s;
    if (i==0) s="f1,";
    else if (i==1) s="f2,";
    else s="s"+to_string(i-1)+",";
    outfile << s.data();
    for(int j=0; j<total_situations-1; ++j)
    {
      outfile << to_string(state_trans_count[i][j]) << ",";
    }
    outfile << to_string(state_trans_count[i][-1]) << endl;
  }
  outfile.close();
  // printf("PERFORMING MODEL CHECKING\n");
  // modelCheck();
}

bool MassAgent::modelCheck()
{
  printf("\t-Verifying controllers\n");
  Py_Initialize();
  printf("\t\t-Interpreter set\n");
  try
  {
    object model_check_module=import("model_check");
    printf("\t\t-Module imported\n");
    string csv_filename=results_dir+"/"+transition_filename;
    bool result=extract<bool>(model_check_module.attr("testFunc")(csv_filename));
    printf("\t\t-Function imported\n");
    string success="no violations\n";
    if (!result) success="violations occurred\n";
    // printf("\t-Number of failures:  %s\n", success.data());
    return result;
  }
  catch (const error_already_set&)
  {
    printf("CAUGHT PYTHON ERROR\n");
    PyErr_Print();
    return false;
  // //   // return 1;
  }
  printf("\t\t-Complete!\n");
}