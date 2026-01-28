#include <box2d/box2d.h>
#include "yaml-cpp/yaml.h"
#include <regex>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <stdlib.h>
#include <agent.h>
#include <mass_agent.h>
#include <normal_agent.h>

#define PI 3.14159265
// #define THRESH 0.25

using namespace std;

struct {
 // For seeds
  std::uniform_real_distribution<double> distribution_seeds;
  default_random_engine gen;
 
 // // For goal location assignment
  random_device dev;
  uniform_int_distribution<std::mt19937::result_type> choice;
} rng_setup;


string parseYAMLENV(string yaml_line)
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


void clearScreenANSI() {
    std::cout << "\033[2J\033[1;1H";
}


b2ShapeId defineBody(b2BodyId particle_id, double radius, double density, double friction)
{
  // Define circle-mass for our dynamic body.
  b2Circle dynamic_circ;
  dynamic_circ.center=(b2Vec2){0.0f, 0.0f};
  dynamic_circ.radius=radius;

  b2ShapeDef shapeDef = b2DefaultShapeDef();
  shapeDef.density = density;
  shapeDef.material.friction = friction;

  b2Vec2 position=b2Body_GetPosition(particle_id);
  b2Rot rotation=b2Body_GetRotation(particle_id);

  b2ShapeId particle=b2CreateCircleShape(particle_id, &shapeDef, &dynamic_circ);
  return particle;
}


void setGoal(agent* ego, vector<vector<double>> goals, int selection)
{
  ego->setTarget(goals[selection][0], goals[selection][2], goals[selection][1], goals[selection][2]);
}


void checkNeigh(NormalAgent* ego, agent* neigh)
{
  b2Vec2 position=b2Body_GetPosition(ego->getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double colregs_range=ego->getRange();
  double vel_mag=ego->getMaxVel();

  b2Vec2 neigh_position=b2Body_GetPosition(neigh->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;
  double other_vel_mag=neigh->getVelMag();

  double dist_x=pos_x-other_pos_x;
  double dist_y=pos_y-other_pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double closest_dist_point=dist-neigh->getRadius();
  if (closest_dist_point<colregs_range)
  {
    ego->updateVel(neigh);
    ++ego->no_neigh;
    ego->sum_neigh_dist+=closest_dist_point;
  }
}


void checkNeighMass(MassAgent* ego, agent* neigh)
{
  b2Vec2 position=b2Body_GetPosition(ego->getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double mass_range=ego->getRange();
  double vel_mag=ego->getMaxVel();

  b2Vec2 neigh_position=b2Body_GetPosition(neigh->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;
  double other_vel_mag=neigh->getVelMag();

  double dist_x=pos_x-other_pos_x;
  double dist_y=pos_y-other_pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double closest_dist_point=dist-neigh->getRadius();
  if (closest_dist_point<mass_range)
  {
    // printf("\t\t-Updating PF...\n");
    ego->updateNeighPF(neigh);
    // printf("\t\t-Updated PF!\n");
    ++(ego->no_neigh);
    ego->sum_neigh_dist+=closest_dist_point;
  }
}


// Returns true if crashed
bool updateMassSituation(MassAgent* ego, agent* neigh)
{
  b2Vec2 position=b2Body_GetPosition(ego->getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double mass_range=ego->getRange();
  double vel_mag=ego->getMaxVel();

  b2Vec2 neigh_position=b2Body_GetPosition(neigh->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;
  double other_vel_mag=neigh->getVelMag();

  double dist_x=pos_x-other_pos_x;
  double dist_y=pos_y-other_pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double closest_dist_point=dist-neigh->getRadius();
  if (closest_dist_point<mass_range) 
  {
    ego->updateSituation(neigh);
  }
  if (closest_dist_point-ego->getRadius()<ego->getCollisionThresh())
  {
    // printf("DOUBLE CHECK COLLISION\n");
    return true;
  }
  return false;
}


void resetSim(agent* ego, b2WorldId worldID, vector<vector<double>> goal_locations)
{
  // std::random_device dev;
  // std::mt19937 rng(dev());
  // std::uniform_int_distribution<std::mt19937::result_type> choice(0,goal_locations.size()-1);
  setGoal(ego, goal_locations, rng_setup.choice(rng_setup.dev));
  // printf("\t-New goals\n");
  float x, y, theta;
  int start_loc=rng_setup.choice(rng_setup.dev);
  x=goal_locations[start_loc][0]+(1-2*drand48())*goal_locations[start_loc][2];
  y=goal_locations[start_loc][1]+(1-2*drand48())*goal_locations[start_loc][2];
  theta=(1-2*drand48())*180.0;
  // printf("\t-Setting pose...\n");
  // ego->setBodyPose(x, y, theta);
  // b2Vec2 position={x, y};
  // b2Rot heading=b2MakeRot(theta);
  // printf("\t-Vals calculated...\n");
  b2DestroyBody(ego->getBodyID());
  ego->setBodyDefPose(x, y, theta);
  ego->setBodyID(b2CreateBody(worldID, ego->getBodyDef()));
  // b2Body_SetTransform(ego->getBodyID(),position,heading); 
}


vector<vector<double>> createGoalLocations(YAML::Node goal_config)
{
  vector<vector<double>> goal_locations;
  YAML::Node goals=goal_config["goals"];
  for(YAML::const_iterator it=goals.begin(); it!=goals.end(); ++it)
  {
    std::string key=it->first.as<std::string>();
    double goal_x=goal_config["goals"][key]["x"].as<double>();
    double goal_y=goal_config["goals"][key]["y"].as<double>();
    double tolerance=goal_config["goals"][key]["tolerance"].as<double>();
    vector<double> new_goal={goal_x, goal_y, tolerance};
    goal_locations.push_back(new_goal);
  }
  return goal_locations;
}


void sim(int TT, vector<NormalAgent*> colregs, vector<MassAgent*> MASS, vector<vector<double>> goal_locations, b2WorldId worldID)
{
  // Define simulation parameters
  // Prepare for simulation. Typically we use a time step of 1/60 of a
  // second (60Hz) and 10 iterations. This provides a high quality simulation
  // in most game scenarios.
  float timeStep = 1.0f / 60.0f;

  /* 
    Substep count: 
      - higher->accuracy
      - lower->performance
  */
  int subStepCount=4;
  int cmd_line_len=0;

 // Run simulation 
  for (int t=0; t<TT; ++t)
  {
    // printf("\t-Updating time...\n");
    if (TT*0.01<1)
    {
      cout << string(cmd_line_len,'\b') << flush;
      string progress="\t-Time step: "+to_string(t)+"/"+to_string(TT)+"\t\t\t\r";
      cmd_line_len=progress.length();
      cout << progress.data();
    }
    else if (t%int(0.01*TT)==0)
    { 
      cout << string(cmd_line_len,'\b') << flush;
      string progress="\t-Time step: "+to_string(t)+"/"+to_string(TT)+"\t\t\t\r";
      cmd_line_len=progress.length();
      cout << progress.data();
    }
    
    // For each vessel
    // 1. Check goal and neighbour distance
    // 2. Update velocity
    // 3. Record New position

   /* Calculate new velocity commands */
    // printf("\t-Calculating for colregs...\n");
    vector<NormalAgent*>::iterator it_ego=colregs.begin();
    for(it_ego; it_ego!=colregs.end(); ++it_ego)
    {
      if((*it_ego)->checkGoal()) setGoal(*it_ego, goal_locations, rng_setup.choice(rng_setup.dev));
      vector<NormalAgent*>::iterator it_neigh=colregs.begin();
      for (it_neigh; it_neigh!=colregs.end(); ++it_neigh) if (it_neigh!=it_ego) checkNeigh(*it_ego, *it_neigh);
      for (vector<MassAgent*>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
      {
        checkNeigh(*it_ego, *it_mass);
      }
    }

    // printf("\t-Calculating for MASS...\n");
    for (vector<MassAgent*>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
    {
      // printf("\t\t-Checking goal...\n");

      b2Vec2 position=b2Body_GetPosition((*it_mass)->getBodyID());
      double pos_x=position.x;
      double pos_y=position.y;
      // printf("Distance to goal:  %f\n", (*it_mass)->getGoalDist(pos_x, pos_y));
      // if (t%1000==0)
      // {
      //   string dist_log="";
      //   dist_log+="\t\t-Agent/Goal Positions:  ("+str(pos_x)+", "+str(pos_y)+")  ("+(*it_mass)+    "\n";
      //   dist_log+="\t\t-Distance to goal:  "+str((*it_mass)->getGoalDist(pos_x, pos_y))+"\n";
      // }

       // printf("\t\t-Distance to goal:  %f\n", (*it_mass)->getGoalDist(pos_x, pos_y));

      if ((*it_mass)->checkGoal())
      {
        (*it_mass)->recordGoalTime();
        while ((*it_mass)->checkGoal()) setGoal(*it_mass, goal_locations, rng_setup.choice(rng_setup.dev));
      // printf("\t\t-Checked goal.\n");
      }

      vector<NormalAgent*>::iterator it_neigh=colregs.begin();
      for (it_neigh; it_neigh!=colregs.end(); ++it_neigh) 
      {
        // printf("\t\t-Checking neighbour...\n");
        checkNeighMass(*it_mass, *it_neigh);
        // printf("\t\t-Checked neighbour.\n");
      }
      vector<MassAgent*>::iterator it_mass_neigh=MASS.begin();
      for (it_mass_neigh; it_mass_neigh!=MASS.end(); ++it_mass_neigh)
      {
        if(it_mass!=it_mass_neigh) checkNeighMass(*it_mass, *it_mass_neigh);
      }
      // printf("\t\t-Setting previous situation...\n");
      (*it_mass)->setPrevSituation();
    }

    // Update vessel's heading
    // Update vessel's velocity

    // printf("\t-Updating MASS...\n");
    for (vector<MassAgent*>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
    {
      (*it_mass)->updateVel();
      double mass_ang_velocity=(*it_mass)->getThetaAcc();
      b2Vec2 mass_lin_velocity={(*it_mass)->getVelX(), (*it_mass)->getVelY()};
    
      b2Body_SetLinearVelocity((*it_mass)->getBodyID(), mass_lin_velocity);
      b2Body_SetAngularVelocity((*it_mass)->getBodyID(), mass_ang_velocity);
      // printf("\t\t-MASS vels:  %f  %f  %f\n",(*it_mass)->getVelX(), (*it_mass)->getVelY(), mass_ang_velocity);
    }

    // printf("\t-Updating colregs...\n");
   
   /* Update with new velocity commands */
    it_ego=colregs.begin();
    for(it_ego; it_ego!=colregs.end(); ++it_ego)
    {
      // Update the robot's heading
      // Update robot's velocity
      (*it_ego)->updateVelMag();
      (*it_ego)->updateTheta();

      double ang_velocity=(*it_ego)->getThetaAcc();
      b2Vec2 lin_velocity={(*it_ego)->getVelX(), (*it_ego)->getVelY()};
      
      // printf("\t\t-COLREGS vels:  %f  %f  %f\n",(*it_ego)->getVelX(), (*it_ego)->getVelY(), ang_velocity);

      b2Body_SetLinearVelocity((*it_ego)->getBodyID(), lin_velocity);
      b2Body_SetAngularVelocity((*it_ego)->getBodyID(), ang_velocity);
    }

    // printf("\t-Taking world step...\n");
    // Instruct the world to perform a single step of simulation.
    // It is generally best to keep the time step and iterations fixed.
    b2World_Step(worldID, timeStep, subStepCount);

    // printf("\t-Recording colregs step...\n");
    it_ego=colregs.begin();
    for(it_ego; it_ego!=colregs.end(); ++it_ego)
    {
      // (*it_ego)->recordStep(t);
      (*it_ego)->agentNeighReset();
    }

   /* Check situation transition for MASS */ 
    bool fail_reset=false;
    bool fail_ttc=false;
    bool fail_collision=false;

    // printf("\t-Recording MASS step...\n");
    for (vector<MassAgent*>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass) 
    {
      // Go through all agents, and update situation to record step
      // printf("\t\t-resetting situations...\n");
      (*it_mass)->resetSituation();
      bool collision_temp=false; 
      for (vector<MassAgent*>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
      {          
        vector<NormalAgent*>::iterator it_neigh=colregs.begin();
        for (it_neigh; it_neigh!=colregs.end(); ++it_neigh) 
        {
          // printf("\t\t-Updating situation...\n");
          // collision_temp=updateMassSituation(*it_mass, *it_neigh);
          updateMassSituation(*it_mass, *it_neigh);
          // printf("\t\t-Situation updated!\n\n");    
          // printf("\t\t-Collided?  %i\n", collision_temp);
          // if (collision_temp) fail_collision=true;
          if ((*it_mass)->getShortestTTC()==-1) fail_ttc=true, fail_reset=true;
          else if ((*it_mass)->getShortestTTC()==-2) fail_collision=true, fail_reset=true;
        }
        vector<MassAgent*>::iterator it_mass_neigh=MASS.begin();
        for (it_mass_neigh; it_mass_neigh!=MASS.end(); ++it_mass_neigh)
        {
          if(it_mass!=it_mass_neigh)
          {
            // printf("\t\t-Updating MASS situation...\n");
            // collision_temp=updateMassSituation(*it_mass, *it_mass_neigh);
            updateMassSituation(*it_mass, *it_mass_neigh);
            // if (collision_temp || (*it_mass)->getShortestTTC()<0) fail_reset=true;
            if ((*it_mass)->getShortestTTC()==-1) fail_ttc=true, fail_reset=true;
            else if ((*it_mass)->getShortestTTC()==-2) fail_collision=true, fail_reset=true;
          }
        }    
      }
      // printf("\t\t-Updating transition matrix...\n");
      (*it_mass)->updateTransitionMatrix();
      // printf("\t\t-Update complete!\n");

      // if (collision_temp) collision=true;
      // printf("\t\t-Recording step...\n");
      // (*it_mass)->recordStep(t);  
      // printf("\t\t-Resetting neigh...\n");
      // printf("\t\t-reseting neighbours...\n");
      // printf("\t\t-Resetting neighbour count...\n");
      (*it_mass)->agentNeighReset();
      // printf("\t\t-Reset complete!\n");
      // printf("\t\t-Resetting situation...\n");
      // printf("\t\t-reseting situation...\n");
      // printf("\t\t-Resetting situation...\n");  
      (*it_mass)->resetSituation();
      // printf("\t\t-Reset complete!\n");
      // printf("\t\t-Situation reset!\n");  
    }
    if (fail_reset)
    {
      int fail_cond;
      if (fail_ttc) fail_cond=1;
      else if (fail_collision) fail_cond=2;
      printf("Reseting sim due to failure (f%i) at time %i\n", fail_cond, t);
      for (int i=0; i<colregs.size(); ++i)
      {
        // printf("\nResetting for vessel %i\n", i);
        resetSim(colregs[i], worldID, goal_locations);
      }
      if (MASS.size()==1)
      {
        b2DestroyBody(MASS[0]->getBodyID());
        MASS[0]->setBodyDefPose(0, 0, 0);
        MASS[0]->setBodyID(b2CreateBody(worldID, MASS[0]->getBodyDef()));
        // printf("\t-Placing MASS at origin\n");
      }
      else
      {
        for (auto mass : MASS) resetSim(mass, worldID, goal_locations);
      }
    }
    // printf("\t-Episode simulation complete!\n");
  }
  return;
}


vector<NormalAgent*> createCOLREGAgents(YAML::Node config, vector<vector<double>> goal_locations, b2WorldId worldID)
{
  std::vector<NormalAgent*> colregs;
  YAML::Node agents=config["agents"];
  for(YAML::const_iterator it=agents.begin(); it!=agents.end(); ++it)
  {
    std::string key=it->first.as<std::string>();         // <- key
    int N=config["agents"][key]["N"].as<int>();
    string agent_yaml_file=config["agents"][key]["yaml file"].as<string>();
    agent_yaml_file=parseYAMLENV(agent_yaml_file);
    for (int n=0; n<N; ++n)
    {
      string results_file=key+"_"+to_string(n)+".txt";
      double seed=rng_setup.distribution_seeds(rng_setup.gen);
      colregs.push_back(new NormalAgent(agent_yaml_file, results_file, seed));
      // setGoal(&colregs.back(), goal_locations, rng_setup.choice(rng_setup.dev));
      setGoal(colregs.back(), goal_locations, rng_setup.choice(rng_setup.dev));
      double init_x, init_y, init_theta;
      int start_loc=rng_setup.choice(rng_setup.dev);
      init_x=goal_locations[start_loc][0]+(1-2*drand48())*goal_locations[start_loc][2];
      init_y=goal_locations[start_loc][1]+(1-2*drand48())*goal_locations[start_loc][2];
      init_theta=(1-2*drand48())*180.0;

      colregs.back()->setBodyDefPose(init_x, init_y, init_theta);
      colregs.back()->setBodyID(b2CreateBody(worldID, colregs.back()->getBodyDef()));
      defineBody(colregs.back()->getBodyID(), colregs.back()->getRadius(), 40, 0.3);
    }
  }
  return colregs;
}


vector<MassAgent*> createMassAgents(YAML::Node config, vector<vector<double>> goal_locations, b2WorldId worldID)
{
  vector<MassAgent*> MASS;
  YAML::Node agents=config["mass agents"];
  for(YAML::const_iterator it=agents.begin(); it!=agents.end(); ++it)
  {
    std::string key=it->first.as<std::string>();
    int N=config["mass agents"][key]["N"].as<int>();
    string agent_yaml_file=config["mass agents"][key]["yaml file"].as<string>();
    agent_yaml_file=parseYAMLENV(agent_yaml_file);
    for (int n=0; n<N; ++n)
    {
      string results_file=key+"_"+to_string(n)+".txt";
      double seed=rng_setup.distribution_seeds(rng_setup.gen);
      MASS.push_back(new MassAgent(agent_yaml_file, results_file, seed));

      int start_loc=rng_setup.choice(rng_setup.dev);

      double init_x=0;//goal_locations[start_loc][0]+(1-2*drand48())*goal_locations[start_loc][2];
      double init_y=0;//goal_locations[start_loc][1]+(1-2*drand48())*goal_locations[start_loc][2];
      double init_theta=(1-2*drand48())*PI;

      MASS.back()->setBodyDefPose(init_x, init_y, init_theta);

      setGoal(MASS.back(), goal_locations, rng_setup.choice(rng_setup.dev));

      MASS.back()->setBodyID(b2CreateBody(worldID,MASS.back()->getBodyDef()));
      defineBody(MASS.back()->getBodyID(), MASS.back()->getRadius(), 40, 0.3);
    }
  }
  return MASS;
}


int main(int argc, const char* argv[])
{
   // Parameters from input args
    string yaml_file(argv[1]);
    YAML::Node config = YAML::LoadFile(yaml_file);
    printf("Loaded scenario config\n");
    string goal_yaml_file=config["goal file"].as<string>();
    goal_yaml_file=parseYAMLENV(goal_yaml_file);
    YAML::Node goal_config=YAML::LoadFile(goal_yaml_file);
    printf("Loaded goal locations config\n");

    const int TT=config["TT"].as<int>();
    // const string results_dir=config["results directory"].as<string>();
    
    const int EE=config["EE"].as<int>();

   // Goal locations setup
    printf("Instantiating goal locations...\n");
    vector<vector<double>> goal_locations=createGoalLocations(goal_config);
    printf("Instantiated goal locations\n");
   // RNG setup
    srand48(time(NULL));
    // rng_setup rng_vals;
    rng_setup.distribution_seeds=uniform_real_distribution<double>(0,1e7);
    rng_setup.gen.seed(time(NULL));
    mt19937 rng(rng_setup.dev());
    rng_setup.choice=uniform_int_distribution<std::mt19937::result_type>(0,goal_locations.size()-1);

   // Run simulation with settings
    for (int e=0; e<EE; ++e)
    {
     clearScreenANSI();
     printf("Performing episode  %i of %i\n", e+1, EE);
     // Set up simulated world/environment
      // Define the gravity vector.
      b2Vec2 gravity={0.0f, 0.0f};

      // Construct a world object, which will hold and simulate the rigid bodies.
      b2WorldDef worldDef = b2DefaultWorldDef();
      worldDef.gravity=gravity;
      b2WorldId worldID=b2CreateWorld(&worldDef);

     // Create COLREG agents
      printf("\t-Creating COLREG agents...\n");
      vector<NormalAgent*> colregs=createCOLREGAgents(config, goal_locations, worldID);
      printf("\t-COLREG agents instantiated\n");

     // Create mass agent(s)
      printf("\t-Creating MASS agents...\n");
      std::vector<MassAgent*> MASS=createMassAgents(config, goal_locations, worldID); // Only using one MASS, but gives possibility to instantiate more
      printf("\t-MASS agents instantiated\n");
     
     // Simulate 
      sim(TT, colregs, MASS, goal_locations, worldID);
      printf("\n\t-Destroyed world\n");
     // Record transition matrix
      for (auto mass : MASS) 
      {
        /*
          Save transition matrix after each episode
          Perform model check to get viable controllers after all episodes 
        */
        mass->saveTransitionMatrix();
        // if (e==EE-1) mass->modelCheck();
        delete mass;
      }
      MASS.clear();
      printf("\t-Destroyed MASS\n");
      for (auto vessel : colregs) 
      {
        delete vessel;
      }
      colregs.clear();
      printf("\t-Destroyed COLREGs\n");
      b2DestroyWorld(worldID);
    }
}