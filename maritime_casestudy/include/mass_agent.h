#include <agent.h>
// #include <vision_unit.h>
#include <filesystem>
#include <csv_functions.h>
#include<algorithm>

class MassAgent : public agent
{
  public:
    MassAgent();
    MassAgent(std::string yaml_file, std::string results_file, double seed) : agent(yaml_file, results_file, seed)
    {
      YAML::Node config=YAML::LoadFile(yaml_file);

      std::default_random_engine gen;

      collision_thresh=config["collision threshold"].as<float>();


     // Set potential field weight
      YAML::Node potential_field_weights=config["potential field weights"];
      {
        YAML::Node::iterator it;
        for (it=potential_field_weights.begin(); it!=potential_field_weights.end(); ++it)
        {
          string key=it->first.as<std::string>();

          if (key=="goal") 
          {
            goal_weight=potential_field_weights[key]["weight"].as<double>();
          }
          else
          {
            int type=potential_field_weights[key]["type"].as<int>();
            pf_weights[type]=potential_field_weights[key]["weight"].as<float>();

           // Get bins
            vector<int> neighbour_bins=potential_field_weights[key]["bins"].as<vector<int>>();

            int bin_count=0;
            for(vector<int>::iterator it_bin=neighbour_bins.begin(); it_bin!=neighbour_bins.end(); it_bin++)
            {
              vector<int> key={type, *it_bin};
              neigh_count_bins[key]=bin_count;
              ++bin_count;
            }
            situation_neigh_comp[type]={bin_count+1};
          }
        }
      }

     // Set up distance zones
      /* 
        TODO: Set up distance zones so that only n-1 needs to be in YAML file, and anything above is considered in last zone
      */ 
      vector<float> dists=config["distance zones"].as<vector<float>>();
      for (vector<float>::iterator it=dists.begin(); it!=dists.end(); it++)
      {
        distance_zones[*it]=-1;
      }
      int zone_count=0;
      for (map<float, int>::iterator it=distance_zones.begin(); it!=distance_zones.end(); it++)
      {
        distance_zones[it->first]=zone_count;
        zone_count++;
      }
      closest_dist_zone=distance_zones.size();

      int mult_const=distance_zones.size();
      for (auto const& [key, val] : situation_neigh_comp)
      {
        situation_neigh_comp[key].push_back(mult_const);
        mult_const*=(val[0]); // By end of loop mult_const is also total number of states

        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << pf_weights[key];
        std::string s=stream.str()+"_";
        transition_filename+=s;
      }
      std::stringstream stream;
      stream << std::fixed << std::setprecision(1) << goal_weight;
      std::string s=stream.str();
      transition_filename+=(s+".csv");
      total_situations=mult_const-(distance_zones.size()-1)+1; // Remove distance combos when no visible neighbours; add 1 for fail state
     // Set up transitions output file (if it does not exist); otherwise load current transition count
      printf("\t - Does file [%s] exist:  %i\n", (results_dir+"/"+transition_filename).data(), filesystem::exists(results_dir+"/"+transition_filename));
      if(!filesystem::exists(results_dir+"/"+transition_filename))
      {
        printf("\t - Creating new CSV file for transitions\n");
        outfile.open(results_dir+"/"+transition_filename);
       // Write column info
        outfile << ",";
        for(int i=0; i<total_situations-1; ++i) outfile<<"s"+to_string(i+1)+",";
        outfile << "s"+to_string(total_situations) << endl;
       // Create transition matrix, and write row info
        for(int i=0; i<total_situations; ++i)
        {
          // ,s1,s2,...,sn
          vector<int> transition_counts;
          outfile << "s"+to_string(i+1)+",";
          for(int j=0; j<total_situations-1; ++j)
          {
            transition_counts.push_back(0);
            outfile << "0,";
          }
          transition_counts.push_back(0);
          outfile << "0" << endl;
          state_trans_count.push_back(transition_counts);
        }
        outfile.close();
      }
      else
      {
        printf("\t - Loading CSV file for transitions\n");
        string output_file=results_dir+'/'+transition_filename;
        vector<pair<string, vector<string>>> transition_info=readCSV(output_file);
        vector<pair<string, vector<string>>> transition_info_no_labels(transition_info.begin()+1, transition_info.end());

        // for (int i=0; i<total_situations; ++i)
        // {
        //   vector<int> transition_counts;
        //   printf("\t - Value for row %s is: ", transition_info[0].second[i].data());
        //   for (int j=1; j<total_situations+1; ++j)
        //   {
        //     printf("%i  ",stoi(transition_info[j].second[i].data()));
        //   }
        //   printf("\n");
        // }

        for (int i=0; i<total_situations; ++i)
        {
          vector<int> transition_counts;
          for (int j=1; j<total_situations+1; ++j)
          {
            transition_counts.push_back(stoi(transition_info[j].second[i].data()));
          }
          state_trans_count.push_back(transition_counts);
        }
      }
    };
    void updateVel();
    void updateNeighPF(agent* neighbour);
    double getGoalDist(double pos_x, double pos_y);
    int situationID();
    void setPrevSituation();
    void resetSituation();
    void recordStep(int t);
    void updateSituation(agent* neighbour);
    void saveTransitionMatrix();
    bool modelCheck();
    float getCollisionThresh();

  private:
    double goal_weight;
    // double neigh_weight;
    double pf_x_neigh;
    double pf_y_neigh;

   // Potential field weights
    map<int, float> pf_weights;

   // Situtation description
    map<int, int> neighbour_types; // To keep track of number of neighbours w.r.t. agent type
    map<vector<int>, int> neigh_count_bins; // <type, number> -> bin
    map<int, vector<int>>  situation_neigh_comp; // <type, <bin count, multiplicative constant for calculating state>
    map<float, int> distance_zones; // Key is max distance of zone
    int closest_dist_zone;

    float collision_thresh;
    // map<vector<int>, int> situation;

   // Transition info
    int total_situations;
    vector<vector<int>> state_trans_count; // Raw counts of transitions
    vector<vector<float>> state_transitions; // Converted to probabilities
    int prev_situation;

   // Recording transition info
    string transition_filename;
};