#ifndef GA_H
#define GA_H

#include <yaml-cpp/yaml.h>
#include<vector>
#include<map>
// #include <solution.h>

using namespace std;

template<class Var, class Val>
struct solution
{
  vector<Val> value_range;
  double lower_bound;
  double upper_bound;
  map<Var*, Val> chromosome;

  vector<double> fitness_vals;

 // For scoring solution
  double front;
  double crowding_dist;
};


template<class T, class T2>
class GeneticAlgorithm 
{
  public:
    GeneticAlgorithm();
    GeneticAlgorithm(string yaml_config_filename, vector<T*> params);
    
    // template<class T, class T2>
    void initialisePopulation(double lower_bound, double upper_bound);
    vector<solution<T, T2>> generateOffspring(bool cross_over=true);

  protected:
    string solution_path;
    string solutions_filename;
    int population_size;
    vector<T*> params;
    double p_mutation;
    double p_crossover;
    vector<int> objectives_sign;  
    vector<solution<T, T2>> population; 
};

#endif