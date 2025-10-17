#include <genetic_algorithm.h>
#include <random>

// GeneticAlgorithm::GeneticAlgorithm()
// {
//   // EMPTY CONSTRUCTOR
// }

using namespace std;

template<class T, class T2>
GeneticAlgorithm<T, T2>::GeneticAlgorithm(string yaml_config_filename, vector<T*> _params)
{
  YAML::Node config = YAML::LoadFile(yaml_config_filename);

  // Path to files
  string solution_path=config["solution directory"].as<string>();
  // GP parameters
  int population_size=config["population size"].as<int>();
  double p_mutation=config["mutation probability"].as<double>();
  double p_crossover=config["crossoverf probability"].as<double>();
  YAML::Node objectives=config["max/min objectives"];
  vector<int> sign=config["max/min objectives"].as<vector<int>>();
  solutions_filename=config["soultions filename"].as<string>();
  params=_params;

  // counter=setup['counter'] - perhaps not needed? Solutions diverse enough?
  //   self.all_solution_scores=np.zeros((2*self.population_size,2))
  //   self.fronts={}
  //   self.processes_no=setup['processes'] 
}

template<class T, class T2>
void GeneticAlgorithm<T, T2>::initialisePopulation(double lower_bound, double upper_bound)
{
  default_random_engine gen;
  uniform_real_distribution<double> range_distribution(lower_bound, upper_bound);
  while(population.size()<population_size)
  {
    solution<T, T2> new_solution;
    new_solution.chromosome=params;
    new_solution.lower_bound=lower_bound;
    new_solution.upper_bound=upper_bound;
    typename vector<T*>::iterator it;
    for(it=params.begin(); it!=params.end(); ++it)
    {
      double val=range_distribution(gen);
      new_solution.chromosome[it]=val;
    }
    population.push_back(new_solution);
  }
  return;
}

/*
  GA algorithm:
  1. Produce offspring using parents (population)
    1.1 Use tournament selection
    1.2 Cross over with winning parents
    1.3 Perform mutations on offspring
    1.4 From above 2 offspring produced  
  2. Get scores of parents and offspring
  3. Keep best from offspring and parents as new parents
    3.1 Non-dominated sorting
    3.2 Crowding distance for tie breaking
  4. Save parents as new parents
*/

template<class T, class T2>
vector<solution<T, T2>> GeneticAlgorithm<T, T2>::generateOffspring(bool cross_over)
{
  vector<solution<T, T2>> offspring;

  auto tournamentSelect=[&]() 
  { 
    default_random_engine gen;
    uniform_int_distribution<int> parent_select(0, population.size());
    solution<T,T2> candidate_1=population[parent_select(gen)];
    solution<T,T2> candidate_2=population[parent_select(gen)];
    if (candidate_1.score>candidate_2) return candidate_1;
    else if (candidate_1.score==candidate_2.score && candidate_1.crowding_dist>candidate_2.crowding_dist) return candidate_1;
    else return candidate_2;
  };
  
  auto crossOver=[](int start, int end, solution<T,T2> child, solution<T,T2> parent)
  {
    for(int i=start; i<end; ++i)
    {
      T* param=parent.keys()[i];
      child[param]=parent[param];
    }
    return child;
  };  

  auto mutate=[](solution<T,T2> child, double p_mutation)
  {
    default_random_engine gen;
    uniform_real_distribution<double> p(0, 1.0);
    uniform_real_distribution<double> noise_dist(-0.1, 0.1);
    
    for(typename vector<solution<T,T2>>::iterator it=child.begin(); it!=child.end(); ++it)
    {
      double noise=noise_dist(gen);
      if (p(gen)<p_mutation)
      {
        child[it]+=noise;
        child[it]=min(max(child.lower_bound), child.upper_bound);
      }
    }
    return child;
  };  


  default_random_engine gen;
  uniform_real_distribution<double> p(0, 1);

  while(offspring.size()<population.size())
  {
    solution<T,T2> parent_1=tournamentSelect();
    solution<T,T2> parent_2=tournamentSelect();

    solution<T,T2> child_1=parent_1;
    solution<T,T2> child_2=parent_2;

    if (cross_over && p(gen)<p_crossover)
    {
      uniform_int_distribution<int> chromosome_cut(0, params.size());      
      int point=chromosome_cut(gen);
      child_1=crossOver(0, point, parent_2);
      child_2=crossOver(point, params.size(), parent_1);
    }

    child_1=mutate(child_1, p_mutation);
    child_2=mutate(child_2, p_mutation);

    offspring.push_back(child_1);
    offspring.push_back(child_2);
  }

  return offspring;
}

// template<class T, class T2>
// void GeneticAlgorithm<T, T2>::evolvePopulation(bool crossover)
// {
//  // Create empty vector to populate
//  // Select parents
//   return;
// }

// template<class T, class T2>
// void GeneticAlgorithm<T, T2>::setFitness(vector<double> _fitness_vals)
// {
//   return;
// }

// template<class T, class T2>
// void GeneticAlgorithm<T, T2>::getScores()
// {
//   for (vector<solution<T,T2>>::iterator it=population.begin(); it!=population.end(); it++)
//   {
//     sortNonDominated();
//   }

//   return;
// }

// void GeneticAlgorithm::dominates()
// {
//   return;
// }

template<class T, class T2>
void GeneticAlgorithm<T,T2>::sortNondominated()
{
  vector<vector<solution<T,T2>*>> front;
    // def sortNondominated(self, N, k=None, parents_only=False, first_front_only=False):
    //     fitness=[]
    //     N=1*self.population_size if parents_only else 2*self.population_size
    //     for solution in self.fitness_vals:
    //         vals=self.fitness_vals[solution]
    //         fitness.append((vals[0],vals[1],vals[2]))
        
    //     if k is None:
    //         k = len(fitness)

    //     # Use objectives as keys to make python dictionary
    //     map_fit_ind = defaultdict(list)
    //     for i, f_value in enumerate(fitness):  # fitness = [(1, 2), (2, 2), (3, 1), (1, 4), (1, 1)...]
    //         map_fit_ind[f_value].append(i)
    //     fits = list(map_fit_ind.keys())  # fitness values

    //     current_front = []
    //     next_front = []
    //     dominating_fits = defaultdict(int)  # n (The number of people dominate you)
    //     dominated_fits = defaultdict(list)  # Sp (The people you dominate)

    //     # Rank first Pareto front
    //     # *fits* is a iterable list of chromosomes. Each has multiple objectives.
    //     for i, fit_i in enumerate(fits):
    //         # print('\n\nValue to test: ', i, fit_i)
    //         for fit_j in fits[i + 1:]:
    //             # print('----Compared against: ',fit_j)
    //             # Eventhough equals or empty list, n & Sp won't be affected
    //             # print('----Outcome i<j: ', self.dominates(fit_i, fit_j, sign=[-1,1]))
    //             # print('----Outcome j<i: ',self.dominates(fit_j, fit_i, sign=[-1,1]))
    //             if self.dominates(fit_i, fit_j, sign=self.sign):#[-1,-1,1]):
    //                 dominating_fits[fit_j] += 1  
    //                 dominated_fits[fit_i].append(fit_j)
    //             elif self.dominates(fit_j, fit_i, sign=self.sign):#[-1,-1,1]):  
    //                 dominating_fits[fit_i] += 1
    //                 dominated_fits[fit_j].append(fit_i)
    //         if dominating_fits[fit_i] == 0: 
    //             current_front.append(fit_i)
    //     fronts = [[]]  # The first front
    //     for fit in current_front:
    //         fronts[-1].extend(map_fit_ind[fit])
    //     pareto_sorted = len(fronts[-1])

    //     # Rank the next front until all individuals are sorted or
    //     # the given number of individual are sorted.
    //     # If Sn=0 then the set of objectives belongs to the next front
    //     if not first_front_only:  # first front only
    //         N = min(len(fitness), k)
    //         while pareto_sorted < N:
    //             fronts.append([])
    //             for fit_p in current_front:
    //                 # Iterate Sn in current fronts
    //                 for fit_d in dominated_fits[fit_p]: 
    //                     dominating_fits[fit_d] -= 1  # Next front -> Sn - 1
    //                     if dominating_fits[fit_d] == 0:  # Sn=0 -> next front
    //                         next_front.append(fit_d)
    //                          # Count and append chromosomes with same objectives
    //                         pareto_sorted += len(map_fit_ind[fit_d]) 
    //                         fronts[-1].extend(map_fit_ind[fit_d])
    //             current_front = next_front
    //             next_front = []

    //     for front in np.arange(len(fronts)):
    //         # print(fronts[front])
    //         for solution in fronts[front]:
    //             self.all_solution_scores[solution][0]=front
    //     # print(fronts)
    //     self.fronts=fronts
  return;
}

// void GeneticAlgorithm::crowdingDist()
// {
//   return;
// }