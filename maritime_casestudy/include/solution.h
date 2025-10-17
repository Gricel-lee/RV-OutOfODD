#ifndef SOLUTION_H
#define SOLUTION_H
#include <map>
#include <vector>
#include <random>

using namespace std;
template<class Var, class Val>
struct solution
{
  // vector<Val> value_range;
  double lower_bound;
  double upper_bound;
  // map<Var*, Val> chromosome;
  vector<double> scores;
};
// template <typename Var,typename Val>
// class solution
// {
//   public:
//     solution();
//     solution(vector<Var*> chromosome_names, double _lower_bound, double _upper_bound)
//     {
//       lower_bound=_lower_bound;
//       upper_bound=_upper_bound;
//       uniform_real_distribution<double> value_distribution(lower_bound, upper_bound);
//       default_random_engine gen;
//       typename vector<Var*>::iterator it;
//       for(it=chromosome_names.begin(); it!=chromosome_names.end(); ++it)
//       {
//         double val=value_distribution(gen);
//         chromosome[it]=val;
//       }
//     };

//     void setScore(vector<double> _scores)
//     {
//       scores=_scores;
//     };

//   protected:
//     vector<Val> value_range;
//     double lower_bound;
//     double upper_bound;
//     map<Var*, Val> chromosome;
//     vector<double> scores;
// };

#endif