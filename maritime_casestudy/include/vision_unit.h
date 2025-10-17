#ifndef VISION_H
#define VISION_H

#include <map>
#include <csv_functions.h>

using namespace std;

struct vessel_class_info 
{
  double length;
  double height;

  double res_640_480;
  double res_480_320;
  double res_320_240;
  double res_240_180;

  // distance{available classes{probability}}
  // map<string, map<string, double>> p_detection;
  map<string, map<string, double>> p_detection;

};

class VisionUnit
{
  public:
    VisionUnit(); // Empty constructor
    VisionUnit(string yaml_config_file);

    string predict(string true_class, double distance);

  private:
    // <vessel type, vessel info>
    map<string, vessel_class_info> vessel_info;

    
};

#endif