#ifndef CSV_H
#define CSV_H
#include <string_view>
#include <vector>

using namespace std;

// vector<vector<string_view>> readCSVOld(const string& filename);
// vector<string_view> parseCSVRow(string_view row);

// template<typename T>
vector<pair<string, vector<string>>> readCSV(string filename);

#endif