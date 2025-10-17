#include <csv_functions.h>
#include <iostream>
#include <fstream>
#include <utility> // pair
#include <stdexcept> // runtime_error
#include <sstream> // stringstream

using namespace std;

// template<typename T>
vector<pair<string, vector<string>>> readCSV(string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    vector<pair<string, vector<string>>> result;

    // Create an input filestream
    ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw runtime_error("Could not open file");

    // Helper vars
    string line, colname;
    string val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        getline(myFile, line);

        // Create a stringstream from line
        stringstream ss(line);

        // Extract each column name
        while(getline(ss, colname, ',')){
            
            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, vector<string> {}});
        }
    }

    // Read data, line by line
    while(getline(myFile, line))
    {    
        // Keep track of the current column index
        int colIdx=0;
        size_t check;//=line.find(',');
        // cout << line << endl;
        while(line.size()>0)
        {   
            check=line.find(',');
            if (check!=string::npos) val=line.substr(0,check), line=line.substr(check+1,-1);
            else val=line, line="";
            // cout << val << endl;
            result.at(colIdx).second.push_back(val);
            // printf("CHECK:  %s\n", result[0].second[-1]);
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}