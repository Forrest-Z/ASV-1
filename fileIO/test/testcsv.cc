/*
***********************************************************************
* testcsv.cc:
* Utility test for the CSV file parser
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <map>
#include "csvstream.h"

int main() {
  // Open file
  csvstream csvin("../data/test.csv");

  // Rows have key = column name, value = cell datum
  std::map<std::string, std::string> row;

  std::vector<double> x;
  std::vector<double> y;
  int totalnum = 0;
  // Read file
  while (csvin >> row) {
    totalnum++;
    for (auto &col : row) {
      const std::string &column_name = col.first;
      const std::string &datum = col.second;
      double value = stod(datum);
      if (column_name == "X") x.push_back(value);
      if (column_name == "Y") y.push_back(value);
    }
  }
  std::cout << totalnum << std::endl;
}