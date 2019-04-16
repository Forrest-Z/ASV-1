/*
***********************************************************************
* testjson.cpp:
* Utility test for the JSON
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include <fstream>
#include <iomanip>
#include <iostream>
#include <json.hpp>
#include <vector>

using nlohmann::json;

int main() {
  // read a JSON file
  std::ifstream in("../data/test.json");
  // json file = json::parse(in);
  json file;
  in >> file;
  std::vector<int> _jsonarray = file["list"];
  std::cout << file["pi"] << std::endl;
  std::cout << file["object"]["currency"] << std::endl;
  std::cout << _jsonarray[0] << std::endl;

  // iterate the array
  for (json::iterator it = file.begin(); it != file.end(); ++it) {
    std::cout << *it << '\n';
  }
  std::ofstream o("../data/pretty.json");
  o << std::setw(4) << file << std::endl;
}