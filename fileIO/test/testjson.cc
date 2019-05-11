/*
***********************************************************************
* testjson.cpp:
* Utility test for the JSON
* This header file can be read by C++ compilers
*
* by Hu.ZH(CrossOcean.ai)
***********************************************************************
*/

#include "jsonparse.h"

int main() {
  // read a JSON file
  jsonparse<3, 3> _jsonparse("../data/test.json");
  _jsonparse.tets();
  std::cout << _jsonparse << std::endl;
  // json file = json::parse(in);

  // std::vector<int> _jsonarray = file["list"];
  // std::cout << file["pi"] << std::endl;
  // std::cout << file["object"]["currency"] << std::endl;
  // std::cout << _jsonarray[0] << std::endl;

  // // iterate the array
  // for (json::iterator it = file.begin(); it != file.end(); ++it) {
  //   std::cout << *it << '\n';
  // }
  // std::ofstream o("../data/pretty.json");
  // o << std::setw(4) << file << std::endl;
}