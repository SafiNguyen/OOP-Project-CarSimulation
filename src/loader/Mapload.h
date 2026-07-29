
#ifndef MAPLOAD_H
#define MAPLOAD_H

#include <string>

class Graph;

namespace MapLoad {

bool loadGraphFromJsonFile(const std::string& filePath, Graph& graph, std::string* error);
bool loadGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error);

}

#endif
