#ifndef MAPLOAD_H
#define MAPLOAD_H

#include <string>
class Graph;

namespace MapLoad {
    bool loadGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error = nullptr);
    bool loadGraphFromJsonFile(const std::string& filename, Graph& graph, std::string* error = nullptr);
}

#endif // MAPLOAD_H
