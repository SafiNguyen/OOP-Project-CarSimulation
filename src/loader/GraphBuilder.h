#ifndef GRAPH_BUILDER_H
#define GRAPH_BUILDER_H

#include <string>
#include <nlohmann/json.hpp>
class Graph;

namespace GraphBuilder {
    bool buildGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error);
}

#endif // GRAPH_BUILDER_H
