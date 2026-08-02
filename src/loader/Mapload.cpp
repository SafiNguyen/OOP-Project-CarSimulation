#include "Mapload.h"
#include "GraphBuilder.h"
#include <fstream>
#include <sstream>

namespace MapLoad {

bool loadGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error) {
    return GraphBuilder::buildGraphFromJsonString(jsonText, graph, error);
}

bool loadGraphFromJsonFile(const std::string& filePath, Graph& graph, std::string* error) {
	std::ifstream file(filePath);
	if (!file.is_open()) {
		if (error) {
			*error = "Failed to open file: " + filePath;
		}
		return false;
	}

	std::stringstream buffer;
	buffer << file.rdbuf();
	return loadGraphFromJsonString(buffer.str(), graph, error);
}

} // namespace MapLoad
