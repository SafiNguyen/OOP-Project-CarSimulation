
#include "mapload.h"

#include <fstream>
#include <sstream>

#include <nlohmann/json.hpp>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"

using nlohmann::json;

namespace {

bool getInt(const json& obj, const char* key, int& value, std::string& error) {
	if (!obj.contains(key)) {
		error = std::string("Missing field: ") + key;
		return false;
	}
	if (!obj.at(key).is_number_integer()) {
		error = std::string("Field must be integer: ") + key;
		return false;
	}
	value = obj.at(key).get<int>();
	return true;
}

bool getDouble(const json& obj, const char* key, double& value, std::string& error) {
	if (!obj.contains(key)) {
		error = std::string("Missing field: ") + key;
		return false;
	}
	if (!obj.at(key).is_number()) {
		error = std::string("Field must be number: ") + key;
		return false;
	}
	value = obj.at(key).get<double>();
	return true;
}

bool getIntOptional(const json& obj, const char* key, int& value, std::string& error) {
	if (!obj.contains(key)) {
		return true;
	}
	if (!obj.at(key).is_number_integer()) {
		error = std::string("Field must be integer: ") + key;
		return false;
	}
	value = obj.at(key).get<int>();
	return true;
}

bool getBoolOptional(const json& obj, const char* key, bool& value, std::string& error) {
	if (!obj.contains(key)) {
		return true;
	}
	if (!obj.at(key).is_boolean()) {
		error = std::string("Field must be boolean: ") + key;
		return false;
	}
	value = obj.at(key).get<bool>();
	return true;
}

bool getDoubleOptional(const json& obj, const char* key, double& value, std::string& error) {
	if (!obj.contains(key)) {
		return true;
	}
	if (!obj.at(key).is_number()) {
		error = std::string("Field must be number: ") + key;
		return false;
	}
	value = obj.at(key).get<double>();
	return true;
}

} 

namespace MapLoad {

bool loadGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error) {
	json root;
	try {
		root = json::parse(jsonText);
	} catch (const json::parse_error& ex) {
		if (error) {
			*error = std::string("JSON parse error: ") + ex.what();
		}
		return false;
	}

	if (!root.is_object()) {
		if (error) {
			*error = "Root must be a JSON object.";
		}
		return false;
	}

	if (!root.contains("intersections") || !root.at("intersections").is_array()) {
		if (error) {
			*error = "Missing or invalid 'intersections' array.";
		}
		return false;
	}

	if (!root.contains("roads") || !root.at("roads").is_array()) {
		if (error) {
			*error = "Missing or invalid 'roads' array.";
		}
		return false;
	}

	graph.clearGraph();

	for (const auto& item : root.at("intersections")) {
		if (!item.is_object()) {
			if (error) {
				*error = "Each intersection must be a JSON object.";
			}
			return false;
		}

		int id = 0;
		double x = 0.0;
		double y = 0.0;
		std::string localError;

		if (!getInt(item, "id", id, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		if (item.contains("x")) {
			if (!getDouble(item, "x", x, localError)) {
				if (error) {
					*error = localError;
				}
				return false;
			}
		}

		if (item.contains("y")) {
			if (!getDouble(item, "y", y, localError)) {
				if (error) {
					*error = localError;
				}
				return false;
			}
		}

		if (graph.getIntersection(id) != nullptr) {
			if (error) {
				*error = "Duplicate intersection id: " + std::to_string(id);
			}
			return false;
		}

		graph.addIntersection(new Intersection(id, x, y));
	}

	for (const auto& item : root.at("roads")) {
		if (!item.is_object()) {
			if (error) {
				*error = "Each road must be a JSON object.";
			}
			return false;
		}

		int id = 0;
		int startId = 0;
		int endId = 0;
		double distance = 0.0;
		double speedLimit = 0.0;
		double congestionLevel = 1.0;
		int lanes = 1;
		bool twoWay = false;
		bool blocked = false;
		std::string localError;

		if (!getInt(item, "id", id, localError) ||
			!getInt(item, "start", startId, localError) ||
			!getInt(item, "end", endId, localError) ||
			!getDouble(item, "distance", distance, localError) ||
			!getDouble(item, "speedLimit", speedLimit, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		if (!getDoubleOptional(item, "congestionLevel", congestionLevel, localError) ||
			!getIntOptional(item, "lanes", lanes, localError) ||
			!getBoolOptional(item, "twoWay", twoWay, localError) ||
			!getBoolOptional(item, "blocked", blocked, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		if (graph.getRoad(id) != nullptr) {
			if (error) {
				*error = "Duplicate road id: " + std::to_string(id);
			}
			return false;
		}

		Intersection* start = graph.getIntersection(startId);
		Intersection* end = graph.getIntersection(endId);
		if (start == nullptr || end == nullptr) {
			if (error) {
				*error = "Road references missing intersection id.";
			}
			return false;
		}

		Road* road = new Road(id, start, end, distance, speedLimit, congestionLevel, lanes);
		if (blocked) {
			road->blockRoad();
		}
		graph.addRoad(road);

		if (twoWay) {
			// create reverse road with negative id (or offset)
			Road* revRoad = new Road(-id, end, start, distance, speedLimit, congestionLevel, lanes);
			if (blocked) revRoad->blockRoad();
			graph.addRoad(revRoad);
		}
	}

	return true;
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

} 
