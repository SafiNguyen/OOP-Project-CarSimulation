
#include "Mapload.h"
	
#include <cctype>
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

bool getStringOptional(const json& obj, const char* key, std::string& value, std::string& error) {
	if (!obj.contains(key)) {
		return true;
	}
	if (!obj.at(key).is_string()) {
		error = std::string("Field must be string: ") + key;
		return false;
	}
	value = obj.at(key).get<std::string>();
	return true;
}

// Road::getDistance()/getTravelTime() and Road::getSpeedLimit() are meant
// to be read together as kilometers / (kilometers-per-hour), so internally
// every road's distance is stored in kilometers. Map authors, however,
// often find it far more natural to lay out a city block in meters (e.g.
// "distance": 120 for a ~120m street) rather than fractional kilometers.
// To support both without breaking any existing map file, an optional
// "distanceUnit" string (per-road, or "defaultDistanceUnit" at the map
// root as a fallback for every road that doesn't override it) selects
// how the raw "distance" number should be interpreted before it's
// converted to kilometers. If no unit is specified anywhere, "km" is
// assumed - i.e. the exact previous behaviour, so old map files (whose
// "distance" values were already being used as-is) load identically.
bool distanceUnitToKmFactor(const std::string& unit, double& factor, std::string& error) {
	std::string normalized = unit;
	for (char& c : normalized) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}

	if (normalized == "km" || normalized == "kilometer" || normalized == "kilometers" || normalized.empty()) {
		factor = 1.0;
	} else if (normalized == "m" || normalized == "meter" || normalized == "meters") {
		factor = 0.001;
	} else if (normalized == "mi" || normalized == "mile" || normalized == "miles") {
		factor = 1.609344;
	} else {
		error = "Unknown distanceUnit: " + unit + " (expected 'km', 'm', or 'mi')";
		return false;
	}
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

	//root-level fallback unit applied to every road that doesn't
	// specify its own "distanceUnit". Defaults to "km" (no-op conversion),
	// so maps written before this feature existed are unaffected.
	std::string defaultDistanceUnit = "km";
	{
		std::string rootUnitError;
		if (!getStringOptional(root, "defaultDistanceUnit", defaultDistanceUnit, rootUnitError)) {
			if (error) {
				*error = rootUnitError;
			}
			return false;
		}
	}

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

		// interpret "distance" using this road's own unit if given,
		// otherwise the map-wide default, and normalize to kilometers -
		// the unit Road/Vehicle travel-time math is written in terms of.
		std::string distanceUnit = defaultDistanceUnit;
		if (!getStringOptional(item, "distanceUnit", distanceUnit, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		double distanceUnitFactor = 1.0;
		if (!distanceUnitToKmFactor(distanceUnit, distanceUnitFactor, localError)) {
			if (error) {
				*error = "Road " + std::to_string(id) + ": " + localError;
			}
			return false;
		}
		const double distanceKm = distance * distanceUnitFactor;

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

		Road* road = new Road(id, start, end, distanceKm, speedLimit, congestionLevel, lanes);
		if (blocked) {
			road->blockRoad();
		}
		graph.addRoad(road);

		if (twoWay) {
			// create reverse road with negative id (or offset)
			Road* revRoad = new Road(-id, end, start, distanceKm, speedLimit, congestionLevel, lanes);
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
