
#include "Mapload.h"
	
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_set>

#include <nlohmann/json.hpp>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Roundabout.h"
#include "model/Bridge.h"
#include "model/Tunnel.h"
#include "model/PointOfInterest.h"
#include "model/SpawnPoint.h"
#include "model/Destination.h"
#include "model/BusStop.h"

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

		// Check for intersection type (e.g. "roundabout")
		std::string intersectionType;
		getStringOptional(item, "type", intersectionType, localError);

		if (intersectionType == "roundabout") {
			double radius = 0.02;
			getDoubleOptional(item, "radius", radius, localError);
			graph.addIntersection(new Roundabout(id, x, y, radius));
		} else {
			graph.addIntersection(new Intersection(id, x, y));
		}
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
		const double distanceMeters = distanceKm * 20.0;

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

		std::string roadType;
		std::string originalRoadName;
		getStringOptional(item, "type", roadType, localError);
		getStringOptional(item, "name", originalRoadName, localError);

		std::string roadName = originalRoadName;
		if (roadName.empty()) {
			roadName = "Road " + std::to_string(startId) + " -> " + std::to_string(endId);
		}

		Road* road = nullptr;
		if (roadType == "bridge") {
			double heightLimit = 4.5;
			double weightLimit = 30.0;
			getDoubleOptional(item, "heightLimit", heightLimit, localError);
			getDoubleOptional(item, "weightLimit", weightLimit, localError);
			road = new Bridge(id, roadName, start, end, distanceMeters, speedLimit,
			                  congestionLevel, lanes, heightLimit, weightLimit);
		} else if (roadType == "tunnel") {
			double heightLimit = 3.5;
			getDoubleOptional(item, "heightLimit", heightLimit, localError);
			road = new Tunnel(id, roadName, start, end, distanceMeters, speedLimit,
			                  congestionLevel, lanes, heightLimit);
		} else {
			road = new Road(id, roadName, start, end, distanceMeters, speedLimit, congestionLevel, lanes);
		}
		if (blocked) {
			road->blockRoad();
		}
		graph.addRoad(road);

		if (twoWay) {
			// create reverse road with negative id
			Road* revRoad = nullptr;
			std::string revRoadName = originalRoadName;
			if (revRoadName.empty()) {
				revRoadName = "Road " + std::to_string(endId) + " -> " + std::to_string(startId);
			}
			if (roadType == "bridge") {
				double heightLimit = 4.5, weightLimit = 30.0;
				getDoubleOptional(item, "heightLimit", heightLimit, localError);
				getDoubleOptional(item, "weightLimit", weightLimit, localError);
				revRoad = new Bridge(-id, revRoadName, end, start, distanceMeters, speedLimit,
				                     congestionLevel, lanes, heightLimit, weightLimit);
			} else if (roadType == "tunnel") {
				double heightLimit = 3.5;
				getDoubleOptional(item, "heightLimit", heightLimit, localError);
				revRoad = new Tunnel(-id, revRoadName, end, start, distanceMeters, speedLimit,
				                     congestionLevel, lanes, heightLimit);
			} else {
				revRoad = new Road(-id, revRoadName, end, start, distanceMeters, speedLimit, congestionLevel, lanes);
			}
			if (blocked) revRoad->blockRoad();
			graph.addRoad(revRoad);
		}
	}

	// --- Parse bus stops (optional section) ---
	if (root.contains("busStops")) {
		if (!root.at("busStops").is_array()) {
			if (error) {
				*error = "Invalid 'busStops': expected an array.";
			}
			return false;
		}

		std::unordered_set<int> busStopIds;
		const auto& busStopItems = root.at("busStops");
		for (std::size_t index = 0; index < busStopItems.size(); ++index) {
			const auto& item = busStopItems.at(index);
			const std::string context = "Bus stop at index " + std::to_string(index);
			if (!item.is_object()) {
				if (error) {
					*error = context + ": expected a JSON object.";
				}
				return false;
			}

			int stopId = 0;
			int roadId = 0;
			int lane = 0;
			double positionRatio = 0.0;
			double dwellTime = BusStop::DEFAULT_DWELL_TIME;
			std::string name;
			std::string localError;

			if (!getInt(item, "id", stopId, localError)) {
				if (error) *error = context + ": " + localError;
				return false;
			}
			const std::string stopContext = "Bus stop " + std::to_string(stopId)
			                              + " at index " + std::to_string(index);

			if (!getInt(item, "roadId", roadId, localError) ||
			    !getDouble(item, "positionRatio", positionRatio, localError)) {
				if (error) *error = stopContext + ": " + localError;
				return false;
			}
			if (!getIntOptional(item, "lane", lane, localError) ||
			    !getDoubleOptional(item, "dwellTime", dwellTime, localError) ||
			    !getStringOptional(item, "name", name, localError)) {
				if (error) *error = stopContext + ": " + localError;
				return false;
			}

			if (!busStopIds.insert(stopId).second) {
				if (error) *error = stopContext + ": duplicate bus stop id.";
				return false;
			}

			Road* road = graph.getRoad(roadId);
			if (road == nullptr) {
				if (error) {
					*error = stopContext + ": roadId " + std::to_string(roadId)
					       + " does not exist.";
				}
				return false;
			}

			if (!std::isfinite(positionRatio) ||
			    positionRatio <= 0.0 ||
			    positionRatio >= 1.0) {
				if (error) {
					*error = stopContext
					       + ": positionRatio must be finite and strictly between 0 and 1.";
				}
				return false;
			}
			if (lane < 0 || lane >= road->getLaneCount()) {
				if (error) {
					*error = stopContext + ": lane " + std::to_string(lane)
					       + " is outside [0, "
					       + std::to_string(road->getLaneCount() - 1) + "].";
				}
				return false;
			}
			if (!std::isfinite(dwellTime) || dwellTime < 0.0) {
				if (error) {
					*error = stopContext + ": dwellTime must be finite and non-negative.";
				}
				return false;
			}

			const double positionOnRoad = positionRatio * road->getDistance();
			for (const auto& existing : road->getBusStops()) {
				if (std::fabs(existing->getPositionOnRoad() - positionOnRoad)
				    < BusStop::MIN_SPACING) {
					if (error) {
						*error = stopContext + ": positionRatio places it within "
						       + std::to_string(BusStop::MIN_SPACING)
						       + " of bus stop " + std::to_string(existing->getId())
						       + " on road " + std::to_string(roadId) + ".";
					}
					return false;
				}
			}

			if (name.empty()) {
				name = "Bus Stop " + std::to_string(stopId);
			}
			const bool hasConfiguredDwellTime = item.contains("dwellTime");
			auto busStop = std::make_unique<BusStop>(
				stopId,
				name,
				road,
				positionOnRoad,
				lane,
				dwellTime,
				hasConfiguredDwellTime);
			if (!road->addBusStop(std::move(busStop))) {
				if (error) {
					*error = stopContext + ": could not be added to road "
					       + std::to_string(roadId) + ".";
				}
				return false;
			}
		}
	}

	// --- Parse POIs (optional section) ---
	if (root.contains("pois") && root.at("pois").is_array()) {
		for (const auto& item : root.at("pois")) {
			if (!item.is_object()) continue;

			int poiId = 0;
			double px = 0.0, py = 0.0;
			std::string poiName, poiTypeStr;
			int nearestId = -1;
			std::string localError;

			if (!getInt(item, "id", poiId, localError)) continue;
			getDoubleOptional(item, "x", px, localError);
			getDoubleOptional(item, "y", py, localError);
			getStringOptional(item, "name", poiName, localError);
			getStringOptional(item, "type", poiTypeStr, localError);
			getIntOptional(item, "nearestIntersection", nearestId, localError);

			Intersection* nearest = (nearestId >= 0) ? graph.getIntersection(nearestId) : nullptr;
			POIType poiType = PointOfInterest::typeFromString(poiTypeStr);

			PointOfInterest* poi = nullptr;
			switch (poiType) {
				// Spawn points
				case POIType::PARKING_LOT:
					poi = new ParkingLot(poiId, poiName, px, py, nearest);
					break;
				case POIType::BUS_STATION:
					poi = new BusStation(poiId, poiName, px, py, nearest);
					break;
				case POIType::HOSPITAL:
					poi = new HospitalSpawn(poiId, poiName, px, py, nearest);
					break;
				// Destinations
				case POIType::RESTAURANT:
					poi = new Restaurant(poiId, poiName, px, py, nearest);
					break;
				case POIType::CINEMA:
					poi = new Cinema(poiId, poiName, px, py, nearest);
					break;
				case POIType::SUPERMARKET:
					poi = new Supermarket(poiId, poiName, px, py, nearest);
					break;
				case POIType::TOURIST_SPOT:
					poi = new TouristSpot(poiId, poiName, px, py, nearest);
					break;
				default:
					poi = new PointOfInterest(poiId, poiName, poiType, px, py, nearest);
					break;
			}
			graph.addPOI(poi);
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
