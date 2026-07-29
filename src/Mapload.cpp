#include "Mapload.h"
	
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/RoadGeometry.h"
#include "model/Roundabout.h"
#include "model/Bridge.h"
#include "model/Tunnel.h"
#include "model/PointOfInterest.h"
#include "model/SpawnPoint.h"
#include "model/Destination.h"
#include "model/BusStop.h"
#include "model/Crosswalk.h"
#include "common/Units.h"

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

bool distanceUnitToMetresFactor(
	const std::string& unit,
	double& factor,
	std::string& error) {
	std::string normalized = unit;
	for (char& c : normalized) {
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}

	if (normalized == "legacy" || normalized == "legacy-map-unit" ||
		normalized.empty()) {
		// Historical maps used distance * 20. Keeping this named adapter
		// preserves their travel times while all downstream physics sees SI.
		factor = 20.0;
	} else if (normalized == "km" || normalized == "kilometer" ||
			   normalized == "kilometers") {
		factor = Units::KM_TO_M_FACTOR;
	} else if (normalized == "m" || normalized == "meter" || normalized == "meters") {
		factor = 1.0;
	} else if (normalized == "mi" || normalized == "mile" || normalized == "miles") {
		factor = 1609.344;
	} else {
		error = "Unknown distanceUnit: " + unit +
			" (expected 'legacy', 'km', 'm', or 'mi')";
		return false;
	}
	return true;
}

bool speedUnitToMetresPerSecondFactor(
	const std::string& unit,
	double& factor,
	std::string& error) {
	std::string normalized = unit;
	for (char& c : normalized) {
		c = static_cast<char>(
			std::tolower(static_cast<unsigned char>(c)));
	}
	if (normalized == "km/h" || normalized == "kmh" ||
		normalized == "kph" || normalized.empty()) {
		factor = 1.0 / Units::KMH_TO_MPS_FACTOR;
	} else if (normalized == "m/s" || normalized == "mps") {
		factor = 1.0;
	} else {
		error = "Unknown speedUnit: " + unit +
			" (expected 'km/h' or 'm/s')";
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

	// Explicit boundary contract: the loader normalizes every physical value
	// to SI. The named legacy distance scale keeps pre-schema maps compatible.
	std::string defaultDistanceUnit = "legacy";
	std::string defaultSpeedUnit = "km/h";
	std::string defaultRadiusUnit = "km";
	double defaultLaneWidth = RoadGeometry::LANE_WIDTH_METRES;
	{
		std::string rootUnitError;
		if (!getStringOptional(root, "defaultDistanceUnit", defaultDistanceUnit, rootUnitError) ||
			!getStringOptional(root, "defaultSpeedUnit", defaultSpeedUnit, rootUnitError) ||
			!getStringOptional(root, "defaultRadiusUnit", defaultRadiusUnit, rootUnitError) ||
			!getDoubleOptional(root, "defaultLaneWidth", defaultLaneWidth, rootUnitError)) {
			if (error) {
				*error = rootUnitError;
			}
			return false;
		}
		if (!std::isfinite(defaultLaneWidth) ||
			defaultLaneWidth <= 0.0) {
			if (error) {
				*error =
					"defaultLaneWidth must be finite and positive.";
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
			std::string radiusUnit = defaultRadiusUnit;
			if (!getDoubleOptional(item, "radius", radius, localError) ||
				!getStringOptional(item, "radiusUnit", radiusUnit, localError)) {
				if (error) *error = localError;
				return false;
			}
			double radiusFactor = 1.0;
			if (!distanceUnitToMetresFactor(
					radiusUnit, radiusFactor, localError) ||
				!std::isfinite(radius) || radius <= 0.0) {
				if (error) {
					*error = !localError.empty()
						? localError
						: "Roundabout radius must be finite and positive.";
				}
				return false;
			}
			graph.addIntersection(new Roundabout(
				id, x, y, radius * radiusFactor));
		} else {
			graph.addIntersection(new Intersection(id, x, y));
		}
	}

	// Tracks every directed (start, end) pair already created by an earlier
	// road entry (including the reverse leg of an earlier twoWay entry), so
	// a second entry can't silently stack a duplicate road/reverse-road on
	// top of it. Without this, two JSON entries describing the same street
	// from opposite ends (e.g. one "1 -> 4, twoWay" and another
	// "4 -> 1, twoWay") each spawn their own forward+reverse pair, leaving
	// 4 roads stacked on the same two intersections instead of 2 - which is
	// what caused two-way roads to render as a single merged line.
	std::set<std::pair<int, int>> seenDirectedPairs;

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
		double laneWidth = defaultLaneWidth;
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
			!getDoubleOptional(item, "laneWidth", laneWidth, localError) ||
			!getIntOptional(item, "lanes", lanes, localError) ||
			!getBoolOptional(item, "twoWay", twoWay, localError) ||
			!getBoolOptional(item, "blocked", blocked, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		// Interpret boundary units once and normalize Road to metres and m/s.
		std::string distanceUnit = defaultDistanceUnit;
		if (!getStringOptional(item, "distanceUnit", distanceUnit, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		double distanceUnitFactor = 1.0;
		if (!distanceUnitToMetresFactor(distanceUnit, distanceUnitFactor, localError)) {
			if (error) {
				*error = "Road " + std::to_string(id) + ": " + localError;
			}
			return false;
		}
		const double distanceMeters = distance * distanceUnitFactor;
		std::string speedUnit = defaultSpeedUnit;
		if (!getStringOptional(item, "speedUnit", speedUnit, localError)) {
			if (error) *error = localError;
			return false;
		}
		double speedFactor = 1.0;
		if (!speedUnitToMetresPerSecondFactor(
				speedUnit, speedFactor, localError)) {
			if (error) {
				*error = "Road " + std::to_string(id) + ": " + localError;
			}
			return false;
		}
		const double speedMetresPerSecond = speedLimit * speedFactor;
		if (!std::isfinite(distanceMeters) || distanceMeters <= 0.0 ||
			!std::isfinite(speedMetresPerSecond) ||
			speedMetresPerSecond <= 0.0 ||
			!std::isfinite(laneWidth) || laneWidth <= 0.0 ||
			lanes <= 0 ||
			!std::isfinite(congestionLevel) ||
			congestionLevel < 1.0) {
			if (error) {
				*error = "Road " + std::to_string(id) +
					": distance, speedLimit, laneWidth and lanes must be "
					"positive; congestionLevel must be finite and at least 1.";
			}
			return false;
		}

		if (graph.getRoad(id) != nullptr) {
			if (error) {
				*error = "Duplicate road id: " + std::to_string(id);
			}
			return false;
		}
		if (twoWay &&
			(id == 0 || graph.getRoad(-id) != nullptr)) {
			if (error) {
				*error = "Road " + std::to_string(id) +
					": its generated reverse id is zero or already in use.";
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

		// Reject a road entry that duplicates a directed pair already
		// created (either as another entry's forward leg, or as the
		// implicit reverse leg of an earlier twoWay entry). See comment
		// on seenDirectedPairs above for why this matters.
		const auto forwardPair = std::make_pair(startId, endId);
		if (seenDirectedPairs.count(forwardPair)) {
			if (error) {
				*error = "Road " + std::to_string(id) + ": duplicate road " +
					std::to_string(startId) + " -> " + std::to_string(endId) +
					" (already created by an earlier entry).";
			}
			return false;
		}
		if (twoWay) {
			const auto reversePair = std::make_pair(endId, startId);
			if (seenDirectedPairs.count(reversePair)) {
				if (error) {
					*error = "Road " + std::to_string(id) +
						": duplicate reverse road " + std::to_string(endId) +
						" -> " + std::to_string(startId) +
						" (already created by an earlier entry).";
				}
				return false;
			}
			seenDirectedPairs.insert(reversePair);
		}
		seenDirectedPairs.insert(forwardPair);

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
			road = new Bridge(id, roadName, start, end, distanceMeters, speedMetresPerSecond,
			                  congestionLevel, lanes, heightLimit, weightLimit,
			                  laneWidth);
		} else if (roadType == "tunnel") {
			double heightLimit = 3.5;
			getDoubleOptional(item, "heightLimit", heightLimit, localError);
			road = new Tunnel(id, roadName, start, end, distanceMeters, speedMetresPerSecond,
			                  congestionLevel, lanes, heightLimit, laneWidth);
		} else {
			road = new Road(id, roadName, start, end, distanceMeters,
				speedMetresPerSecond, congestionLevel, lanes, laneWidth);
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
				revRoad = new Bridge(-id, revRoadName, end, start, distanceMeters, speedMetresPerSecond,
				                     congestionLevel, lanes, heightLimit, weightLimit,
				                     laneWidth);
			} else if (roadType == "tunnel") {
				double heightLimit = 3.5;
				getDoubleOptional(item, "heightLimit", heightLimit, localError);
				revRoad = new Tunnel(-id, revRoadName, end, start, distanceMeters, speedMetresPerSecond,
				                     congestionLevel, lanes, heightLimit, laneWidth);
			} else {
				revRoad = new Road(-id, revRoadName, end, start, distanceMeters,
					speedMetresPerSecond, congestionLevel, lanes, laneWidth);
			}
			if (blocked) revRoad->blockRoad();
			graph.addRoad(revRoad);
		}
	}

	// --- Parse explicit intersection signal plans (optional/backward-compatible) ---
	if (root.contains("trafficLights")) {
		if (!root.at("trafficLights").is_array()) {
			if (error) {
				*error = "Invalid 'trafficLights': expected an array.";
			}
			return false;
		}

		std::unordered_set<int> configuredIntersections;
		const auto& signalItems = root.at("trafficLights");
		for (std::size_t index = 0;
			 index < signalItems.size();
			 ++index) {
			const auto& item = signalItems.at(index);
			const std::string context =
				"Traffic light config at index " +
				std::to_string(index);
			if (!item.is_object()) {
				if (error) {
					*error = context + ": expected a JSON object.";
				}
				return false;
			}

			int intersectionId = 0;
			bool enabled = true;
			double greenDuration = 25.0;
			double yellowDuration = 3.0;
			double allRedDuration = 1.5;
			std::string localError;
			if (!getInt(
					item,
					"intersectionId",
					intersectionId,
					localError) ||
				!getBoolOptional(
					item, "enabled", enabled, localError) ||
				!getDoubleOptional(
					item,
					"greenDuration",
					greenDuration,
					localError) ||
				!getDoubleOptional(
					item,
					"yellowDuration",
					yellowDuration,
					localError) ||
				!getDoubleOptional(
					item,
					"allRedDuration",
					allRedDuration,
					localError)) {
				if (error) *error = context + ": " + localError;
				return false;
			}
			const std::string signalContext =
				"Traffic light config for intersection " +
				std::to_string(intersectionId);
			if (!configuredIntersections.insert(
					intersectionId).second) {
				if (error) {
					*error = signalContext +
						": duplicate intersection config.";
				}
				return false;
			}

			Intersection* intersection =
				graph.getIntersection(intersectionId);
			if (intersection == nullptr) {
				if (error) {
					*error = signalContext +
						": intersection does not exist.";
				}
				return false;
			}
			if (!enabled) continue;

			if (!std::isfinite(greenDuration) ||
				greenDuration <= 0.0 ||
				!std::isfinite(yellowDuration) ||
				yellowDuration <= 0.0 ||
				!std::isfinite(allRedDuration) ||
				allRedDuration < 0.0) {
				if (error) {
					*error = signalContext +
						": greenDuration and yellowDuration must be "
						"positive; allRedDuration must be non-negative.";
				}
				return false;
			}

			if (!item.contains("phases")) {
				// Compact plans cover every incoming road and group opposing
				// approaches geometrically. This is useful for T-junctions,
				// roundabouts and other non-four-way conflict points.
				std::string planError;
				if (!intersection->configureTrafficSignalsAutomatically(
						greenDuration,
						yellowDuration,
						allRedDuration,
						&planError)) {
					if (error) {
						*error = signalContext + ": " + planError;
					}
					return false;
				}
				continue;
			}
			if (!item.at("phases").is_array() ||
				item.at("phases").empty()) {
				if (error) {
					*error = signalContext +
						": phases must be a non-empty array.";
				}
				return false;
			}

			std::vector<std::vector<Road*>> phases;
			const auto& phaseItems = item.at("phases");
			phases.reserve(phaseItems.size());
			for (std::size_t phaseIndex = 0;
				 phaseIndex < phaseItems.size();
				 ++phaseIndex) {
				const auto& phaseItem =
					phaseItems.at(phaseIndex);
				if (!phaseItem.is_object() ||
					!phaseItem.contains("incomingRoadIds") ||
					!phaseItem.at("incomingRoadIds").is_array() ||
					phaseItem.at("incomingRoadIds").empty()) {
					if (error) {
						*error = signalContext + ": phase " +
							std::to_string(phaseIndex) +
							" must contain a non-empty "
							"incomingRoadIds array.";
					}
					return false;
				}

				std::vector<Road*> phase;
				for (const auto& roadIdItem :
					 phaseItem.at("incomingRoadIds")) {
					if (!roadIdItem.is_number_integer()) {
						if (error) {
							*error = signalContext + ": phase " +
								std::to_string(phaseIndex) +
								" contains a non-integer road id.";
						}
						return false;
					}
					const int roadId =
						roadIdItem.get<int>();
					Road* road = graph.getRoad(roadId);
					if (road == nullptr ||
						road->getEnd() != intersection) {
						if (error) {
							*error = signalContext + ": road " +
								std::to_string(roadId) +
								" is not an incoming road.";
						}
						return false;
					}
					phase.push_back(road);
				}
				phases.push_back(std::move(phase));
			}

			std::string planError;
			if (!intersection->configureTrafficSignals(
					phases,
					greenDuration,
					yellowDuration,
					allRedDuration,
					&planError)) {
				if (error) {
					*error = signalContext + ": " + planError;
				}
				return false;
			}
		}
	}

	// --- Parse signalized pedestrian crossings (optional) ---
	if (root.contains("crosswalks")) {
		if (!root.at("crosswalks").is_array()) {
			if (error) {
				*error =
					"Invalid 'crosswalks': expected an array.";
			}
			return false;
		}

		std::unordered_set<int> crosswalkIds;
		const auto& items = root.at("crosswalks");
		for (std::size_t index = 0;
			 index < items.size();
			 ++index) {
			const auto& item = items.at(index);
			const std::string context =
				"Crosswalk at index " +
				std::to_string(index);
			if (!item.is_object()) {
				if (error) {
					*error = context +
						": expected a JSON object.";
				}
				return false;
			}

			int crosswalkId = 0;
			int intersectionId = 0;
			int incomingRoadId = 0;
			bool enabled = true;
			double width =
				Crosswalk::DEFAULT_WIDTH_METRES;
			CrosswalkTiming timing;
			std::string localError;
			if (!getInt(
					item, "id", crosswalkId, localError) ||
				!getInt(
					item,
					"intersectionId",
					intersectionId,
					localError) ||
				!getInt(
					item,
					"incomingRoadId",
					incomingRoadId,
					localError) ||
				!getBoolOptional(
					item,
					"enabled",
					enabled,
					localError) ||
				!getDoubleOptional(
					item, "width", width, localError) ||
				!getDoubleOptional(
					item,
					"minimumWait",
					timing.minimumWaitSeconds,
					localError) ||
				!getDoubleOptional(
					item,
					"walkDuration",
					timing.walkDurationSeconds,
					localError) ||
				!getDoubleOptional(
					item,
					"designWalkingSpeed",
					timing.
						designWalkingSpeedMetresPerSecond,
					localError) ||
				!getDoubleOptional(
					item,
					"clearanceBuffer",
					timing.clearanceBufferSeconds,
					localError)) {
				if (error) {
					*error = context + ": " + localError;
				}
				return false;
			}

			const std::string crosswalkContext =
				"Crosswalk " +
				std::to_string(crosswalkId) +
				" at index " +
				std::to_string(index);
			if (!crosswalkIds.insert(
					crosswalkId).second) {
				if (error) {
					*error = crosswalkContext +
						": duplicate crosswalk id.";
				}
				return false;
			}
			if (!enabled) continue;

			Intersection* intersection =
				graph.getIntersection(intersectionId);
			Road* incomingRoad =
				graph.getRoad(incomingRoadId);
			if (intersection == nullptr) {
				if (error) {
					*error = crosswalkContext +
						": intersectionId " +
						std::to_string(intersectionId) +
						" does not exist.";
				}
				return false;
			}
			if (incomingRoad == nullptr) {
				if (error) {
					*error = crosswalkContext +
						": incomingRoadId " +
						std::to_string(incomingRoadId) +
						" does not exist.";
				}
				return false;
			}
			if (incomingRoad->getEnd() != intersection) {
				if (error) {
					*error = crosswalkContext +
						": incoming road does not end at "
						"the configured intersection.";
				}
				return false;
			}
			if (!intersection->hasTrafficLights()) {
				if (error) {
					*error = crosswalkContext +
						": intersection must have a traffic "
						"signal plan.";
				}
				return false;
			}

			const bool validTiming =
				std::isfinite(
					timing.minimumWaitSeconds) &&
				timing.minimumWaitSeconds >= 0.0 &&
				std::isfinite(
					timing.walkDurationSeconds) &&
				timing.walkDurationSeconds > 0.0 &&
				std::isfinite(
					timing.
						designWalkingSpeedMetresPerSecond) &&
				timing.
					designWalkingSpeedMetresPerSecond >
					0.0 &&
				std::isfinite(
					timing.clearanceBufferSeconds) &&
				timing.clearanceBufferSeconds >= 0.0;
			if (!std::isfinite(width) ||
				width <= 0.0 ||
				!validTiming) {
				if (error) {
					*error = crosswalkContext +
						": width, walkDuration and "
						"designWalkingSpeed must be positive; "
						"minimumWait and clearanceBuffer must "
						"be non-negative.";
				}
				return false;
			}
			if (incomingRoad->getDistance() <=
				width +
					Crosswalk::JUNCTION_EDGE_GAP_METRES +
					Crosswalk::STOP_LINE_GAP_METRES) {
				if (error) {
					*error = crosswalkContext +
						": incoming road is too short for "
						"the crossing and upstream stop line.";
				}
				return false;
			}

			auto crosswalk =
				std::make_unique<Crosswalk>(
					crosswalkId,
					intersection,
					incomingRoad,
					width,
					timing);
			if (!graph.addCrosswalk(
					std::move(crosswalk))) {
				if (error) {
					*error = crosswalkContext +
						": duplicate physical approach or "
						"invalid graph ownership.";
				}
				return false;
			}
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

	graph.bindPOIsToRoads();
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
