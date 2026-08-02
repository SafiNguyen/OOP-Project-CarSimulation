#include "GraphBuilder.h"
#include "MapParser.h"
	
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <nlohmann/json.hpp>

#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "RoadGeometry.h"
#include "Roundabout.h"
#include "Bridge.h"
#include "Tunnel.h"
#include "PointOfInterest.h"
#include "SpawnPoint.h"
#include "Destination.h"
#include "BusStop.h"
#include "BusService.h"
#include "common/Units.h"

using nlohmann::json;

namespace {

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool distanceUnitToMetresFactor(
        const std::string& distanceUnit,
        double& factor,
        std::string& error) {
    const std::string unit = toLowerCopy(distanceUnit);
    if (unit == "m") {
        factor = 1.0;
        return true;
    }
    if (unit == "km") {
        factor = Units::KM_TO_M_FACTOR;
        return true;
    }
    if (unit == "mi") {
        factor = 1609.344;
        return true;
    }
    if (unit == "legacy") {
        factor = 20.0;
        return true;
    }
    error = "Invalid distance unit: " + distanceUnit;
    return false;
}

bool speedUnitToMetresPerSecondFactor(
        const std::string& speedUnit,
        double& factor,
        std::string& error) {
    const std::string unit = toLowerCopy(speedUnit);
    if (unit == "m/s") {
        factor = 1.0;
        return true;
    }
    if (unit == "km/h" || unit == "kmh") {
        factor = 1.0 / Units::KMH_TO_MPS_FACTOR;
        return true;
    }
    error = "Invalid speed unit: " + speedUnit;
    return false;
}

} // namespace

namespace GraphBuilder {


bool buildGraphFromJsonString(const std::string& jsonText, Graph& graph, std::string* error) {
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
		if (!MapParser::getStringOptional(root, "defaultDistanceUnit", defaultDistanceUnit, rootUnitError) ||
			!MapParser::getStringOptional(root, "defaultSpeedUnit", defaultSpeedUnit, rootUnitError) ||
			!MapParser::getStringOptional(root, "defaultRadiusUnit", defaultRadiusUnit, rootUnitError) ||
			!MapParser::getDoubleOptional(root, "defaultLaneWidth", defaultLaneWidth, rootUnitError)) {
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

		if (!MapParser::getInt(item, "id", id, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		if (item.contains("x")) {
			if (!MapParser::getDouble(item, "x", x, localError)) {
				if (error) {
					*error = localError;
				}
				return false;
			}
		}

		if (item.contains("y")) {
			if (!MapParser::getDouble(item, "y", y, localError)) {
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
		MapParser::getStringOptional(item, "type", intersectionType, localError);

		if (intersectionType == "roundabout") {
			double radius = 0.02;
			double roadWidth = 7.0;
			std::string radiusUnit = defaultRadiusUnit;
			std::string roadWidthUnit = defaultRadiusUnit;
			if (!MapParser::getDoubleOptional(item, "radius", radius, localError) ||
				!MapParser::getStringOptional(item, "radiusUnit", radiusUnit, localError) ||
				!MapParser::getDoubleOptional(item, "roadWidth", roadWidth, localError) ||
				!MapParser::getStringOptional(
					item, "roadWidthUnit", roadWidthUnit, localError)) {
				if (error) *error = localError;
				return false;
			}
			double radiusFactor = 1.0;
			double roadWidthFactor = 1.0;
			if (!distanceUnitToMetresFactor(
					radiusUnit, radiusFactor, localError) ||
				!distanceUnitToMetresFactor(
					roadWidthUnit, roadWidthFactor, localError) ||
				!std::isfinite(radius) || radius <= 0.0) {
				if (error) {
					*error = !localError.empty()
						? localError
						: "Roundabout radius must be finite and positive.";
				}
				return false;
			}
			if (!std::isfinite(roadWidth) || roadWidth <= 0.0) {
				if (error) {
					*error =
						"Roundabout roadWidth must be finite and positive.";
				}
				return false;
			}
			graph.addIntersection(new Roundabout(
				id,
				x,
				y,
				radius * radiusFactor,
				roadWidth * roadWidthFactor));
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

		if (!MapParser::getInt(item, "id", id, localError) ||
			!MapParser::getInt(item, "start", startId, localError) ||
			!MapParser::getInt(item, "end", endId, localError) ||
			!MapParser::getDouble(item, "distance", distance, localError) ||
			!MapParser::getDouble(item, "speedLimit", speedLimit, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		if (!MapParser::getDoubleOptional(item, "congestionLevel", congestionLevel, localError) ||
			!MapParser::getDoubleOptional(item, "laneWidth", laneWidth, localError) ||
			!MapParser::getIntOptional(item, "lanes", lanes, localError) ||
			!MapParser::getBoolOptional(item, "twoWay", twoWay, localError) ||
			!MapParser::getBoolOptional(item, "blocked", blocked, localError)) {
			if (error) {
				*error = localError;
			}
			return false;
		}

		// Interpret boundary units once and normalize Road to metres and m/s.
		std::string distanceUnit = defaultDistanceUnit;
		if (!MapParser::getStringOptional(item, "distanceUnit", distanceUnit, localError)) {
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
		if (!MapParser::getStringOptional(item, "speedUnit", speedUnit, localError)) {
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
		MapParser::getStringOptional(item, "type", roadType, localError);
		MapParser::getStringOptional(item, "name", originalRoadName, localError);

		std::string roadName = originalRoadName;
		if (roadName.empty()) {
			roadName = "Road " + std::to_string(startId) + " -> " + std::to_string(endId);
		}

		Road* road = nullptr;
		if (roadType == "bridge") {
			double heightLimit = 4.5;
			double weightLimit = 30.0;
			MapParser::getDoubleOptional(item, "heightLimit", heightLimit, localError);
			MapParser::getDoubleOptional(item, "weightLimit", weightLimit, localError);
			road = new Bridge(id, roadName, start, end, distanceMeters, speedMetresPerSecond,
			                  congestionLevel, lanes, heightLimit, weightLimit,
			                  laneWidth);
		} else if (roadType == "tunnel") {
			double heightLimit = 3.5;
			MapParser::getDoubleOptional(item, "heightLimit", heightLimit, localError);
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
				MapParser::getDoubleOptional(item, "heightLimit", heightLimit, localError);
				MapParser::getDoubleOptional(item, "weightLimit", weightLimit, localError);
				revRoad = new Bridge(-id, revRoadName, end, start, distanceMeters, speedMetresPerSecond,
				                     congestionLevel, lanes, heightLimit, weightLimit,
				                     laneWidth);
			} else if (roadType == "tunnel") {
				double heightLimit = 3.5;
				MapParser::getDoubleOptional(item, "heightLimit", heightLimit, localError);
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
			bool allowRightTurnOnRed = true;
			double greenDuration = 25.0;
			double yellowDuration = 3.0;
			double allRedDuration = 1.5;
			std::string localError;
			if (!MapParser::getInt(
					item,
					"intersectionId",
					intersectionId,
					localError) ||
				!MapParser::getBoolOptional(
					item, "enabled", enabled, localError) ||
				!MapParser::getBoolOptional(
					item,
					"allowRightTurnOnRed",
					allowRightTurnOnRed,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"greenDuration",
					greenDuration,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"yellowDuration",
					yellowDuration,
					localError) ||
				!MapParser::getDoubleOptional(
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
			intersection->setAllowRightTurnOnRed(
				allowRightTurnOnRed);
			if (!enabled) continue;

			if (!std::isfinite(greenDuration) ||
				greenDuration <= 0.0 ||
				!std::isfinite(yellowDuration) ||
				yellowDuration <= 0.0 ||
				!std::isfinite(allRedDuration) ||
				allRedDuration <= 0.0) {
				if (error) {
					*error = signalContext +
						": greenDuration, yellowDuration and "
						"allRedDuration must be positive.";
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

	// --- Parse bus stops (optional section) ---
	std::unordered_map<int, const BusStop*> busStopsById;
	std::unordered_set<std::string> busStopCodes;
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
			std::string code;
			std::string name;
			std::string localError;

			if (!MapParser::getInt(item, "id", stopId, localError)) {
				if (error) *error = context + ": " + localError;
				return false;
			}
			const std::string stopContext = "Bus stop " + std::to_string(stopId)
			                              + " at index " + std::to_string(index);

			if (!MapParser::getInt(item, "roadId", roadId, localError) ||
			    !MapParser::getDouble(item, "positionRatio", positionRatio, localError)) {
				if (error) *error = stopContext + ": " + localError;
				return false;
			}
			if (!MapParser::getIntOptional(item, "lane", lane, localError) ||
			    !MapParser::getDoubleOptional(item, "dwellTime", dwellTime, localError) ||
			    !MapParser::getStringOptional(item, "code", code, localError) ||
			    !MapParser::getStringOptional(item, "name", name, localError)) {
				if (error) *error = stopContext + ": " + localError;
				return false;
			}

			if (!busStopIds.insert(stopId).second) {
				if (error) *error = stopContext + ": duplicate bus stop id.";
				return false;
			}
			if (item.contains("code") && code.empty()) {
				if (error) {
					*error = stopContext +
						": code must be a non-empty string.";
				}
				return false;
			}
			if (code.empty()) {
				code = "S" + std::to_string(stopId);
			}
			if (!busStopCodes.insert(code).second) {
				if (error) {
					*error = stopContext +
						": code '" + code +
						"' must be non-empty and unique.";
				}
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
				hasConfiguredDwellTime,
				code);
			const BusStop* storedBusStop = busStop.get();
			if (!road->addBusStop(std::move(busStop))) {
				if (error) {
					*error = stopContext + ": could not be added to road "
					       + std::to_string(roadId) + ".";
				}
				return false;
			}
			busStopsById.emplace(stopId, storedBusStop);
		}
	}

	// --- Parse public-transit stations (optional/backward-compatible) ---
	if (root.contains("busStations")) {
		if (!root.at("busStations").is_array()) {
			if (error) {
				*error =
					"Invalid 'busStations': expected an array.";
			}
			return false;
		}

		const auto& stationItems = root.at("busStations");
		for (std::size_t index = 0;
			 index < stationItems.size();
			 ++index) {
			const auto& item = stationItems.at(index);
			const std::string indexContext =
				"Bus station at index " +
				std::to_string(index);
			if (!item.is_object()) {
				if (error) {
					*error = indexContext +
						": expected a JSON object.";
				}
				return false;
			}

			int stationId = 0;
			int accessIntersectionId = 0;
			int departureRoadId = 0;
			int arrivalRoadId = 0;
			int capacity = 0;
			int accessLane = -1;
			double stationX = 0.0;
			double stationY = 0.0;
			double accessProgress = 0.0;
			std::string code;
			std::string name;
			std::string localError;
			if (!MapParser::getInt(item, "id", stationId, localError)) {
				if (error) {
					*error = indexContext + ": " + localError;
				}
				return false;
			}
			const std::string stationContext =
				"Bus station " + std::to_string(stationId) +
				" at index " + std::to_string(index);
			if (!MapParser::getStringOptional(
					item, "code", code, localError) ||
				!MapParser::getStringOptional(
					item, "name", name, localError) ||
				!MapParser::getDoubleOptional(
					item, "x", stationX, localError) ||
				!MapParser::getDoubleOptional(
					item, "y", stationY, localError) ||
				!MapParser::getInt(
					item,
					"accessIntersectionId",
					accessIntersectionId,
					localError) ||
				!MapParser::getInt(
					item,
					"departureRoadId",
					departureRoadId,
					localError) ||
				!MapParser::getInt(
					item,
					"arrivalRoadId",
					arrivalRoadId,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"accessProgress",
					accessProgress,
					localError) ||
				!MapParser::getIntOptional(
					item,
					"accessLane",
					accessLane,
					localError) ||
				!MapParser::getInt(
					item, "capacity", capacity, localError)) {
				if (error) {
					*error = stationContext + ": " + localError;
				}
				return false;
			}
			if (code.empty() || name.empty()) {
				if (error) {
					*error = stationContext +
						": code and name must be non-empty strings.";
				}
				return false;
			}
			if (capacity <= 0) {
				if (error) {
					*error = stationContext +
						": capacity must be positive.";
				}
				return false;
			}
			if (item.contains("x") != item.contains("y")) {
				if (error) {
					*error = stationContext +
						": x and y must be configured together.";
				}
				return false;
			}
			if (graph.getBusStation(stationId) != nullptr ||
				graph.getBusStationByCode(code) != nullptr) {
				if (error) {
					*error = stationContext +
						": id and code must be unique.";
				}
				return false;
			}

			Intersection* access =
				graph.getIntersection(accessIntersectionId);
			Road* departureRoad =
				graph.getRoad(departureRoadId);
			Road* arrivalRoad =
				graph.getRoad(arrivalRoadId);
			if (access == nullptr) {
				if (error) {
					*error = stationContext +
						": accessIntersectionId " +
						std::to_string(accessIntersectionId) +
						" does not exist.";
				}
				return false;
			}
			if (departureRoad == nullptr) {
				if (error) {
					*error = stationContext +
						": departureRoadId " +
						std::to_string(departureRoadId) +
						" does not exist.";
				}
				return false;
			}
			if (arrivalRoad == nullptr) {
				if (error) {
					*error = stationContext +
						": arrivalRoadId " +
						std::to_string(arrivalRoadId) +
						" does not exist.";
				}
				return false;
			}
			if (departureRoad->getStart() != access ||
				arrivalRoad->getEnd() != access) {
				if (error) {
					*error = stationContext +
						": departure road must start and arrival "
						"road must end at the access intersection.";
				}
				return false;
			}
			if (!item.contains("x")) {
				stationX = access->getX();
				stationY = access->getY();
			}
			if (!std::isfinite(stationX) ||
				!std::isfinite(stationY) ||
				!std::isfinite(accessProgress) ||
				accessProgress < 0.0 ||
				accessProgress >
					departureRoad->getDistance() ||
				accessLane < -1 ||
				accessLane >=
					departureRoad->getLaneCount()) {
				if (error) {
					*error = stationContext +
						": invalid coordinates, accessProgress, "
						"or accessLane.";
				}
				return false;
			}

			auto station = std::make_unique<BusStation>(
				stationId,
				code,
				name,
				stationX,
				stationY,
				access,
				departureRoad,
				arrivalRoad,
				capacity,
				accessProgress,
				accessLane);
			if (!graph.addBusStation(std::move(station))) {
				if (error) {
					*error = stationContext +
						": could not be added to Graph.";
				}
				return false;
			}
		}
	}

	// --- Parse immutable public-transit services ---
	if (root.contains("busServices")) {
		if (!root.at("busServices").is_array()) {
			if (error) {
				*error =
					"Invalid 'busServices': expected an array.";
			}
			return false;
		}
		if (graph.getAllBusStations().empty() &&
			!root.at("busServices").empty()) {
			if (error) {
				*error =
					"Invalid 'busServices': no busStations are configured.";
			}
			return false;
		}

		const auto& serviceItems = root.at("busServices");
		for (std::size_t index = 0;
			 index < serviceItems.size();
			 ++index) {
			const auto& item = serviceItems.at(index);
			const std::string indexContext =
				"Bus service at index " +
				std::to_string(index);
			if (!item.is_object()) {
				if (error) {
					*error = indexContext +
						": expected a JSON object.";
				}
				return false;
			}

			int serviceId = 0;
			int originStationId = 0;
			int destinationStationId = 0;
			std::string code;
			std::string name;
			std::string localError;
			if (!MapParser::getInt(item, "id", serviceId, localError)) {
				if (error) {
					*error = indexContext + ": " + localError;
				}
				return false;
			}
			const std::string serviceContext =
				"Bus service " + std::to_string(serviceId) +
				" at index " + std::to_string(index);
			if (!MapParser::getStringOptional(
					item, "code", code, localError) ||
				!MapParser::getStringOptional(
					item, "name", name, localError) ||
				!MapParser::getInt(
					item,
					"originStationId",
					originStationId,
					localError) ||
				!MapParser::getInt(
					item,
					"destinationStationId",
					destinationStationId,
					localError)) {
				if (error) {
					*error = serviceContext + ": " + localError;
				}
				return false;
			}
			if (code.empty() || name.empty()) {
				if (error) {
					*error = serviceContext +
						": code and name must be non-empty strings.";
				}
				return false;
			}
			if (graph.getBusService(serviceId) != nullptr ||
				graph.getBusServiceByCode(code) != nullptr) {
				if (error) {
					*error = serviceContext +
						": id and code must be unique.";
				}
				return false;
			}
			BusStation* origin =
				graph.getBusStation(originStationId);
			BusStation* destination =
				graph.getBusStation(destinationStationId);
			if (origin == nullptr || destination == nullptr) {
				if (error) {
					*error = serviceContext +
						": originStationId and destinationStationId "
						"must reference configured stations.";
				}
				return false;
			}
			if (origin == destination) {
				if (error) {
					*error = serviceContext +
						": origin and destination must be different.";
				}
				return false;
			}
			if (!item.contains("roadIds") ||
				!item.at("roadIds").is_array() ||
				item.at("roadIds").empty()) {
				if (error) {
					*error = serviceContext +
						": roadIds must be a non-empty array.";
				}
				return false;
			}
			if (!item.contains("stopIds") ||
				!item.at("stopIds").is_array()) {
				if (error) {
					*error = serviceContext +
						": stopIds must be an array.";
				}
				return false;
			}

			std::vector<Road*> route;
			for (const auto& roadIdItem :
				 item.at("roadIds")) {
				if (!roadIdItem.is_number_integer()) {
					if (error) {
						*error = serviceContext +
							": roadIds contains a non-integer value.";
					}
					return false;
				}
				const int roadId = roadIdItem.get<int>();
				Road* road = graph.getRoad(roadId);
				if (road == nullptr) {
					if (error) {
						*error = serviceContext +
							": roadId " + std::to_string(roadId) +
							" does not exist.";
					}
					return false;
				}
				if (!route.empty() &&
					route.back()->getEnd() != road->getStart()) {
					if (error) {
						*error = serviceContext +
							": road route is not continuous before road " +
							std::to_string(roadId) + ".";
					}
					return false;
				}
				route.push_back(road);
			}
			if (route.front() != origin->getDepartureRoad() ||
				route.front()->getStart() !=
					origin->getAccessIntersection()) {
				if (error) {
					*error = serviceContext +
						": route must begin on the origin station's "
						"departure road.";
				}
				return false;
			}
			if (route.back() != destination->getArrivalRoad() ||
				route.back()->getEnd() !=
					destination->getAccessIntersection()) {
				if (error) {
					*error = serviceContext +
						": route must end on the destination station's "
						"arrival road.";
				}
				return false;
			}

			std::vector<const BusStop*> orderedStops;
			std::unordered_set<int> serviceStopIds;
			std::size_t minimumRouteIndex = 0;
			double previousPosition = -1.0;
			bool hasPreviousStop = false;
			for (const auto& stopIdItem :
				 item.at("stopIds")) {
				if (!stopIdItem.is_number_integer()) {
					if (error) {
						*error = serviceContext +
							": stopIds contains a non-integer value.";
					}
					return false;
				}
				const int stopId = stopIdItem.get<int>();
				if (!serviceStopIds.insert(stopId).second) {
					if (error) {
						*error = serviceContext +
							": duplicate stopId " +
							std::to_string(stopId) + ".";
					}
					return false;
				}
				const auto stopFound =
					busStopsById.find(stopId);
				if (stopFound == busStopsById.end()) {
					if (error) {
						*error = serviceContext +
							": stopId " + std::to_string(stopId) +
							" does not exist.";
					}
					return false;
				}
				const BusStop* stop = stopFound->second;
				std::size_t routeIndex = minimumRouteIndex;
				while (routeIndex < route.size() &&
					   route[routeIndex] != stop->getRoad()) {
					++routeIndex;
				}
				if (routeIndex == route.size()) {
					if (error) {
						*error = serviceContext +
							": stop " + std::to_string(stopId) +
							" is not on the ordered directional route.";
					}
					return false;
				}
				if (hasPreviousStop &&
					routeIndex == minimumRouteIndex &&
					stop->getPositionOnRoad() <=
						previousPosition) {
					if (error) {
						*error = serviceContext +
							": stop order is not increasing on road " +
							std::to_string(stop->getRoadId()) + ".";
					}
					return false;
				}
				previousPosition =
					stop->getPositionOnRoad();
				minimumRouteIndex = routeIndex;
				hasPreviousStop = true;
				orderedStops.push_back(stop);
			}

			auto service = std::make_unique<BusService>(
				serviceId,
				code,
				name,
				*origin,
				*destination,
				std::move(route),
				std::move(orderedStops));
			if (!graph.addBusService(std::move(service))) {
				if (error) {
					*error = serviceContext +
						": could not be added to Graph.";
				}
				return false;
			}
		}
	}

	// --- Parse POIs (optional section) ---
	if (root.contains("pois") && root.at("pois").is_array()) {
		for (const auto& item : root.at("pois")) {
			if (!item.is_object()) {
				if (error) *error = "Each POI must be a JSON object.";
				return false;
			}

			int poiId = 0;
			double px = 0.0, py = 0.0;
			std::string poiName, poiTypeStr;
			int nearestId = -1;
			int capacity = -1;
			int accessRoadId = 0;
			int accessLane = -1;
			double spawnWeight = 1.0;
			double destinationWeight = 1.0;
			double spawnCooldown = 1.0;
			double accessProgress = 0.0;
			double positionRatio = 0.0;
			std::string localError;

			if (!MapParser::getInt(item, "id", poiId, localError) ||
				!MapParser::getDoubleOptional(item, "x", px, localError) ||
				!MapParser::getDoubleOptional(item, "y", py, localError) ||
				!MapParser::getStringOptional(item, "name", poiName, localError) ||
				!MapParser::getStringOptional(item, "type", poiTypeStr, localError) ||
				!MapParser::getIntOptional(
					item, "nearestIntersection", nearestId, localError) ||
				!MapParser::getIntOptional(item, "capacity", capacity, localError) ||
				!MapParser::getIntOptional(
					item, "accessRoadId", accessRoadId, localError) ||
				!MapParser::getIntOptional(
					item, "accessLane", accessLane, localError) ||
				!MapParser::getDoubleOptional(
					item, "spawnWeight", spawnWeight, localError) ||
				!MapParser::getDoubleOptional(
					item,
					"destinationWeight",
					destinationWeight,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"spawnCooldown",
					spawnCooldown,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"accessProgress",
					accessProgress,
					localError) ||
				!MapParser::getDoubleOptional(
					item,
					"positionRatio",
					positionRatio,
					localError)) {
				if (error) {
					*error =
						"POI " + std::to_string(poiId) +
						": " + localError;
				}
				return false;
			}
			if (graph.getPOI(poiId) != nullptr) {
				if (error) {
					*error =
						"Duplicate POI id: " +
						std::to_string(poiId);
				}
				return false;
			}
			if (!std::isfinite(px) || !std::isfinite(py) ||
				!std::isfinite(spawnWeight) ||
				!std::isfinite(destinationWeight) ||
				!std::isfinite(spawnCooldown) ||
				spawnWeight < 0.0 ||
				destinationWeight < 0.0 ||
				spawnCooldown <= 0.0) {
				if (error) {
					*error =
						"POI " + std::to_string(poiId) +
						": coordinates and weights must be finite; "
						"weights must be non-negative and "
						"spawnCooldown must be positive.";
				}
				return false;
			}
			if (item.contains("accessProgress") &&
				item.contains("positionRatio")) {
				if (error) {
					*error =
						"POI " + std::to_string(poiId) +
						": configure either accessProgress or "
						"positionRatio, not both.";
				}
				return false;
			}

			Intersection* nearest = (nearestId >= 0) ? graph.getIntersection(nearestId) : nullptr;
			if (nearestId >= 0 && nearest == nullptr) {
				if (error) {
					*error =
						"POI " + std::to_string(poiId) +
						": nearestIntersection does not exist.";
				}
				return false;
			}
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
				case POIType::RESIDENTIAL_AREA:
					poi = new ResidentialArea(
						poiId, poiName, px, py, nearest);
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
			poi->setSpawnWeight(spawnWeight);
			poi->setDestinationWeight(destinationWeight);
			poi->setSpawnCooldownSeconds(spawnCooldown);

			if (item.contains("capacity")) {
				SpawnPoint* spawnPoint =
					dynamic_cast<SpawnPoint*>(poi);
				if (spawnPoint == nullptr || capacity <= 0) {
					delete poi;
					if (error) {
						*error =
							"POI " + std::to_string(poiId) +
							": capacity is only valid for a spawn "
							"point and must be positive.";
					}
					return false;
				}
				spawnPoint->setCapacity(capacity);
			}

			if (item.contains("accessRoadId")) {
				Road* accessRoad =
					graph.getRoad(accessRoadId);
				if (accessRoad == nullptr) {
					delete poi;
					if (error) {
						*error =
							"POI " + std::to_string(poiId) +
							": accessRoadId " +
							std::to_string(accessRoadId) +
							" does not exist.";
					}
					return false;
				}
				double progress = 0.0;
				if (item.contains("positionRatio")) {
					if (!std::isfinite(positionRatio) ||
						positionRatio < 0.0 ||
						positionRatio > 1.0) {
						delete poi;
						if (error) {
							*error =
								"POI " + std::to_string(poiId) +
								": positionRatio must be in [0, 1].";
						}
						return false;
					}
					progress =
						positionRatio *
						accessRoad->getDistance();
				} else if (item.contains("accessProgress")) {
					progress = accessProgress;
				} else {
					delete poi;
					if (error) {
						*error =
							"POI " + std::to_string(poiId) +
							": explicit accessRoadId requires "
							"accessProgress or positionRatio.";
					}
					return false;
				}
				if (!std::isfinite(progress) ||
					progress < 0.0 ||
					progress > accessRoad->getDistance() ||
					accessLane < -1 ||
					accessLane >=
						accessRoad->getLaneCount()) {
					delete poi;
					if (error) {
						*error =
							"POI " + std::to_string(poiId) +
							": invalid road access progress or lane.";
					}
					return false;
				}
				poi->configureRoadAccess(
					accessRoad,
					progress,
					accessLane);
				if (nearest == nullptr) {
					poi->setNearestIntersection(
						progress <=
								accessRoad->getDistance() * 0.5
							? accessRoad->getStart()
							: accessRoad->getEnd());
				}
			} else if (
				item.contains("accessProgress") ||
				item.contains("positionRatio") ||
				item.contains("accessLane")) {
				delete poi;
				if (error) {
					*error =
						"POI " + std::to_string(poiId) +
						": road access fields require accessRoadId.";
				}
				return false;
			}
			graph.addPOI(poi);
		}
	}

	graph.bindPOIsToRoads();
	return true;
}


}
