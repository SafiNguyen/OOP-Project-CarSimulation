#include "DebugConsole.h"

#include <algorithm>
#include <exception>
#include <new>
#include <sstream>
#include <vector>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "Intersection.h"
#include "model/infrastructure/PointOfInterest.h"
#include "model/infrastructure/SpawnPoint.h"
#include "Vehicle.h"
#include "model/vehicle/VehicleFactory.h"
#include "simulation/TrafficSimulator.h"
#include "UiTheme.h"

using debugconsole_detail::poiLabel;
using debugconsole_detail::nextFreeVehicleId;
using debugconsole_detail::isManualSpawnDestination;
using debugconsole_detail::isManualSpawnOrigin;
using debugconsole_detail::vehicleKindFromIndex;

namespace {

constexpr int MAX_MANUAL_SPAWN_BATCH = 1000;
constexpr std::size_t MAX_REPORTED_SPAWN_IDS = 12u;

const char* endpointRuleHint(VehicleKind kind) {
    switch (kind) {
        case VehicleKind::Car:
        case VehicleKind::Motorbike:
            return "Start and destination lists use each POI's configured civilian capabilities. OSM places with a positive spawnWeight are valid origins by default.";
        case VehicleKind::Bus:
            return "Start and destination require POIs configured for Bus vehicles.";
        case VehicleKind::Emergency:
            return "Start and destination require POIs configured for Emergency vehicles, including hospitals, clinics, police and fire stations.";
    }
    return "";
}

} // namespace

void DebugConsole::drawSpawnVehiclePanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Spawn Vehicle", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    const char* vehicleTypeNames[4] = { "Car", "Bus", "Motorbike", "Emergency Vehicle" };
    if (ImGui::Combo("Vehicle type", &spawnVehicleTypeIdx_, vehicleTypeNames, 4)) {
        const VehicleKind kind =
            vehicleKindFromIndex(spawnVehicleTypeIdx_);
        if (!isManualSpawnOrigin(
                kind,
                graph_.getPointOfInterest(spawnStartId_))) {
            spawnStartId_ = -1;
        }
        if (!isManualSpawnDestination(
                kind,
                graph_.getPointOfInterest(spawnEndId_))) {
            spawnEndId_ = -1;
        }
        if (spawnVehicleTypeIdx_ == 3) { // Emergency
            for (auto* poi : poisSnapshot_) {
                if (isManualSpawnOrigin(kind, poi)) {
                    spawnStartId_ = poi->getId();
                    break;
                }
            }
        }
    }

    const VehicleKind panelKind =
        vehicleKindFromIndex(spawnVehicleTypeIdx_);
    ImGui::TextWrapped("%s", endpointRuleHint(panelKind));

    int eligibleOriginCount = 0;
    int eligibleDestinationCount = 0;
    for (PointOfInterest* poi : poisSnapshot_) {
        if (isManualSpawnOrigin(panelKind, poi)) {
            ++eligibleOriginCount;
        }
        if (isManualSpawnDestination(panelKind, poi)) {
            ++eligibleDestinationCount;
        }
    }
    ImGui::TextDisabled(
        "Eligible POIs: %d origins, %d destinations",
        eligibleOriginCount,
        eligibleDestinationCount);

    {
        std::string previewStart = spawnStartId_ >= 0 ? ("#" + std::to_string(spawnStartId_)) : "(none)";
        if (spawnStartId_ >= 0) {
            auto it = std::find_if(poisSnapshot_.begin(), poisSnapshot_.end(),
                [this](PointOfInterest* p) { return p->getId() == spawnStartId_; });
            if (it != poisSnapshot_.end()) {
                previewStart = poiLabel(*it);
            }
        }

        if (ImGui::BeginCombo("Start##spawn", previewStart.c_str())) {
            const VehicleKind kind =
                vehicleKindFromIndex(spawnVehicleTypeIdx_);
            for (PointOfInterest* poi : poisSnapshot_) {
                if (!isManualSpawnOrigin(kind, poi)) continue;
                bool selected = (poi->getId() == spawnStartId_);
                if (ImGui::Selectable(poiLabel(poi).c_str(), selected)) {
                    spawnStartId_ = poi->getId();
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SameLine();
    if (UiTheme::selectionButton(
            pickTarget_ == PickTarget::SPAWN_START ? "Click map...##spawnstart"
                                                   : "Pick##spawnstart",
            pickTarget_ == PickTarget::SPAWN_START)) {
        pickTarget_ = (pickTarget_ == PickTarget::SPAWN_START) ? PickTarget::NONE : PickTarget::SPAWN_START;
    }
    UiTheme::tooltip("Pick the spawn POI directly on the map");

    {
        std::string previewEnd = spawnEndId_ >= 0 ? ("#" + std::to_string(spawnEndId_)) : "(none)";
        if (spawnEndId_ >= 0) {
            auto it = std::find_if(poisSnapshot_.begin(), poisSnapshot_.end(),
                [this](PointOfInterest* p) { return p->getId() == spawnEndId_; });
            if (it != poisSnapshot_.end()) {
                previewEnd = poiLabel(*it);
            }
        }

        if (ImGui::BeginCombo("Destination##spawn", previewEnd.c_str())) {
            const VehicleKind kind =
                vehicleKindFromIndex(spawnVehicleTypeIdx_);
            for (PointOfInterest* poi : poisSnapshot_) {
                if (!isManualSpawnDestination(kind, poi)) continue;
                bool selected = (poi->getId() == spawnEndId_);
                if (ImGui::Selectable(poiLabel(poi).c_str(), selected)) {
                    spawnEndId_ = poi->getId();
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SameLine();
    if (UiTheme::selectionButton(
            pickTarget_ == PickTarget::SPAWN_END ? "Click map...##spawnend"
                                                 : "Pick##spawnend",
            pickTarget_ == PickTarget::SPAWN_END)) {
        pickTarget_ = (pickTarget_ == PickTarget::SPAWN_END) ? PickTarget::NONE : PickTarget::SPAWN_END;
    }
    UiTheme::tooltip("Pick the destination POI directly on the map");

    ImGui::InputFloat("Base speed", &spawnVehicleSpeed_);
    spawnVehicleSpeed_ = std::max(1.0f, spawnVehicleSpeed_);
    ImGui::InputInt("Count to spawn", &spawnVehicleCount_);
    spawnVehicleCount_ = std::clamp(
        spawnVehicleCount_, 1, MAX_MANUAL_SPAWN_BATCH);
    ImGui::TextDisabled(
        "Maximum per action: %d vehicles",
        MAX_MANUAL_SPAWN_BATCH);

    PointOfInterest* startPoi = graph_.getPOI(spawnStartId_);
    if (!startPoi) startPoi = graph_.getBusStation(spawnStartId_);
    
    PointOfInterest* endPoi = graph_.getPOI(spawnEndId_);
    if (!endPoi) endPoi = graph_.getBusStation(spawnEndId_);

    Intersection* selectedStart = startPoi ? startPoi->getNearestIntersection() : nullptr;
    Intersection* selectedEnd = endPoi ? endPoi->getNearestIntersection() : nullptr;
    const VehicleKind selectedKind =
        vehicleKindFromIndex(spawnVehicleTypeIdx_);
    const bool validOrigin =
        isManualSpawnOrigin(selectedKind, startPoi);
    const bool validDestination =
        isManualSpawnDestination(selectedKind, endPoi);
    
    const bool canSpawn = simulator && validOrigin && validDestination &&
        selectedStart != nullptr && selectedEnd != nullptr &&
        startPoi != endPoi;
        
    ImGui::BeginDisabled(!canSpawn);
    if (UiTheme::actionButton("Spawn Vehicle", ImVec2(142.0f, 36.0f))) {
        spawnMessage_.clear();
        if (!simulator) {
            spawnMessage_ = "No active simulation.";
        } else if (startPoi == nullptr || endPoi == nullptr) {
            spawnMessage_ =
                "Pick both a start and a destination POI first.";
        } else if (startPoi == endPoi) {
            spawnMessage_ = "Start and destination must be different.";
        } else {
          try {
            std::vector<int> spawnedIds;
            spawnedIds.reserve(
                static_cast<std::size_t>(spawnVehicleCount_));
            for (int i = 0; i < spawnVehicleCount_; ++i) {
                const int newId = nextFreeVehicleId(simulator.get());
                Vehicle* v = VehicleFactory::createVehicle(
                    selectedKind,
                    newId,
                    spawnVehicleSpeed_,
                    selectedStart,
                    selectedEnd);
                if (v) {
                    v->setSpawnPOI(startPoi);
                    v->setTargetPOI(endPoi);
                }
                if (simulator->addVehicleImmediately(v)) {
                    spawnedIds.push_back(newId);
                    const auto existing = std::find_if(
                        manualSpawnHighlights_.begin(),
                        manualSpawnHighlights_.end(),
                        [newId](
                            const ManualSpawnHighlight& highlight) {
                            return highlight.vehicleId == newId;
                        });
                    if (existing !=
                        manualSpawnHighlights_.end()) {
                        existing->remainingSeconds =
                            MANUAL_SPAWN_HIGHLIGHT_SECONDS;
                    } else {
                        manualSpawnHighlights_.push_back(
                            {newId,
                             MANUAL_SPAWN_HIGHLIGHT_SECONDS});
                    }
                }
            }
            if (!spawnedIds.empty()) {
                std::ostringstream message;
                if (spawnedIds.size() == 1u) {
                    message << "Spawned vehicle ID #"
                            << spawnedIds.front() << ".";
                } else {
                    message << "Spawned " << spawnedIds.size()
                            << " vehicles. IDs: ";
                    const std::size_t displayedIds = std::min(
                        spawnedIds.size(), MAX_REPORTED_SPAWN_IDS);
                    for (std::size_t index = 0u;
                         index < displayedIds;
                         ++index) {
                        if (index > 0u) {
                            message << ", ";
                        }
                        message << '#' << spawnedIds[index];
                    }
                    if (spawnedIds.size() > displayedIds) {
                        message << ", ... ("
                                << (spawnedIds.size() - displayedIds)
                                << " more)";
                    }
                    message << '.';
                }
                const int failedCount =
                    spawnVehicleCount_ -
                    static_cast<int>(spawnedIds.size());
                if (failedCount > 0) {
                    message << ' ' << failedCount
                            << " could not be spawned.";
                }
                spawnMessage_ = message.str();
                setNotice(
                    failedCount == 0
                        ? NoticeTone::SUCCESS
                        : NoticeTone::WARNING,
                    spawnMessage_);
            } else {
                spawnMessage_ =
                    "Could not spawn immediately. Check that the route is available and its roads are not blocked.";
                setNotice(NoticeTone::ERROR, spawnMessage_);
            }
          } catch (const std::bad_alloc&) {
            spawnMessage_ =
                "Spawn stopped because the batch ran out of memory.";
            setNotice(NoticeTone::ERROR, spawnMessage_);
          } catch (const std::exception& exception) {
            spawnMessage_ = std::string("Spawn stopped: ") +
                exception.what();
            setNotice(NoticeTone::ERROR, spawnMessage_);
          } catch (...) {
            spawnMessage_ =
                "Spawn stopped after an unknown error.";
            setNotice(NoticeTone::ERROR, spawnMessage_);
          }
        }
    }
    ImGui::EndDisabled();
    if (!canSpawn) {
        UiTheme::tooltip(
            "Select a valid spawn point and destination for this vehicle type");
    }
    if (!spawnMessage_.empty()) {
        const bool success =
            spawnMessage_.rfind("Spawned", 0) == 0;
        const bool warning =
            success &&
            spawnMessage_.find("could not be spawned") !=
                std::string::npos;
        ImGui::TextColored(
            warning
                ? UiTheme::Warning
                : (success ? UiTheme::Success
                           : UiTheme::Error),
            "%s",
            spawnMessage_.c_str());
    }
    ImGui::TreePop();
}
