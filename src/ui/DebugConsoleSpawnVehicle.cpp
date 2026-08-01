#include "DebugConsole.h"

#include <algorithm>

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

void DebugConsole::drawSpawnVehiclePanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Spawn Vehicle", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    const char* vehicleTypeNames[4] = { "Car", "Bus", "Motorbike", "Emergency Vehicle" };
    if (ImGui::Combo("Vehicle type", &spawnVehicleTypeIdx_, vehicleTypeNames, 4)) {
        if (spawnVehicleTypeIdx_ == 3) { // Emergency
            for (auto* poi : poisSnapshot_) {
                if (poi->getType() == POIType::HOSPITAL) {
                    spawnStartId_ = poi->getId();
                    break;
                }
            }
        }
    }

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
            for (PointOfInterest* poi : poisSnapshot_) {
                if (spawnVehicleTypeIdx_ == 1 && poi->getType() != POIType::BUS_STATION) continue;
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
            for (PointOfInterest* poi : poisSnapshot_) {
                if (spawnVehicleTypeIdx_ == 1 && poi->getType() != POIType::BUS_STATION) continue;
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
    if (spawnVehicleCount_ < 1) spawnVehicleCount_ = 1;

    PointOfInterest* startPoi = graph_.getPOI(spawnStartId_);
    if (!startPoi) startPoi = graph_.getBusStation(spawnStartId_);
    
    PointOfInterest* endPoi = graph_.getPOI(spawnEndId_);
    if (!endPoi) endPoi = graph_.getBusStation(spawnEndId_);

    Intersection* selectedStart = startPoi ? startPoi->getNearestIntersection() : nullptr;
    Intersection* selectedEnd = endPoi ? endPoi->getNearestIntersection() : nullptr;
    
    const bool canSpawn = simulator && startPoi != nullptr
        && endPoi != nullptr && selectedStart != nullptr && selectedEnd != nullptr && startPoi != endPoi;
        
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
            int successCount = 0;
            for (int i = 0; i < spawnVehicleCount_; ++i) {
                const int newId = nextFreeVehicleId(simulator.get());
                Vehicle* v = nullptr;
                VehicleKind kind = VehicleKind::Car;
                if (spawnVehicleTypeIdx_ == 0) {
                    kind = VehicleKind::Car;
                } else if (spawnVehicleTypeIdx_ == 1) {
                    kind = VehicleKind::Bus;
                } else if (spawnVehicleTypeIdx_ == 2) {
                    kind = VehicleKind::Motorbike;
                } else {
                    kind = VehicleKind::Emergency;
                }
                v = VehicleFactory::createVehicle(
                    kind, newId, spawnVehicleSpeed_, selectedStart, selectedEnd);
                if (v) {
                    v->setSpawnPOI(startPoi);
                    v->setTargetPOI(endPoi);
                }
                if (simulator->addVehicle(v)) {
                    successCount++;
                }
            }
            if (successCount > 0) {
                spawnMessage_ =
                    "Created " +
                    std::to_string(successCount) +
                    " vehicles. Busy entrances use the safe spawn queue.";
                setNotice(NoticeTone::SUCCESS, spawnMessage_);
            } else {
                spawnMessage_ = "No path exists between those two points; vehicles were not spawned.";
                setNotice(NoticeTone::ERROR, spawnMessage_);
            }
        }
    }
    ImGui::EndDisabled();
    if (!canSpawn) {
        UiTheme::tooltip("Select two different POIs and keep a simulation active");
    }
    if (!spawnMessage_.empty()) {
        const bool success =
            spawnMessage_.rfind("Created", 0) == 0;
        ImGui::TextColored(success ? UiTheme::Success : UiTheme::Error,
                           "%s", spawnMessage_.c_str());
    }
    ImGui::TreePop();
}
