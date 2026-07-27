#include "DebugConsole.h"

#include <algorithm>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "model/Bus.h"
#include "model/Car.h"
#include "model/EmergencyVehicle.h"
#include "model/Intersection.h"
#include "model/Motorbike.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "UiTheme.h"

using debugconsole_detail::intersectionLabel;
using debugconsole_detail::nextFreeVehicleId;

void DebugConsole::drawSpawnVehiclePanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Spawn Vehicle", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    {
        std::string previewStart = spawnStartId_ >= 0 ? ("#" + std::to_string(spawnStartId_)) : "(none)";
        if (ImGui::BeginCombo("Start##spawn", previewStart.c_str())) {
            for (Intersection* it : intersectionsSnapshot_) {
                bool selected = (it->getId() == spawnStartId_);
                if (ImGui::Selectable(intersectionLabel(it).c_str(), selected)) {
                    spawnStartId_ = it->getId();
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
    UiTheme::tooltip("Pick the spawn intersection directly on the map");

    {
        std::string previewEnd = spawnEndId_ >= 0 ? ("#" + std::to_string(spawnEndId_)) : "(none)";
        if (ImGui::BeginCombo("Destination##spawn", previewEnd.c_str())) {
            for (Intersection* it : intersectionsSnapshot_) {
                bool selected = (it->getId() == spawnEndId_);
                if (ImGui::Selectable(intersectionLabel(it).c_str(), selected)) {
                    spawnEndId_ = it->getId();
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
    UiTheme::tooltip("Pick the destination intersection directly on the map");

    const char* vehicleTypeNames[4] = { "Car", "Bus", "Motorbike", "Emergency Vehicle" };
    ImGui::Combo("Vehicle type", &spawnVehicleTypeIdx_, vehicleTypeNames, 4);
    ImGui::InputFloat("Base speed", &spawnVehicleSpeed_);
    spawnVehicleSpeed_ = std::max(1.0f, spawnVehicleSpeed_);
    ImGui::InputInt("Count to spawn", &spawnVehicleCount_);
    if (spawnVehicleCount_ < 1) spawnVehicleCount_ = 1;

    Intersection* selectedStart = graph_.getIntersection(spawnStartId_);
    Intersection* selectedEnd = graph_.getIntersection(spawnEndId_);
    const bool canSpawn = simulator && selectedStart != nullptr
        && selectedEnd != nullptr && selectedStart != selectedEnd;
    ImGui::BeginDisabled(!canSpawn);
    if (UiTheme::actionButton("Spawn Vehicle", ImVec2(142.0f, 36.0f))) {
        spawnMessage_.clear();
        Intersection* start = graph_.getIntersection(spawnStartId_);
        Intersection* end = graph_.getIntersection(spawnEndId_);
        if (!simulator) {
            spawnMessage_ = "No active simulation.";
        } else if (start == nullptr || end == nullptr) {
            spawnMessage_ = "Pick both a start and a destination POI first.";
        } else if (start == end) {
            spawnMessage_ = "Start and destination must be different.";
        } else {
            int successCount = 0;
            for (int i = 0; i < spawnVehicleCount_; ++i) {
                const int newId = nextFreeVehicleId(simulator.get());
                Vehicle* v = nullptr;
                switch (spawnVehicleTypeIdx_) {
                    case 0: v = new Car(newId, spawnVehicleSpeed_, start, end); break;
                    case 1: v = new Bus(newId, spawnVehicleSpeed_, start, end); break;
                    case 2: v = new Motorbike(newId, spawnVehicleSpeed_, start, end); break;
                    default: v = new EmergencyVehicle(newId, spawnVehicleSpeed_, start, end); break;
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
        UiTheme::tooltip("Select two different intersections and keep a simulation active");
    }
    if (!spawnMessage_.empty()) {
        const bool success =
            spawnMessage_.rfind("Created", 0) == 0;
        ImGui::TextColored(success ? UiTheme::Success : UiTheme::Error,
                           "%s", spawnMessage_.c_str());
    }
    ImGui::TreePop();
}
