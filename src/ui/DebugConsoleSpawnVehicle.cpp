#include "DebugConsole.h"

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "model/Bus.h"
#include "model/Car.h"
#include "model/EmergencyVehicle.h"
#include "model/Intersection.h"
#include "model/Motorbike.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"

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
    if (ImGui::Button(pickTarget_ == PickTarget::SPAWN_START ? "Click map..." : "Pick##spawnstart")) {
        pickTarget_ = (pickTarget_ == PickTarget::SPAWN_START) ? PickTarget::NONE : PickTarget::SPAWN_START;
    }

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
    if (ImGui::Button(pickTarget_ == PickTarget::SPAWN_END ? "Click map..." : "Pick##spawnend")) {
        pickTarget_ = (pickTarget_ == PickTarget::SPAWN_END) ? PickTarget::NONE : PickTarget::SPAWN_END;
    }

    const char* vehicleTypeNames[4] = { "Car", "Bus", "Motorbike", "Emergency Vehicle" };
    ImGui::Combo("Vehicle type", &spawnVehicleTypeIdx_, vehicleTypeNames, 4);
    ImGui::InputFloat("Base speed", &spawnVehicleSpeed_);
    ImGui::InputInt("Count to spawn", &spawnVehicleCount_);
    if (spawnVehicleCount_ < 1) spawnVehicleCount_ = 1;

    if (ImGui::Button("Spawn Vehicle")) {
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
                spawnMessage_ = "Successfully spawned " + std::to_string(successCount) + " vehicles.";
            } else {
                spawnMessage_ = "No path exists between those two points; vehicles were not spawned.";
            }
        }
    }
    if (!spawnMessage_.empty()) {
        ImGui::TextColored(ImVec4(0.75f, 0.90f, 0.75f, 1.0f), "%s", spawnMessage_.c_str());
    }
    ImGui::TreePop();
}
