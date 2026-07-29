#include "VehicleInspector.h"

#include <algorithm>
#include <cmath>

#include <imgui.h>

#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"
#include "algorithm/PathFindingStrategy.h"

namespace {
// m/s -> km/h chỉ dùng để HIỂN THỊ (theo nguyên tắc trong Units.h: convert
// chỉ ở biên UI, không đụng vào physics).
double mpsToKmh(double mps) { return mps * 3.6; }
}

void VehicleInspector::tryPickVehicle(TrafficSimulator* simulator,
                                       const VisualizationEngine& visualization,
                                       const sf::Vector2f& screenPos,
                                       float pickRadiusPixels) {
    if (simulator == nullptr) {
        clearSelection();
        return;
    }

    Vehicle* best = nullptr;
    float bestDistSq = pickRadiusPixels * pickRadiusPixels;

    // Chỉ pick trong số xe đang active trên đường - đúng với những gì
    // đang thực sự được vẽ lên map (Rendering.cpp cũng chỉ vẽ getVehicles()).
    for (Vehicle* v : simulator->getVehicles()) {
        if (v->getCurrentRoad() == nullptr) continue; // không có vị trí hợp lệ để vẽ

        VehicleSprite sprite(v, &visualization);
        const sf::Vector2f p = sprite.getPosition();
        const float dx = p.x - screenPos.x;
        const float dy = p.y - screenPos.y;
        const float distSq = dx * dx + dy * dy;

        if (distSq <= bestDistSq) {
            bestDistSq = distSq;
            best = v;
        }
    }

    if (best != nullptr) {
        selectedVehicleId_ = best->getId();
    } else {
        // Click-to-deselect: click trúng "khoảng trống" trên map (không có
        // xe nào trong bán kính pick) sẽ bỏ chọn xe hiện tại, nếu có.
        // Điều kiện WantCaptureMouse/isPicking() đã được lọc ở call site
        // (InputHandling.cpp) trước khi hàm này được gọi, nên ở đây một
        // "miss" luôn có nghĩa là click thật vào vùng map trống.
        clearSelection();
    }
}

Vehicle* VehicleInspector::findSelectedVehicle(TrafficSimulator* simulator) const {
    if (simulator == nullptr || selectedVehicleId_ == -1) return nullptr;

    for (Vehicle* v : simulator->getVehicles()) {
        if (v->getId() == selectedVehicleId_) return v;
    }
    // Xe đã hoàn thành trip vẫn được giữ lại trong finishedVehicles (không
    // bị delete - xem TrafficSimulator::removeFinishedVehicles), nên panel
    // vẫn hiển thị được thông tin cuối cùng của nó thay vì biến mất đột ngột.
    for (Vehicle* v : simulator->getFinishedVehicles()) {
        if (v->getId() == selectedVehicleId_) return v;
    }
    return nullptr;
}

void VehicleInspector::draw(TrafficSimulator* simulator, const VisualizationEngine& visualization) {
    (void)visualization;
    if (!hasSelection()) return;

    Vehicle* vehicle = findSelectedVehicle(simulator);
    if (vehicle == nullptr) {
        // Map đã được reload / simulator reset -> ID cũ không còn ý nghĩa.
        clearSelection();
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320.0f, 260.0f), ImGuiCond_FirstUseEver);

    bool open = true;
    std::string title = "Vehicle #" + std::to_string(vehicle->getId());
    if (ImGui::Begin(title.c_str(), &open)) {

        const bool finished = vehicle->hasReachedDestination();
        if (finished) {
            ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.5f, 1.0f), "Status: Completed trip");
        } else if (vehicle->isPaused()) {
            const char* reasonStr = "Unknown";
            switch (vehicle->getPauseReason()) {
                case PauseReason::TrafficLight:  reasonStr = "Waiting at red light"; break;
                case PauseReason::PedestrianCrossing:
                    reasonStr = "Waiting for pedestrians";
                    break;
                case PauseReason::Intersection:  reasonStr = "Waiting for intersection slot"; break;
                case PauseReason::BusStop:       reasonStr = "Dwelling at bus stop"; break;
                default: break;
            }
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "Status: %s", reasonStr);
        } else {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: Moving");
        }

        ImGui::Separator();

        // --- 1. Origin / Destination ---
        Intersection* origin = vehicle->getSpawnPoint();
        Intersection* dest = vehicle->getDestination();
        ImGui::Text("Origin:      #%d", origin ? origin->getId() : -1);
        ImGui::Text("Destination: #%d", dest ? dest->getId() : -1);

        // --- 3. Algorithm ---
        if (simulator && simulator->getPathFindingStrategy()) {
            ImGui::Text("Algorithm:   %s", simulator->getPathFindingStrategy()->name().c_str());
        }

        // --- 4. Current speed (real-time) ---
        ImGui::Separator();
        const double speedKmh = mpsToKmh(vehicle->getCurrentSpeed());
        ImGui::Text("Current speed: %.1f km/h", speedKmh);
        ImGui::Text("Progress on current road: %.0f%%", vehicle->getProgressRatio() * 100.0);

        // --- 2. Route ---
        ImGui::Separator();
        const auto& route = vehicle->getCurrentRoute();
        const int currentIdx = vehicle->getCurrentRouteIndex();
        ImGui::Text("Route (%d roads):", static_cast<int>(route.size()));

        ImGui::BeginChild("##route_list", ImVec2(0.0f, 100.0f), true);
        for (int i = 0; i < static_cast<int>(route.size()); ++i) {
            Road* road = route[i];
            if (road == nullptr) continue;

            std::string label = "#" + std::to_string(road->getId());
            if (!road->getName().empty()) label += " (" + road->getName() + ")";
            label += " [" + std::to_string(road->getStart()->getId()) + " -> "
                    + std::to_string(road->getEnd()->getId()) + "]";

            if (i == currentIdx) {
                // Xe đang ở đúng road này ngay bây giờ.
                ImGui::TextColored(ImVec4(0.4f, 0.9f, 0.4f, 1.0f), "-> %s", label.c_str());
            } else if (i < currentIdx) {
                ImGui::TextDisabled("   %s", label.c_str());
            } else {
                ImGui::Text("   %s", label.c_str());
            }
        }
        ImGui::EndChild();

        ImGui::Separator();
        if (ImGui::Button("Close")) {
            open = false;
        }
    }
    ImGui::End();

    if (!open) {
        clearSelection();
    }
}
