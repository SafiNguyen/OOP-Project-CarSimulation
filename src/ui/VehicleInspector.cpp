#include "VehicleInspector.h"

#include <algorithm>
#include <cmath>

#include <imgui.h>

#include "Intersection.h"
#include "Bus.h"
#include "BusService.h"
#include "BusStop.h"
#include "Road.h"
#include "SpawnPoint.h"
#include "Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/VehicleSprite.h"
#include "visualization/VisualizationEngine.h"
#include "algorithm/PathFindingStrategy.h"

namespace {
// m/s -> km/h chỉ dùng để HIỂN THỊ (theo nguyên tắc trong Units.h: convert
// chỉ ở biên UI, không đụng vào physics).
double mpsToKmh(double mps) { return mps * 3.6; }

const char* busTripStateLabel(BusTripState state) {
    switch (state) {
        case BusTripState::WaitingAtOrigin:
            return "Waiting at origin";
        case BusTripState::Departing:
            return "Departing";
        case BusTripState::EnRoute:
            return "En route";
        case BusTripState::Dwelling:
            return "Dwelling";
        case BusTripState::Arrived:
            return "Arrived";
    }
    return "Unknown";
}

const char* spawnStateLabel(
    SpawnLifecycleState state) {
    switch (state) {
        case SpawnLifecycleState::Scheduled:
            return "Scheduled";
        case SpawnLifecycleState::
                WaitingForSourceCapacity:
            return "Waiting for source capacity";
        case SpawnLifecycleState::WaitingForRoute:
            return "Waiting for route";
        case SpawnLifecycleState::WaitingForRoadGap:
            return "Waiting for a safe road gap";
        case SpawnLifecycleState::Merging:
            return "Merging from origin";
        case SpawnLifecycleState::Active:
            return "Active";
    }
    return "Unknown";
}

const char* poiMergePhaseLabel(PoiMergePhase phase) {
    switch (phase) {
        case PoiMergePhase::ApproachingYieldLine:
            return "Approaching road yield line";
        case PoiMergePhase::WaitingForGap:
            return "Waiting for a safe merge gap";
        case PoiMergePhase::Committed:
            return "Merging into lane";
        case PoiMergePhase::None:
            return "Merging from origin";
    }
    return "Merging from origin";
}
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
    if (!hasSelection()) return;

    Vehicle* vehicle = findSelectedVehicle(simulator);
    if (vehicle == nullptr) {
        // Map đã được reload / simulator reset -> ID cũ không còn ý nghĩa.
        clearSelection();
        return;
    }

    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(
        ImVec2(380.0f, 520.0f),
        ImGuiCond_FirstUseEver);

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
                case PauseReason::Intersection:  reasonStr = "Waiting for intersection slot"; break;
                case PauseReason::BusStop:       reasonStr = "Dwelling at bus stop"; break;
                default: break;
            }
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.3f, 1.0f), "Status: %s", reasonStr);
        } else if (vehicle->getSpawnLifecycleState() ==
                   SpawnLifecycleState::Merging) {
            ImGui::TextColored(
                ImVec4(0.95f, 0.75f, 0.3f, 1.0f),
                "Status: %s",
                poiMergePhaseLabel(
                    vehicle->getPoiMergePhase()));
        } else {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "Status: Moving");
        }

        ImGui::Separator();

        // --- 1. Origin / Destination ---
        Intersection* origin = vehicle->getSpawnPoint();
        Intersection* dest = vehicle->getDestination();
        const PointOfInterest* originPOI =
            vehicle->getSpawnPOI();
        const PointOfInterest* destinationPOI =
            vehicle->getTargetPOI();
        const Bus* transitBus = nullptr;
        if (vehicle->getVehicleKind() ==
                VehicleKind::Bus) {
            const auto& bus =
                static_cast<const Bus&>(*vehicle);
            if (bus.hasTransitService()) {
                transitBus = &bus;
            }
        }

        if (transitBus != nullptr) {
            const BusStation* station =
                transitBus->getOriginStation();
            ImGui::Text(
                "Origin:      Bus Station %s - %s (#%d)",
                station->getCode().c_str(),
                station->getName().c_str(),
                station->getId());
        } else if (originPOI != nullptr) {
            const std::string displayName =
                visualization.
                    getPointOfInterestDisplayName(
                        simulator->getGraph(),
                        originPOI);
            ImGui::Text(
                "Origin:      %s (#%d)",
                displayName.c_str(),
                originPOI->getId());
        } else {
            ImGui::Text(
                "Origin:      intersection #%d",
                origin ? origin->getId() : -1);
        }
        if (transitBus != nullptr) {
            const BusStation* station =
                transitBus->getDestinationStation();
            ImGui::Text(
                "Destination: Bus Station %s - %s (#%d)",
                station->getCode().c_str(),
                station->getName().c_str(),
                station->getId());
        } else if (destinationPOI != nullptr) {
            const std::string displayName =
                visualization.
                    getPointOfInterestDisplayName(
                        simulator->getGraph(),
                        destinationPOI);
            ImGui::Text(
                "Destination: %s (#%d)",
                displayName.c_str(),
                destinationPOI->getId());
        } else {
            ImGui::Text(
                "Destination: intersection #%d",
                dest ? dest->getId() : -1);
        }

        if (transitBus != nullptr) {
            const auto& bus = *transitBus;
            const BusService* service =
                bus.getService();
            const BusStop* nextStop =
                bus.getNextScheduledStop();
            ImGui::Separator();
            ImGui::Text(
                "Service: %s",
                service->getCode().c_str());
            ImGui::Text(
                "Trip state: %s",
                busTripStateLabel(
                    bus.getTripState()));
            ImGui::Text(
                "Next stop: %s",
                nextStop != nullptr
                    ? nextStop->getCode().c_str()
                    : "none");
            ImGui::Text(
                "Served: %d   Missed: %d",
                static_cast<int>(
                    bus.getServedStopIds().size()),
                static_cast<int>(
                    bus.getMissedStopIds().size()));
            ImGui::Text("Stops assigned to this Bus:");
            ImGui::BeginChild(
                "##bus_stop_list",
                ImVec2(0.0f, 115.0f),
                true);
            const auto& stops =
                bus.getAssignedStops();
            const auto& servedStopIds =
                bus.getServedStopIds();
            const auto& missedStopIds =
                bus.getMissedStopIds();
            for (std::size_t index = 0;
                 index < stops.size();
                 ++index) {
                const BusStop* stop =
                    stops[index];
                if (stop == nullptr) continue;
                const bool served =
                    std::find(
                        servedStopIds.begin(),
                        servedStopIds.end(),
                        stop->getId()) !=
                    servedStopIds.end();
                const bool missed =
                    std::find(
                        missedStopIds.begin(),
                        missedStopIds.end(),
                        stop->getId()) !=
                    missedStopIds.end();
                const bool next =
                    index ==
                    bus.getScheduledStopIndex();
                const char* status =
                    served ? "served"
                    : missed ? "missed"
                    : next ? "next"
                    : "pending";
                const ImVec4 color =
                    served
                        ? ImVec4(
                              0.4f, 0.9f,
                              0.4f, 1.0f)
                    : missed
                        ? ImVec4(
                              0.95f, 0.4f,
                              0.35f, 1.0f)
                    : next
                        ? ImVec4(
                              0.4f, 0.8f,
                              1.0f, 1.0f)
                        : ImVec4(
                              0.75f, 0.75f,
                              0.75f, 1.0f);
                ImGui::TextColored(
                    color,
                    "%s %s  [%s]",
                    stop->getCode().c_str(),
                    stop->getName().c_str(),
                    status);
            }
            ImGui::EndChild();
        }

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
