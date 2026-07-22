#include "DebugConsole.h"

#include <algorithm>
#include <cfloat>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/VisualizationEngine.h"

using debugconsole_detail::pickIntersectionNear;

DebugConsole::DebugConsole(Graph& graph,
                           VisualizationEngine& visualization,
                           LoadMapFn loadAndRefresh,
                           ResetSimulationFn resetSimulation,
                           ClampViewFn clampViewToMap,
                           FileDialogFn openFileDialog)
    : graph_(graph),
      visualization_(visualization),
      loadAndRefresh_(std::move(loadAndRefresh)),
      resetSimulation_(std::move(resetSimulation)),
      clampViewToMap_(std::move(clampViewToMap)),
      openFileDialog_(std::move(openFileDialog)) {
}

bool DebugConsole::isPicking() const {
    return pickTarget_ != PickTarget::NONE;
}

void DebugConsole::onMapChanged() {
    pickTarget_ = PickTarget::NONE;
    addRoadStartId_ = -1;
    addRoadEndId_ = -1;
    addRoadMessage_.clear();
    spawnStartId_ = -1;
    spawnEndId_ = -1;
    spawnMessage_.clear();
    accidentRoadIdx_ = -1;
    snapshotDirty_ = true;
}

void DebugConsole::handleMapClick(const Graph& graph,
                                   const VisualizationEngine& visualization,
                                   const sf::Vector2f& worldPos) {
    if (pickTarget_ == PickTarget::NONE) {
        return;
    }

    Intersection* picked = pickIntersectionNear(graph, visualization, worldPos);
    if (picked != nullptr) {
        switch (pickTarget_) {
            case PickTarget::ADD_ROAD_START: addRoadStartId_ = picked->getId(); break;
            case PickTarget::ADD_ROAD_END:   addRoadEndId_ = picked->getId(); break;
            case PickTarget::SPAWN_START:    spawnStartId_ = picked->getId(); break;
            case PickTarget::SPAWN_END:      spawnEndId_ = picked->getId(); break;
            default: break;
        }
    }
    pickTarget_ = PickTarget::NONE;
}

PathFindingStrategy* DebugConsole::getSelectedStrategy() {
    return currentStrategy();
}

PathFindingStrategy* DebugConsole::currentStrategy() {
    switch (selectedAlgorithmIdx_) {
        case 0: return &bfsStrategy_;
        case 1: return &dijkstraStrategy_;
        default: return &aStar_;
    }
}

void DebugConsole::rebuildSnapshotsIfNeeded(const Graph& graph) {
    const size_t curIntersections = graph.getAllIntersections().size();
    const size_t curRoads = graph.getAllRoads().size();

    if (!snapshotDirty_ && curIntersections == lastKnownIntersectionCount_
        && curRoads == lastKnownRoadCount_) {
        return; // graph hasn't changed shape since last frame - reuse cached snapshots
    }

    intersectionsSnapshot_ = graph.getAllIntersections();
    std::sort(intersectionsSnapshot_.begin(), intersectionsSnapshot_.end(),
              [](Intersection* a, Intersection* b) { return a->getId() < b->getId(); });

    roadsSnapshot_ = graph.getAllRoads();
    std::sort(roadsSnapshot_.begin(), roadsSnapshot_.end(),
              [](Road* a, Road* b) { return a->getId() < b->getId(); });

    lastKnownIntersectionCount_ = curIntersections;
    lastKnownRoadCount_ = curRoads;
    snapshotDirty_ = false;

    // The accident road index refers into roadsSnapshot_ by position; if the
    // road count changed, treat any previous selection as stale.
    accidentRoadIdx_ = -1;
}

void DebugConsole::draw(sf::RenderWindow& window,
                         std::unique_ptr<TrafficSimulator>& simulator,
                         sf::View& view,
                         float& zoomFactor,
                         bool& heatMapEnabled,
                         bool& showParkedVehicles,
                         std::string& mapPathInput,
                         bool usingDemoMap,
                         const std::string& loadError) {
    rebuildSnapshotsIfNeeded(graph_);

    const sf::Vector2u winSize = window.getSize();

    // Only applied the first time this window is ever shown - after that the
    // user's own move/resize takes over completely, exactly like any other
    // desktop or game HUD window.
    ImGui::SetNextWindowPos(ImVec2(20.0f, static_cast<float>(winSize.y) * 0.55f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::min(480.0f, static_cast<float>(winSize.x) - 40.0f),
                                     static_cast<float>(winSize.y) * 0.40f),
                              ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(280.0f, 90.0f), ImVec2(FLT_MAX, FLT_MAX));

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.06f, 0.07f, 0.10f, 0.92f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.30f, 0.35f, 0.50f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 8.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f, 5.0f));

    // Plain titlebar - no NoCollapse flag - so ImGui draws its own collapse
    // arrow and handles the collapse/expand state itself, exactly like
    // StatsPanel. That replaces the previous custom "Hide body"/"Expand
    // body" button, so there's a single, familiar collapse control.
    if (ImGui::Begin("Debug Console")) {
        // Scrollable body: whatever doesn't fit in the current window height
        // scrolls instead of being clipped or forcing the window to grow.
        ImGui::BeginChild("##debug_console_body", ImVec2(0.0f, 0.0f), false);

        drawTopBar(window, simulator, view, zoomFactor, heatMapEnabled, showParkedVehicles, mapPathInput, usingDemoMap, loadError);
        ImGui::Separator();
        drawAddRoadPanel(graph_, visualization_);
        ImGui::Separator();
        drawSpawnVehiclePanel(simulator);
        ImGui::Separator();
        drawAccidentPanel(simulator);
        ImGui::Separator();
        drawAlgorithmPanel(simulator);
        ImGui::Separator();
        drawTrafficLightPanel();
        ImGui::EndChild();
    }
    ImGui::End();

    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void DebugConsole::drawFailedRecalcMarkers(sf::RenderWindow& window,
                                            TrafficSimulator* simulator,
                                            const VisualizationEngine& visualization) const {
    if (simulator == nullptr) {
        return;
    }
    const auto& failedRecalcIds = simulator->getFailedRecalcIds();
    if (failedRecalcIds.empty()) {
        return;
    }

    for (Vehicle* v : simulator->getVehicles()) {
        if (failedRecalcIds.count(v->getId()) == 0 || v->getCurrentRoad() == nullptr) {
            continue;
        }
        const double ratio = v->getProgressRatio();
        const Intersection* rs = v->getCurrentRoad()->getStart();
        const Intersection* re = v->getCurrentRoad()->getEnd();
        const double wx = rs->getX() + ratio * (re->getX() - rs->getX());
        const double wy = rs->getY() + ratio * (re->getY() - rs->getY());
        const sf::Vector2f pos = visualization.worldToScreen(wx, wy);

        sf::CircleShape warnDot(5.0f, 3);
        warnDot.setOrigin(5.0f, 5.0f);
        warnDot.setPosition(pos.x, pos.y - 18.0f);
        warnDot.setFillColor(sf::Color(230, 30, 30));
        warnDot.setOutlineThickness(1.0f);
        warnDot.setOutlineColor(sf::Color::White);
        window.draw(warnDot);
    }
}
