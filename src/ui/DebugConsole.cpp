#include "DebugConsole.h"

#include <algorithm>
#include <cmath>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Vehicle.h"
#include "simulation/StatisticsManager.h"
#include "simulation/TrafficSimulator.h"
#include "StatsPanel.h"
#include "UiTheme.h"
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
    mapPathBufferInitialized_ = false;
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
                         const std::string& loadError,
                         StatsPanel& statsPanel,
                         const StatisticsSummary* statistics,
                         float frameDt) {
    rebuildSnapshotsIfNeeded(graph_);
    const bool completeLoadAfterFrame = mapLoadPending_;
    if (!loadStatusInitialized_) {
        lastLoadFailed_ = !loadError.empty()
            && loadError.rfind("No map path supplied", 0) != 0;
        loadStatusInitialized_ = true;
    }

    if (frameDt > 0.0001f) {
        const float instantaneousFps = std::min(999.0f, 1.0f / frameDt);
        smoothedFps_ = smoothedFps_ * 0.90f + instantaneousFps * 0.10f;
    }
    noticeTimeRemaining_ = std::max(0.0f, noticeTimeRemaining_ - frameDt);

    if (drawerOpen_ && ImGui::IsKeyPressed(ImGuiKey_Escape)
        && !ImGui::IsAnyItemActive()) {
        drawerOpen_ = false;
        pickTarget_ = PickTarget::NONE;
    }

    drawTopHud(window, simulator, mapPathInput, usingDemoMap, statistics);
    drawBottomDock(window, simulator, view, zoomFactor, heatMapEnabled,
                   showParkedVehicles, mapPathInput, loadError);
    if (drawerOpen_) {
        drawDrawer(window, simulator, view, zoomFactor, heatMapEnabled,
                   showParkedVehicles, mapPathInput, usingDemoMap, loadError,
                   statsPanel, statistics);
    }
    drawToast(window);
    if (completeLoadAfterFrame) {
        completePendingMapLoad(simulator, mapPathInput, loadError);
    }
}

void DebugConsole::openDrawer(DrawerTab tab) {
    activeTab_ = tab;
    drawerOpen_ = true;
    drawerTabSelectionPending_ = true;
}

void DebugConsole::setNotice(NoticeTone tone, const std::string& message) {
    noticeTone_ = tone;
    noticeMessage_ = message;
    noticeTimeRemaining_ = 4.0f;
}

void DebugConsole::drawDrawer(sf::RenderWindow& window,
                              std::unique_ptr<TrafficSimulator>& simulator,
                              sf::View& view,
                              float& zoomFactor,
                              bool& heatMapEnabled,
                              bool& showParkedVehicles,
                              std::string& mapPathInput,
                              bool usingDemoMap,
                              const std::string& loadError,
                              StatsPanel& statsPanel,
                              const StatisticsSummary* statistics) {
    (void)view;
    (void)zoomFactor;
    const sf::Vector2u size = window.getSize();
    const float width = static_cast<float>(size.x);
    const float height = static_cast<float>(size.y);
    const bool narrow = width < 720.0f;
    const float topInset = width < 650.0f ? 60.0f : 66.0f;
    const float dockHeight = 64.0f;
    const float maxDrawerWidth = width >= 1000.0f ? 560.0f : 440.0f;
    const float drawerWidth = narrow ? width : std::min(maxDrawerWidth, width * 0.46f);
    const float drawerHeight = std::max(160.0f, height - topInset - dockHeight);

    ImGui::SetNextWindowPos(ImVec2(width - drawerWidth, topInset), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(drawerWidth, drawerHeight), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, UiTheme::Background);
    ImGui::PushStyleColor(ImGuiCol_Border, UiTheme::Border);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, narrow ? 0.0f : 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(16.0f, 13.0f));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar;

    if (ImGui::Begin("##control_drawer", &drawerOpen_, flags)) {
        ImGui::TextColored(UiTheme::AccentStrong, "CONTROL CENTER");
        ImGui::SameLine();
        ImGui::TextColored(UiTheme::TextMuted, "  Advanced tools & diagnostics");
        const float closeWidth = 34.0f;
        ImGui::SameLine(std::max(ImGui::GetCursorPosX(),
                                ImGui::GetWindowContentRegionMax().x - closeWidth));
        if (ImGui::Button("X##close_drawer", ImVec2(closeWidth, 30.0f))) {
            drawerOpen_ = false;
            pickTarget_ = PickTarget::NONE;
        }
        UiTheme::tooltip("Close control drawer (Esc)");

        ImGui::Separator();
        if (ImGui::BeginTabBar("##drawer_tabs",
                               ImGuiTabBarFlags_FittingPolicyScroll |
                               ImGuiTabBarFlags_NoCloseWithMiddleMouseButton)) {
            const bool applyProgrammaticSelection = drawerTabSelectionPending_;
            ImGuiTabItemFlags tabFlags =
                applyProgrammaticSelection && activeTab_ == DrawerTab::OVERVIEW
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Overview", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::OVERVIEW;
                ImGui::BeginChild("##overview_scroll", ImVec2(0.0f, 0.0f), false);
                drawOverviewTab(simulator, heatMapEnabled, showParkedVehicles,
                                mapPathInput, usingDemoMap, loadError,
                                statsPanel, statistics);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            tabFlags = applyProgrammaticSelection && activeTab_ == DrawerTab::PERFORMANCE
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Performance", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::PERFORMANCE;
                ImGui::BeginChild("##performance_scroll", ImVec2(0.0f, 0.0f), false);
                ImGui::TextColored(UiTheme::TextMuted, "PATHFINDING TELEMETRY");
                ImGui::Spacing();
                if (statistics != nullptr) {
                    statsPanel.drawPerformance(*statistics);
                } else {
                    ImGui::TextDisabled("No active statistics source.");
                }
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            tabFlags = applyProgrammaticSelection && activeTab_ == DrawerTab::MAP
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Map", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::MAP;
                ImGui::BeginChild("##map_scroll", ImVec2(0.0f, 0.0f), false);
                drawMapTab(simulator, mapPathInput, usingDemoMap, loadError);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            tabFlags = applyProgrammaticSelection && activeTab_ == DrawerTab::ROAD_TOOLS
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Road Tools", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::ROAD_TOOLS;
                ImGui::BeginChild("##road_tools_scroll", ImVec2(0.0f, 0.0f), false);
                ImGui::TextColored(UiTheme::TextMuted,
                                   "Choose endpoints from the lists or use Pick, then click an intersection on the map.");
                if (isPicking()) {
                    ImGui::Spacing();
                    ImGui::TextColored(UiTheme::Warning,
                                       "PICK MODE ACTIVE  -  click an intersection on the map");
                }
                ImGui::Spacing();
                drawAddRoadPanel(graph_, visualization_);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            tabFlags = applyProgrammaticSelection && activeTab_ == DrawerTab::SIMULATION
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Simulation", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::SIMULATION;
                ImGui::BeginChild("##simulation_scroll", ImVec2(0.0f, 0.0f), false);
                drawAlgorithmPanel(simulator);
                ImGui::Spacing();
                drawSpawnVehiclePanel(simulator);
                ImGui::EndChild();
                ImGui::EndTabItem();
            }

            tabFlags = applyProgrammaticSelection && activeTab_ == DrawerTab::DEBUG
                ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None;
            if (ImGui::BeginTabItem("Debug", nullptr, tabFlags)) {
                activeTab_ = DrawerTab::DEBUG;
                ImGui::BeginChild("##debug_scroll", ImVec2(0.0f, 0.0f), false);
                drawAccidentPanel(simulator);
                ImGui::Spacing();
                drawTrafficLightPanel();
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
            ImGui::EndTabBar();
            drawerTabSelectionPending_ = false;
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void DebugConsole::drawOverviewTab(std::unique_ptr<TrafficSimulator>& simulator,
                                   bool& heatMapEnabled,
                                   bool& showParkedVehicles,
                                   const std::string& mapPathInput,
                                   bool usingDemoMap,
                                   const std::string& loadError,
                                   StatsPanel& statsPanel,
                                   const StatisticsSummary* statistics) {
    ImGui::TextColored(UiTheme::TextMuted, "MAP STATUS");
    ImGui::Text("%s", usingDemoMap ? "Built-in demo map" :
                (mapPathInput.empty() ? "No map selected" : mapPathInput.c_str()));
    ImGui::Text("Intersections  %zu", intersectionsSnapshot_.size());
    ImGui::SameLine(200.0f);
    ImGui::Text("Roads  %zu", roadsSnapshot_.size());
    ImGui::Text("Active vehicles  %zu",
                simulator ? simulator->getVehicles().size() : 0u);
    ImGui::Text("Heatmap  %s", heatMapEnabled ? "ON" : "OFF");
    ImGui::SameLine(200.0f);
    ImGui::Text("Parked  %s", showParkedVehicles ? "ON" : "OFF");

    if (lastLoadFailed_ && !loadError.empty()) {
        ImGui::Spacing();
        ImGui::TextColored(UiTheme::Error, "MAP LOAD ERROR");
        ImGui::TextWrapped("%s", loadError.c_str());
        ImGui::TextColored(UiTheme::TextMuted,
                           "The built-in demo map is active so the simulation can continue.");
    }

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::TextColored(UiTheme::TextMuted, "SIMULATION SPEED");
    const double currentSpeed = simulator ? simulator->getSpeedMultiplier() : 1.0;
    const double speeds[] = {0.5, 1.0, 2.0, 4.0};
    const char* labels[] = {"0.5x", "1x", "2x", "4x"};
    ImGui::BeginDisabled(!simulator);
    for (int i = 0; i < 4; ++i) {
        if (i > 0) ImGui::SameLine();
        if (UiTheme::selectionButton(labels[i], std::abs(currentSpeed - speeds[i]) < 0.01,
                                     ImVec2(62.0f, 34.0f)) && simulator) {
            simulator->setSpeedMultiplier(speeds[i]);
        }
    }
    ImGui::EndDisabled();

    ImGui::Spacing();
    if (UiTheme::toggleButton("overview_heatmap", "Heatmap", heatMapEnabled,
                              ImVec2(126.0f, 34.0f))) {
        heatMapEnabled = !heatMapEnabled;
        visualization_.setHeatMapEnabled(heatMapEnabled);
    }
    UiTheme::tooltip("Toggle traffic-density heatmap");
    ImGui::SameLine();
    if (UiTheme::toggleButton("overview_parked", "Parked", showParkedVehicles,
                              ImVec2(126.0f, 34.0f))) {
        showParkedVehicles = !showParkedVehicles;
    }
    UiTheme::tooltip("Toggle completed vehicles near destinations");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::TextColored(UiTheme::TextMuted, "SESSION STATISTICS");
    ImGui::Spacing();
    if (statistics != nullptr) {
        statsPanel.drawOverview(*statistics);
    } else {
        ImGui::TextDisabled("No active statistics source.");
    }
}

void DebugConsole::drawToast(sf::RenderWindow& window) {
    if (noticeTimeRemaining_ <= 0.0f || noticeMessage_.empty()) {
        return;
    }

    const sf::Vector2u size = window.getSize();
    const float toastWidth = std::min(420.0f, static_cast<float>(size.x) - 24.0f);
    ImGui::SetNextWindowPos(
        ImVec2((static_cast<float>(size.x) - toastWidth) * 0.5f,
               static_cast<float>(size.x) < 650.0f ? 68.0f : 74.0f),
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(toastWidth, 0.0f), ImGuiCond_Always);

    ImVec4 accent = UiTheme::AccentStrong;
    if (noticeTone_ == NoticeTone::SUCCESS) accent = UiTheme::Success;
    if (noticeTone_ == NoticeTone::WARNING) accent = UiTheme::Warning;
    if (noticeTone_ == NoticeTone::ERROR) accent = UiTheme::Error;

    ImGui::PushStyleColor(ImGuiCol_WindowBg, UiTheme::SurfaceRaised);
    ImGui::PushStyleColor(ImGuiCol_Border, accent);
    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_NoNav;
    if (ImGui::Begin("##hud_notice", nullptr, flags)) {
        ImGui::TextColored(accent, "%s", noticeTone_ == NoticeTone::ERROR ? "ERROR" :
                          noticeTone_ == NoticeTone::WARNING ? "NOTICE" :
                          noticeTone_ == NoticeTone::SUCCESS ? "SUCCESS" : "INFO");
        ImGui::SameLine();
        ImGui::TextWrapped("%s", noticeMessage_.c_str());
    }
    ImGui::End();
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
