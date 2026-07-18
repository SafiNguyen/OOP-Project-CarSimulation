#include "DebugConsole.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <random>
#include <sstream>

#include <imgui.h>

#include "mapload.h"
#include "model/Car.h"
#include "model/Bus.h"
#include "model/Motorbike.h"
#include "model/EmergencyVehicle.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "model/Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/TrafficEvent.h"
#include "visualization/VisualizationEngine.h"

namespace {

int nextFreeRoadId(const Graph& graph) {
    int maxId = 0;
    for (Road* r : graph.getAllRoads()) {
        maxId = std::max(maxId, std::abs(r->getId()));
    }
    return maxId + 1;
}

int nextFreeVehicleId(TrafficSimulator* simulator) {
    int maxId = 0;
    if (simulator) {
        for (Vehicle* v : simulator->getVehicles()) {
            maxId = std::max(maxId, v->getId());
        }
        for (Vehicle* v : simulator->getFinishedVehicles()) {
            maxId = std::max(maxId, v->getId());
        }
    }
    return maxId + 1;
}

Intersection* pickIntersectionNear(const Graph& graph,
                                    const VisualizationEngine& visualization,
                                    const sf::Vector2f& screenPos,
                                    float pickRadiusPixels = 20.0f) {
    Intersection* best = nullptr;
    float bestDistSq = pickRadiusPixels * pickRadiusPixels;

    for (Intersection* candidate : graph.getAllIntersections()) {
        const sf::Vector2f p = visualization.worldToScreen(candidate->getX(), candidate->getY());
        const float dx = p.x - screenPos.x;
        const float dy = p.y - screenPos.y;
        const float distSq = dx * dx + dy * dy;
        if (distSq <= bestDistSq) {
            bestDistSq = distSq;
            best = candidate;
        }
    }
    return best;
}

std::string intersectionLabel(Intersection* it) {
    return "#" + std::to_string(it->getId());
}

} // namespace

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

    // A regular titlebar: draggable and resizable from any edge/corner
    // (no NoMove/NoResize, unlike the old pinned bar). NoCollapse is
    // intentional - collapsing is handled by our own Hide/Expand button
    // below instead of ImGui's built-in arrow, so there's exactly one
    // collapse mechanism instead of two fighting each other.
    ImGui::Begin("Debug Console", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::Button(collapsed_ ? "Expand body" : "Hide body")) {
        collapsed_ = !collapsed_;
    }

    if (!collapsed_) {
        ImGui::Separator();

        // Scrollable body: whatever doesn't fit in the current window height
        // scrolls instead of being clipped or forcing the window to grow.
        ImGui::BeginChild("##debug_console_body", ImVec2(0.0f, 0.0f), false);

        drawTopBar(window, simulator, view, zoomFactor, heatMapEnabled, mapPathInput, usingDemoMap, loadError);
        ImGui::Separator();
        drawAddRoadPanel(graph_, visualization_);
        ImGui::Separator();
        drawSpawnVehiclePanel(simulator);
        ImGui::Separator();
        drawAccidentPanel(simulator);
        ImGui::Separator();
        drawAlgorithmPanel(simulator);

        ImGui::EndChild();
    }

    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void DebugConsole::drawTopBar(sf::RenderWindow& window,
                               std::unique_ptr<TrafficSimulator>& simulator,
                               sf::View& view,
                               float& zoomFactor,
                               bool& heatMapEnabled,
                               std::string& mapPathInput,
                               bool usingDemoMap,
                               const std::string& loadError) {
    static std::array<char, 1024> mapPathBuffer{};
    std::copy_n(mapPathInput.begin(), std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1), mapPathBuffer.begin());
    mapPathBuffer[std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1)] = '\0';
    ImGui::InputText("Map path", mapPathBuffer.data(), mapPathBuffer.size());
    mapPathInput = mapPathBuffer.data();

    if (ImGui::Button("Load map")) {
        std::string chosenPath;
        if (openFileDialog_ && openFileDialog_(chosenPath)) {
            mapPathInput = chosenPath;
        }
        loadAndRefresh_(mapPathInput);
        simulator = resetSimulation_();
        onMapChanged();
    }
    ImGui::SameLine();
    if (ImGui::Button("Demo map")) {
        loadAndRefresh_("");
        simulator = resetSimulation_();
        onMapChanged();
    }
    ImGui::SameLine();
    if (ImGui::Button(simulator && simulator->isPaused() ? "Resume" : "Pause")) {
        if (simulator) {
            if (simulator->isPaused()) simulator->resume();
            else simulator->pause();
        }
    }

    if (ImGui::Button("Reset view")) {
        zoomFactor = 1.0f;
        view = window.getDefaultView();
        clampViewToMap_();
    }
    ImGui::SameLine();
    if (ImGui::Button(heatMapEnabled ? "Heat: ON" : "Heat: OFF")) {
        heatMapEnabled = !heatMapEnabled;
        visualization_.setHeatMapEnabled(heatMapEnabled);
    }

    ImGui::TextWrapped("Status: %s", usingDemoMap ? "Demo map is active." : (mapPathInput.empty() ? "No map selected." : mapPathInput.c_str()));
    ImGui::Text("Intersections: %zu | Roads: %zu | Vehicles: %zu",
                intersectionsSnapshot_.size(), roadsSnapshot_.size(),
                simulator ? simulator->getVehicles().size() : 0u);

    if (!loadError.empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "Warning: %s", loadError.c_str());
    }
    if (intersectionsSnapshot_.empty() || roadsSnapshot_.empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "The current scene is missing intersections or roads.");
    }
    if (simulator && simulator->getVehicles().empty()) {
        ImGui::TextColored(ImVec4(0.90f, 0.90f, 0.65f, 1.0f), "No vehicles are currently active.");
    }
}

void DebugConsole::drawAddRoadPanel(Graph& graph, VisualizationEngine& visualization) {
    if (!ImGui::TreeNodeEx("Add Road", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    {
        std::string previewStart = addRoadStartId_ >= 0 ? ("#" + std::to_string(addRoadStartId_)) : "(none)";
        if (ImGui::BeginCombo("Start##addroad", previewStart.c_str())) {
            for (Intersection* it : intersectionsSnapshot_) {
                bool selected = (it->getId() == addRoadStartId_);
                if (ImGui::Selectable(intersectionLabel(it).c_str(), selected)) {
                    addRoadStartId_ = it->getId();
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(pickTarget_ == PickTarget::ADD_ROAD_START ? "Click map..." : "Pick##addroadstart")) {
        pickTarget_ = (pickTarget_ == PickTarget::ADD_ROAD_START) ? PickTarget::NONE : PickTarget::ADD_ROAD_START;
    }

    {
        std::string previewEnd = addRoadEndId_ >= 0 ? ("#" + std::to_string(addRoadEndId_)) : "(none)";
        if (ImGui::BeginCombo("End##addroad", previewEnd.c_str())) {
            for (Intersection* it : intersectionsSnapshot_) {
                bool selected = (it->getId() == addRoadEndId_);
                if (ImGui::Selectable(intersectionLabel(it).c_str(), selected)) {
                    addRoadEndId_ = it->getId();
                }
            }
            ImGui::EndCombo();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button(pickTarget_ == PickTarget::ADD_ROAD_END ? "Click map..." : "Pick##addroadend")) {
        pickTarget_ = (pickTarget_ == PickTarget::ADD_ROAD_END) ? PickTarget::NONE : PickTarget::ADD_ROAD_END;
    }

    ImGui::Checkbox("Auto distance (from coordinates)", &addRoadAutoDistance_);
    if (!addRoadAutoDistance_) {
        ImGui::InputFloat("Distance", &addRoadDistance_);
    }
    ImGui::InputFloat("Speed limit", &addRoadSpeedLimit_);
    ImGui::InputInt("Lanes", &addRoadLanes_);
    addRoadLanes_ = std::max(1, addRoadLanes_);
    ImGui::Checkbox("Two-way", &addRoadTwoWay_);

    if (ImGui::Button("Create Road")) {
        addRoadMessage_.clear();
        Intersection* start = graph.getIntersection(addRoadStartId_);
        Intersection* end = graph.getIntersection(addRoadEndId_);
        if (start == nullptr || end == nullptr) {
            addRoadMessage_ = "Pick both a start and an end intersection first.";
        } else if (start == end) {
            addRoadMessage_ = "Start and end must be different intersections.";
        } else {
            const double distance = addRoadAutoDistance_
                ? graph.calculateDistance(start->getId(), end->getId())
                : static_cast<double>(addRoadDistance_);
            const int newId = nextFreeRoadId(graph);
            Road* road = new Road(newId, start, end, distance, addRoadSpeedLimit_, 1.0, addRoadLanes_);
            graph.addRoad(road);
            if (addRoadTwoWay_) {
                Road* revRoad = new Road(-newId, end, start, distance, addRoadSpeedLimit_, 1.0, addRoadLanes_);
                graph.addRoad(revRoad);
            }
            visualization.prepare(graph);
            snapshotDirty_ = true;
            addRoadMessage_ = "Road #" + std::to_string(newId) + " created ("
                + std::to_string(start->getId()) + " -> " + std::to_string(end->getId()) + ").";
        }
    }
    if (!addRoadMessage_.empty()) {
        ImGui::TextColored(ImVec4(0.75f, 0.90f, 0.75f, 1.0f), "%s", addRoadMessage_.c_str());
    }
    ImGui::TreePop();
}

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
            const int newId = nextFreeVehicleId(simulator.get());
            Vehicle* v = nullptr;
            switch (spawnVehicleTypeIdx_) {
                case 0: v = new Car(newId, spawnVehicleSpeed_, start, end); break;
                case 1: v = new Bus(newId, spawnVehicleSpeed_, start, end); break;
                case 2: v = new Motorbike(newId, spawnVehicleSpeed_, start, end); break;
                default: v = new EmergencyVehicle(newId, spawnVehicleSpeed_, start, end); break;
            }
            if (simulator->addVehicle(v)) {
                spawnMessage_ = "Spawned vehicle #" + std::to_string(newId) + ".";
            } else {
                spawnMessage_ = "No path exists between those two points; vehicle was not spawned.";
            }
        }
    }
    if (!spawnMessage_.empty()) {
        ImGui::TextColored(ImVec4(0.75f, 0.90f, 0.75f, 1.0f), "%s", spawnMessage_.c_str());
    }
    ImGui::TreePop();
}

void DebugConsole::drawAccidentPanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Trigger Accident", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    std::string previewRoad = "Random road";
    if (accidentRoadIdx_ >= 0 && accidentRoadIdx_ < static_cast<int>(roadsSnapshot_.size())) {
        previewRoad = "Road #" + std::to_string(roadsSnapshot_[accidentRoadIdx_]->getId());
    }
    if (ImGui::BeginCombo("Road", previewRoad.c_str())) {
        bool randomSelected = (accidentRoadIdx_ < 0);
        if (ImGui::Selectable("Random road", randomSelected)) {
            accidentRoadIdx_ = -1;
        }
        for (int i = 0; i < static_cast<int>(roadsSnapshot_.size()); ++i) {
            bool selected = (accidentRoadIdx_ == i);
            std::string label = "Road #" + std::to_string(roadsSnapshot_[i]->getId())
                + " (" + std::to_string(roadsSnapshot_[i]->getStart()->getId())
                + " -> " + std::to_string(roadsSnapshot_[i]->getEnd()->getId()) + ")";
            if (ImGui::Selectable(label.c_str(), selected)) {
                accidentRoadIdx_ = i;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::InputFloat("Duration (s)", &accidentDuration_);
    accidentDuration_ = std::max(1.0f, accidentDuration_);

    if (ImGui::Button("Trigger Accident")) {
        if (simulator && !roadsSnapshot_.empty()) {
            Road* r = nullptr;
            if (accidentRoadIdx_ >= 0 && accidentRoadIdx_ < static_cast<int>(roadsSnapshot_.size())) {
                r = roadsSnapshot_[accidentRoadIdx_];
            } else {
                std::mt19937 rng(std::random_device{}());
                std::uniform_int_distribution<size_t> dist(0, roadsSnapshot_.size() - 1);
                r = roadsSnapshot_[dist(rng)];
            }
            auto te = std::make_unique<AccidentEvent>(r->getId(), static_cast<double>(accidentDuration_));
            simulator->triggerEvent(std::move(te));
        }
    }
    ImGui::TreePop();
}

void DebugConsole::drawAlgorithmPanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Pathfinding Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    const char* algorithmNames[3] = { "BFS (fewest roads)", "Dijkstra (congestion-aware)", "A* (speed-optimized)" };
    if (ImGui::Combo("Algorithm", &selectedAlgorithmIdx_, algorithmNames, 3)) {
        if (simulator) {
            simulator->setPathFindingStrategy(currentStrategy());
        }
    }
    if (simulator && !simulator->getFailedRecalcIds().empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f),
            "%zu vehicle(s) could not find a new route (marked in red on the map).",
            simulator->getFailedRecalcIds().size());
    }
    ImGui::TreePop();
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
