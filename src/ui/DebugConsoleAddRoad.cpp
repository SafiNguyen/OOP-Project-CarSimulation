#include "DebugConsole.h"

#include <algorithm>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "UiTheme.h"
#include "visualization/VisualizationEngine.h"

using debugconsole_detail::intersectionLabel;
using debugconsole_detail::nextFreeRoadId;

void DebugConsole::drawAddRoadPanel(Graph& graph, VisualizationEngine& visualization) {
    if (!ImGui::TreeNodeEx("Add Road", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    {
        std::string previewStart = addRoadStartId_ >= 0 ? ("#" + std::to_string(addRoadStartId_)) : "(none)";
        ImGui::Text("Start:");
        ImGui::SameLine();
        ImGui::TextColored(UiTheme::TextMuted, "%s", previewStart.c_str());
    }
    ImGui::SameLine();
    if (UiTheme::selectionButton(
            pickTarget_ == PickTarget::ADD_ROAD_START ? "Click map...##addroadstart"
                                                      : "Pick##addroadstart",
            pickTarget_ == PickTarget::ADD_ROAD_START)) {
        pickTarget_ = (pickTarget_ == PickTarget::ADD_ROAD_START) ? PickTarget::NONE : PickTarget::ADD_ROAD_START;
    }
    UiTheme::tooltip("Pick the start intersection directly on the map");

    {
        std::string previewEnd = addRoadEndId_ >= 0 ? ("#" + std::to_string(addRoadEndId_)) : "(none)";
        ImGui::Text("End:");
        ImGui::SameLine();
        ImGui::TextColored(UiTheme::TextMuted, "%s", previewEnd.c_str());
    }
    ImGui::SameLine();
    if (UiTheme::selectionButton(
            pickTarget_ == PickTarget::ADD_ROAD_END ? "Click map...##addroadend"
                                                    : "Pick##addroadend",
            pickTarget_ == PickTarget::ADD_ROAD_END)) {
        pickTarget_ = (pickTarget_ == PickTarget::ADD_ROAD_END) ? PickTarget::NONE : PickTarget::ADD_ROAD_END;
    }
    UiTheme::tooltip("Pick the end intersection directly on the map");

    ImGui::Checkbox("Auto distance (from coordinates)", &addRoadAutoDistance_);
    if (!addRoadAutoDistance_) {
        ImGui::InputFloat("Distance", &addRoadDistance_);
        addRoadDistance_ = std::max(1.0f, addRoadDistance_);
    }
    ImGui::InputFloat("Speed limit", &addRoadSpeedLimit_);
    addRoadSpeedLimit_ = std::max(1.0f, addRoadSpeedLimit_);
    ImGui::InputInt("Lanes", &addRoadLanes_);
    addRoadLanes_ = std::max(1, addRoadLanes_);
    ImGui::Checkbox("Two-way", &addRoadTwoWay_);

    const Intersection* selectedStart = graph.getIntersection(addRoadStartId_);
    const Intersection* selectedEnd = graph.getIntersection(addRoadEndId_);
    const bool canCreate = selectedStart != nullptr && selectedEnd != nullptr
        && selectedStart != selectedEnd;
    ImGui::BeginDisabled(!canCreate);
    if (UiTheme::actionButton("Create Road", ImVec2(132.0f, 36.0f))) {
        addRoadMessage_.clear();
        Intersection* start = graph.getIntersection(addRoadStartId_);
        Intersection* end = graph.getIntersection(addRoadEndId_);
        if (start == nullptr || end == nullptr) {
            addRoadMessage_ = "Pick both a start and an end intersection first.";
        } else if (start == end) {
            addRoadMessage_ = "Start and end must be different intersections.";
        } else {
            bool foundDuplicate = false;
            for (Road* existingRoad : graph.getAllRoads()) {
                if (existingRoad->getStart()->getId() == start->getId() &&
                    existingRoad->getEnd()->getId() == end->getId()) {
                    addRoadMessage_ = "Error: Road from " + std::to_string(start->getId()) + " to " + std::to_string(end->getId()) + " already exists!";
                    setNotice(NoticeTone::ERROR, addRoadMessage_);
                    foundDuplicate = true;
                    break;
                }
                if (addRoadTwoWay_ &&
                    existingRoad->getStart()->getId() == end->getId() &&
                    existingRoad->getEnd()->getId() == start->getId()) {
                    addRoadMessage_ = "Error: Road from " + std::to_string(end->getId()) + " to " + std::to_string(start->getId()) + " already exists!";
                    setNotice(NoticeTone::ERROR, addRoadMessage_);
                    foundDuplicate = true;
                    break;
                }
            }

            if (!foundDuplicate) {
                const double distance = addRoadAutoDistance_
                    ? graph.calculateDistance(start->getId(), end->getId())
                    : static_cast<double>(addRoadDistance_);
                const int newId = nextFreeRoadId(graph);
                Road* road = new Road(newId, "Custom Road", start, end, distance, addRoadSpeedLimit_, 1.0, addRoadLanes_);
                graph.addRoad(road);
                if (addRoadTwoWay_) {
                    Road* revRoad = new Road(-newId, "Custom Road", end, start, distance, addRoadSpeedLimit_, 1.0, addRoadLanes_);
                    graph.addRoad(revRoad);
                }
                visualization.prepare(graph);
                snapshotDirty_ = true;
                addRoadMessage_ = "Road #" + std::to_string(newId) + " created ("
                    + std::to_string(start->getId()) + " -> " + std::to_string(end->getId()) + ").";
                setNotice(NoticeTone::SUCCESS, addRoadMessage_);
            }
        }
    }
    ImGui::EndDisabled();
    if (!canCreate) {
        UiTheme::tooltip("Select two different intersections before creating a road");
    } else {
        UiTheme::tooltip("Create the configured road and refresh the map");
    }
    if (!addRoadMessage_.empty()) {
        const bool success = addRoadMessage_.rfind("Road #", 0) == 0;
        ImGui::TextColored(success ? UiTheme::Success : UiTheme::Error,
                           "%s", addRoadMessage_.c_str());
    }
    ImGui::TreePop();
}
