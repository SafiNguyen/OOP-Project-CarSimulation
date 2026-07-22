#include "DebugConsole.h"

#include <algorithm>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"
#include "visualization/VisualizationEngine.h"

using debugconsole_detail::intersectionLabel;
using debugconsole_detail::nextFreeRoadId;

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
            bool foundDuplicate = false;
            for (Road* existingRoad : graph.getAllRoads()) {
                if (existingRoad->getStart()->getId() == start->getId() &&
                    existingRoad->getEnd()->getId() == end->getId()) {
                    addRoadMessage_ = "Error: Road from " + std::to_string(start->getId()) + " to " + std::to_string(end->getId()) + " already exists!";
                    foundDuplicate = true;
                    break;
                }
                if (addRoadTwoWay_ &&
                    existingRoad->getStart()->getId() == end->getId() &&
                    existingRoad->getEnd()->getId() == start->getId()) {
                    addRoadMessage_ = "Error: Road from " + std::to_string(end->getId()) + " to " + std::to_string(start->getId()) + " already exists!";
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
            }
        }
    }
    if (!addRoadMessage_.empty()) {
        ImGui::TextColored(ImVec4(0.75f, 0.90f, 0.75f, 1.0f), "%s", addRoadMessage_.c_str());
    }
    ImGui::TreePop();
}
