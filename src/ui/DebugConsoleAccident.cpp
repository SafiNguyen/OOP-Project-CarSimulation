#include "DebugConsole.h"

#include <algorithm>
#include <memory>
#include <random>

#include <imgui.h>

#include "model/Road.h"
#include "simulation/TrafficEvent.h"
#include "simulation/TrafficSimulator.h"

void DebugConsole::drawAccidentPanel(std::unique_ptr<TrafficSimulator>& simulator) {
    if (!ImGui::TreeNodeEx("Trigger Event", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    const char* eventTypeNames[3] = { "Accident (Block Lane)", "Congestion", "Road Closure (Block All Lanes)" };
    ImGui::Combo("Event Type", &eventTypeIdx_, eventTypeNames, 3);

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
    
    if (eventTypeIdx_ == 0) {
        ImGui::InputInt("Lane Index (-1=Random)", &accidentLaneIdx_);
        accidentLaneIdx_ = std::max(-1, accidentLaneIdx_);
    }
    
    if (eventTypeIdx_ == 1) { // Congestion
        ImGui::InputFloat("Severity", &accidentSeverity_);
        accidentSeverity_ = std::max(1.1f, accidentSeverity_);
    }

    if (ImGui::Button("Trigger Event")) {
        if (simulator && !roadsSnapshot_.empty()) {
            Road* r = nullptr;
            std::mt19937 rng(std::random_device{}());
            if (accidentRoadIdx_ >= 0 && accidentRoadIdx_ < static_cast<int>(roadsSnapshot_.size())) {
                r = roadsSnapshot_[accidentRoadIdx_];
            } else {
                std::uniform_int_distribution<size_t> dist(0, roadsSnapshot_.size() - 1);
                r = roadsSnapshot_[dist(rng)];
            }
            
            std::unique_ptr<TrafficEvent> te;
            if (eventTypeIdx_ == 0) { // Accident
                int targetLane = accidentLaneIdx_;
                if (targetLane < 0 || targetLane >= r->getLaneCount()) {
                    std::uniform_int_distribution<int> distLane(0, std::max(0, r->getLaneCount() - 1));
                    targetLane = distLane(rng);
                }
                te = std::make_unique<AccidentEvent>(r->getId(), static_cast<double>(accidentDuration_), targetLane);
            } else if (eventTypeIdx_ == 1) { // Congestion
                te = std::make_unique<CongestionEvent>(r->getId(), static_cast<double>(accidentDuration_), static_cast<double>(accidentSeverity_));
            } else { // Road Closure
                te = std::make_unique<RoadClosureEvent>(r->getId(), static_cast<double>(accidentDuration_));
            }
            simulator->triggerEvent(std::move(te));
        }
    }
    ImGui::TreePop();
}
