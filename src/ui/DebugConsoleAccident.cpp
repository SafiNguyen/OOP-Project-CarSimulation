#include "DebugConsole.h"

#include <algorithm>
#include <memory>
#include <random>

#include <imgui.h>

#include "model/Road.h"
#include "simulation/TrafficEvent.h"
#include "simulation/TrafficSimulator.h"

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
