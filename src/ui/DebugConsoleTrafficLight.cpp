#include "DebugConsole.h"

#include <algorithm>
#include <string>

#include <imgui.h>

#include "DebugConsoleInternal.h"
#include "Graph.h"
#include "Intersection.h"
#include "Road.h"
#include "TrafficLight.h"
#include "UiTheme.h"

using debugconsole_detail::intersectionLabel;

void DebugConsole::drawTrafficLightPanel() {
    if (!ImGui::TreeNodeEx("Traffic Lights", ImGuiTreeNodeFlags_DefaultOpen)) {
        return;
    }

    // --- Intersection selector ---
    {
        std::string preview = "(none)";
        if (trafficLightIntersectionIdx_ >= 0
            && trafficLightIntersectionIdx_ < static_cast<int>(intersectionsSnapshot_.size())) {
            preview = intersectionLabel(intersectionsSnapshot_[trafficLightIntersectionIdx_]);
        }

        ImGui::Text("Intersection:");
        ImGui::SameLine();
        ImGui::TextColored(UiTheme::TextMuted, "%s", preview.c_str());
    }
    ImGui::SameLine();
    if (UiTheme::selectionButton(
            pickTarget_ == PickTarget::TRAFFIC_LIGHT_INTERSECTION ? "Click map...##tl"
                                                                  : "Pick##tl",
            pickTarget_ == PickTarget::TRAFFIC_LIGHT_INTERSECTION)) {
        pickTarget_ = (pickTarget_ == PickTarget::TRAFFIC_LIGHT_INTERSECTION) ? PickTarget::NONE : PickTarget::TRAFFIC_LIGHT_INTERSECTION;
    }
    UiTheme::tooltip("Pick the intersection directly on the map");

    if (trafficLightIntersectionIdx_ < 0
        || trafficLightIntersectionIdx_ >= static_cast<int>(intersectionsSnapshot_.size())) {
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "Select an intersection to manage its traffic lights.");
        ImGui::TreePop();
        return;
    }

    Intersection* intersection = intersectionsSnapshot_[trafficLightIntersectionIdx_];
    const auto& incomingRoads = intersection->getIncomingRoads();

    if (incomingRoads.empty()) {
        ImGui::TextColored(ImVec4(0.8f, 0.6f, 0.3f, 1.0f), "No incoming roads at this intersection.");
        ImGui::TreePop();
        return;
    }

    ImGui::Text("Incoming roads: %d", static_cast<int>(incomingRoads.size()));

    // --- Add All / Remove All buttons ---
    if (ImGui::Button("Add All Lights")) {
        for (Road* road : incomingRoads) {
            intersection->registerIncomingLight(road);
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Remove All Lights")) {
        for (Road* road : incomingRoads) {
            intersection->unregisterIncomingLight(road);
        }
    }

    ImGui::Separator();

    // --- Per-road toggle ---
    for (Road* road : incomingRoads) {
        const TrafficLight* light =
            intersection->getLightForIncomingRoad(road);
        bool hasLight = (light != nullptr);

        std::string roadLabel = "Road #" + std::to_string(road->getId());
        if (!road->getName().empty()) {
            roadLabel += " (" + road->getName() + ")";
        }
        roadLabel += " [" + std::to_string(road->getLaneCount()) + " lane"
                   + (road->getLaneCount() > 1 ? "s" : "") + "]";

        // Light state indicator
        if (hasLight) {
            ImVec4 stateColor;
            const char* stateLabel;
            switch (light->getState()) {
                case LightState::GREEN:
                    stateColor = ImVec4(0.3f, 0.85f, 0.35f, 1.0f);
                    stateLabel = "GREEN";
                    break;
                case LightState::YELLOW:
                    stateColor = ImVec4(0.95f, 0.8f, 0.2f, 1.0f);
                    stateLabel = "YELLOW";
                    break;
                case LightState::RED:
                default:
                    stateColor = ImVec4(0.9f, 0.25f, 0.25f, 1.0f);
                    stateLabel = "RED";
                    break;
            }
            ImGui::TextColored(stateColor, "[%s]", stateLabel);
            ImGui::SameLine();
        }

        ImGui::Text("%s", roadLabel.c_str());
        ImGui::SameLine();

        std::string btnId = hasLight
            ? ("Remove##tl_" + std::to_string(road->getId()))
            : ("Add##tl_" + std::to_string(road->getId()));

        if (hasLight) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.6f, 0.15f, 0.15f, 1.0f));
            if (ImGui::SmallButton(btnId.c_str())) {
                intersection->unregisterIncomingLight(road);
            }
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.5f, 0.15f, 1.0f));
            if (ImGui::SmallButton(btnId.c_str())) {
                intersection->registerIncomingLight(road);
            }
            ImGui::PopStyleColor();
        }
    }

    ImGui::TreePop();
}
