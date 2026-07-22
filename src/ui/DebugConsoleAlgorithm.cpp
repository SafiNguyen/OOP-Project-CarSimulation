#include "DebugConsole.h"

#include <imgui.h>

#include "simulation/TrafficSimulator.h"

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

    // Distance <-> speed blend, shared by Dijkstra and A*. BFS is
    // hop-count-only by design and ignores this slider entirely.
    const bool sliderAppliesToCurrent = (selectedAlgorithmIdx_ != 0);
    if (!sliderAppliesToCurrent) {
        ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
            "BFS ignores distance/speed (fewest-roads only). Switch to "
            "Dijkstra or A* to use the slider below.");
    }
    ImGui::BeginDisabled(!sliderAppliesToCurrent);
    if (ImGui::SliderFloat("Route preference", &speedPreference_, 0.0f, 1.0f,
                            speedPreference_ < 0.01f ? "Shortest distance"
                            : speedPreference_ > 0.99f ? "Fastest travel time"
                            : "%.2f (blend)")) {
        aStar_.setSpeedPreference(speedPreference_);
        dijkstraStrategy_.setSpeedPreference(speedPreference_);
        if (simulator && sliderAppliesToCurrent) {
            // Same strategy object/pointer as before, but its internal
            // weight changed - force every vehicle to recompute its route
            // against the new cost function so the effect is visible
            // immediately instead of only affecting brand-new vehicles.
            simulator->setPathFindingStrategy(currentStrategy());
        }
    }
    ImGui::EndDisabled();
    ImGui::TextWrapped(
        "0.0 = shortest distance only. 1.0 = fastest travel time "
        "(accounts for each road's speed limit and current congestion). "
        "Values in between blend both.");

    if (simulator && !simulator->getFailedRecalcIds().empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f),
            "%zu vehicle(s) could not find a new route (marked in red on the map).",
            simulator->getFailedRecalcIds().size());
    }
    ImGui::TreePop();
}
