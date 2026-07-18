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
    if (simulator && !simulator->getFailedRecalcIds().empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f),
            "%zu vehicle(s) could not find a new route (marked in red on the map).",
            simulator->getFailedRecalcIds().size());
    }
    ImGui::TreePop();
}
