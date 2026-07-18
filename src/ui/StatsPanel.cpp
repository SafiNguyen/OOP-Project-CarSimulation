#include "StatsPanel.h"

#include <cfloat>

#include <imgui.h>

void StatsPanel::draw(const StatisticsSummary& summary, sf::Vector2u windowSize) {

    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(windowSize.x) - 320.0f, 20.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300.0f, static_cast<float>(windowSize.y) * 0.5f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSizeConstraints(ImVec2(220.0f, 90.0f), ImVec2(FLT_MAX, FLT_MAX));

    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.08f, 0.08f, 0.08f, 0.80f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.40f, 0.40f, 0.40f, 0.85f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 8.0f));

    ImGui::Begin("Traffic Simulation Stats", nullptr, ImGuiWindowFlags_NoCollapse);

    if (ImGui::Button(collapsed_ ? "Expand" : "Hide")) {
        collapsed_ = !collapsed_;
    }

    if (!collapsed_) {
        ImGui::Separator();

        // Scrollable body: a long per-algorithm breakdown scrolls instead
        // of being clipped or forcing the window to grow.
        ImGui::BeginChild("##stats_panel_body", ImVec2(0.0f, 0.0f), false);

        ImGui::Text("Simulated Time: %.1f s", summary.totalSimulatedTime);
        ImGui::Text("Vehicles Tracked: %d", summary.totalVehiclesTracked);
        ImGui::Text("Completed Trips: %d", summary.totalCompletedTrips);
        ImGui::Text("Recalculations: %d", summary.totalRecalculations);

        for (const auto& metric : summary.perAlgorithm) {
            ImGui::Separator();
            ImGui::TextColored(ImVec4(0.85f, 0.90f, 1.0f, 1.0f), "%s", metric.algorithmName.c_str());
            ImGui::Text("Calls: %d", metric.callCount);
            ImGui::Text("Found: %d / Not Found: %d", metric.pathsFound, metric.pathsNotFound);
            ImGui::Text("Avg Time: %.2f ms", metric.averageComputeTimeMs());
            ImGui::Text("Avg Nodes: %.0f", metric.averageNodesExplored());
            ImGui::Text("Avg Cost: %.2f", metric.averagePathCost());
        }

        ImGui::EndChild();
    }

    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor(2);
}