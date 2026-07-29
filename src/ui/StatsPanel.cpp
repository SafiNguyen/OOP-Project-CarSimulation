#include "StatsPanel.h"

#include <cstdio>
#include <string>

#include <imgui.h>

#include "UiTheme.h"

namespace {

std::string formatSimTime(double totalSeconds) {
    const int seconds = static_cast<int>(totalSeconds);
    const int hours = seconds / 3600;
    const int minutes = (seconds % 3600) / 60;
    const int remainder = seconds % 60;
    char buffer[32];
    if (hours > 0) {
        std::snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, remainder);
    } else {
        std::snprintf(buffer, sizeof(buffer), "%02d:%02d", minutes, remainder);
    }
    return buffer;
}

void statCard(const char* id, const char* label, const std::string& value,
              const ImVec4& accent = UiTheme::AccentStrong) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, UiTheme::SurfaceRaised);
    ImGui::PushStyleColor(ImGuiCol_Border, UiTheme::Border);
    ImGui::BeginChild(id, ImVec2(0.0f, 68.0f), true,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::TextColored(UiTheme::TextMuted, "%s", label);
    ImGui::TextColored(accent, "%s", value.c_str());
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
}

void metricCard(const char* id, const char* label, const char* format, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), format, value);
    statCard(id, label, buffer);
}

} // namespace

void StatsPanel::drawOverview(const StatisticsSummary& summary) {
    const int columns = ImGui::GetContentRegionAvail().x >= 360.0f ? 2 : 1;
    if (ImGui::BeginTable("##overview_stat_cards", columns,
                          ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoSavedSettings)) {
        ImGui::TableNextColumn();
        statCard("##sim_time_card", "SIMULATION TIME", formatSimTime(summary.totalSimulatedTime));
        ImGui::TableNextColumn();
        statCard("##tracked_card", "VEHICLES TRACKED", std::to_string(summary.totalVehiclesTracked));
        ImGui::TableNextColumn();
        statCard("##completed_card", "COMPLETED TRIPS", std::to_string(summary.totalCompletedTrips),
                 UiTheme::Success);
        ImGui::TableNextColumn();
        statCard("##recalc_card", "ROUTE RECALCULATIONS",
                 std::to_string(summary.totalRecalculations), UiTheme::Warning);
        ImGui::TableNextColumn();
        statCard(
            "##ped_active_card",
            "ACTIVE PEDESTRIANS",
            std::to_string(
                summary.activePedestrians));
        ImGui::TableNextColumn();
        statCard(
            "##ped_waiting_card",
            "WAITING / CROSSING",
            std::to_string(
                summary.waitingPedestrians) +
                " / " +
                std::to_string(
                    summary.crossingPedestrians),
            UiTheme::Warning);
        ImGui::TableNextColumn();
        statCard(
            "##ped_completed_card",
            "PEDESTRIAN TRIPS",
            std::to_string(
                summary.completedPedestrianTrips),
            UiTheme::Success);
        ImGui::TableNextColumn();
        metricCard(
            "##ped_wait_card",
            "AVG CROSSING WAIT",
            "%.1f s",
            summary.
                averagePedestrianWaitSeconds);
        ImGui::EndTable();
    }
}

void StatsPanel::drawPerformance(const StatisticsSummary& summary) {
    if (summary.perAlgorithm.empty()) {
        ImGui::TextColored(UiTheme::TextMuted,
                           "No pathfinding samples have been recorded yet.");
        ImGui::TextWrapped("Statistics will appear here after vehicles request or recalculate routes.");
        return;
    }

    for (std::size_t index = 0; index < summary.perAlgorithm.size(); ++index) {
        const AlgorithmMetric& metric = summary.perAlgorithm[index];
        ImGui::PushID(static_cast<int>(index));
        ImGui::TextColored(UiTheme::AccentStrong, "%s", metric.algorithmName.c_str());
        ImGui::SameLine();
        const float successRate = metric.callCount > 0
            ? static_cast<float>(metric.pathsFound) / static_cast<float>(metric.callCount)
            : 0.0f;
        ImGui::TextColored(UiTheme::TextMuted, "  %.0f%% success", successRate * 100.0f);

        const int columns = ImGui::GetContentRegionAvail().x >= 360.0f ? 3 : 2;
        if (ImGui::BeginTable("##algorithm_metrics", columns,
                              ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_NoSavedSettings)) {
            ImGui::TableNextColumn();
            statCard("##calls", "CALLS", std::to_string(metric.callCount));
            ImGui::TableNextColumn();
            statCard("##found", "FOUND", std::to_string(metric.pathsFound), UiTheme::Success);
            ImGui::TableNextColumn();
            statCard("##not_found", "NOT FOUND", std::to_string(metric.pathsNotFound),
                     metric.pathsNotFound > 0 ? UiTheme::Error : UiTheme::Text);
            ImGui::TableNextColumn();
            metricCard("##avg_time", "AVG SEARCH", "%.2f ms", metric.averageComputeTimeMs());
            ImGui::TableNextColumn();
            metricCard("##avg_nodes", "AVG NODES", "%.0f", metric.averageNodesExplored());
            ImGui::TableNextColumn();
            metricCard("##avg_cost", "AVG COST", "%.2f", metric.averagePathCost());
            ImGui::EndTable();
        }
        if (index + 1 < summary.perAlgorithm.size()) {
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }
        ImGui::PopID();
    }
}
