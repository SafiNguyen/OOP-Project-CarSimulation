#include "DebugConsole.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <imgui.h>

#include "simulation/StatisticsManager.h"
#include "simulation/TrafficSimulator.h"
#include "simulation/TimePlaybackController.h"
#include "UiTheme.h"
#include "visualization/VisualizationEngine.h"

namespace {

std::string formatTime(double totalSeconds, bool compact) {
    const int seconds = static_cast<int>(totalSeconds);
    const int hours = seconds / 3600;
    const int minutes = (seconds % 3600) / 60;
    const int remainder = seconds % 60;
    char buffer[32];
    if (hours > 0 && !compact) {
        std::snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, remainder);
    } else {
        std::snprintf(buffer, sizeof(buffer), "%02d:%02d", minutes + hours * 60, remainder);
    }
    return buffer;
}

std::string formatTimeDelta(double totalSeconds) {
    const bool negative = totalSeconds < 0.0;
    const std::string formatted = formatTime(std::abs(totalSeconds), false);
    return negative ? "-" + formatted : "+" + formatted;
}

std::string rewindButtonLabel(const TimePlaybackController* controller) {
    if (controller == nullptr || controller->available() < 2u) {
        return "<<";
    }

    const std::size_t currentIndex = controller->currentIndex();
    const std::size_t fromIndex =
        currentIndex == SnapshotManager::npos
            ? controller->available() - 1u
            : currentIndex;
    if (fromIndex == 0u) {
        return "<<";
    }

    const double delta =
        controller->timeAt(fromIndex) - controller->timeAt(fromIndex - 1u);
    return std::string("<< ") + formatTimeDelta(delta);
}

std::string forwardButtonLabel(const TimePlaybackController* controller) {
    if (controller == nullptr || controller->available() < 2u) {
        return ">>";
    }

    const std::size_t currentIndex = controller->currentIndex();
    if (currentIndex == SnapshotManager::npos ||
        currentIndex >= controller->available() - 1u) {
        return ">>";
    }

    const double delta =
        controller->timeAt(currentIndex + 1u) - controller->timeAt(currentIndex);
    return formatTimeDelta(delta) + " >>";
}

std::string shortMapName(const std::string& path) {
    if (path.empty()) return "NO MAP";
    const std::size_t slash = path.find_last_of("/\\");
    const std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
    if (name.size() <= 24) return name;
    return name.substr(0, 21) + "...";
}

void metricBlock(const char* label, const std::string& value, bool compact) {
    if (!compact) {
        ImGui::TextColored(UiTheme::TextMuted, "%s", label);
    }
    ImGui::TextColored(UiTheme::Text, "%s", value.c_str());
}

void statusPill(const char* label, const ImVec4& color) {
    const ImVec2 textSize = ImGui::CalcTextSize(label);
    const ImVec2 size(textSize.x + 20.0f, 30.0f);
    const ImVec2 start = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##simulation_status", size);
    ImDrawList* drawList = ImGui::GetWindowDrawList();
    const ImU32 fill = ImGui::ColorConvertFloat4ToU32(
        ImVec4(color.x * 0.22f, color.y * 0.22f, color.z * 0.22f, 1.0f));
    const ImU32 border = ImGui::ColorConvertFloat4ToU32(color);
    drawList->AddRectFilled(start, ImVec2(start.x + size.x, start.y + size.y),
                            fill, 14.0f);
    drawList->AddRect(start, ImVec2(start.x + size.x, start.y + size.y),
                      border, 14.0f, 0, 1.0f);
    drawList->AddCircleFilled(ImVec2(start.x + 10.0f, start.y + size.y * 0.5f),
                              3.0f, border);
    drawList->AddText(ImVec2(start.x + 16.0f,
                             start.y + (size.y - textSize.y) * 0.5f),
                      ImGui::ColorConvertFloat4ToU32(UiTheme::Text), label);
}

int speedIndex(double speed) {
    const double values[] = {0.5, 1.0, 2.0, 4.0};
    int result = 0;
    double distance = std::abs(speed - values[0]);
    for (int i = 1; i < 4; ++i) {
        const double candidate = std::abs(speed - values[i]);
        if (candidate < distance) {
            result = i;
            distance = candidate;
        }
    }
    return result;
}

} // namespace

void DebugConsole::drawTopHud(sf::RenderWindow& window,
                              std::unique_ptr<TrafficSimulator>& simulator,
                              const std::string& mapPathInput,
                              bool usingDemoMap,
                              const StatisticsSummary* statistics) {
    const sf::Vector2u windowSize = window.getSize();
    const float width = static_cast<float>(windowSize.x);
    const bool narrow = width < 650.0f;
    const bool wide = width >= 1040.0f;
    const float hudHeight = narrow ? 60.0f : 66.0f;

    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, hudHeight), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, UiTheme::Background);
    ImGui::PushStyleColor(ImGuiCol_Border, UiTheme::Border);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                        ImVec2(narrow ? 8.0f : 12.0f, narrow ? 10.0f : 9.0f));

    const ImGuiWindowFlags windowFlags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse;

    if (ImGui::Begin("##top_hud", nullptr, windowFlags)) {
        const int columns = wide ? 8 : (narrow ? 6 : 7);
        if (ImGui::BeginTable("##top_hud_layout", columns,
                              ImGuiTableFlags_SizingFixedFit |
                              ImGuiTableFlags_NoSavedSettings)) {
            ImGui::TableSetupColumn("##brand", ImGuiTableColumnFlags_WidthFixed,
                                    narrow ? 46.0f : (wide ? 180.0f : 125.0f));
            ImGui::TableSetupColumn("##status", ImGuiTableColumnFlags_WidthFixed, 82.0f);
            ImGui::TableSetupColumn("##time", ImGuiTableColumnFlags_WidthFixed,
                                    narrow ? 54.0f : 76.0f);
            ImGui::TableSetupColumn("##vehicles", ImGuiTableColumnFlags_WidthFixed,
                                    narrow ? 52.0f : 68.0f);
            if (!narrow) {
                ImGui::TableSetupColumn("##trips", ImGuiTableColumnFlags_WidthFixed, 62.0f);
            }
            if (wide) {
                ImGui::TableSetupColumn("##fps", ImGuiTableColumnFlags_WidthFixed, 52.0f);
            }
            ImGui::TableSetupColumn("##pause", ImGuiTableColumnFlags_WidthFixed,
                                    narrow ? 76.0f : 88.0f);
            ImGui::TableSetupColumn("##speed", ImGuiTableColumnFlags_WidthStretch);

            ImGui::TableNextColumn();
            ImGui::TextColored(UiTheme::AccentStrong, "%s", narrow ? "UTS" : "URBAN TRAFFIC");
            if (!narrow) {
                ImGui::TextColored(UiTheme::TextMuted, "%s",
                                   usingDemoMap ? "DEMO GRID" : shortMapName(mapPathInput).c_str());
                UiTheme::tooltip(mapPathInput.empty() ? "No map file selected" : mapPathInput.c_str());
            }

            ImGui::TableNextColumn();
            if (mapLoadPending_) {
                statusPill("LOADING", UiTheme::AccentStrong);
            } else if (lastLoadFailed_) {
                statusPill("ERROR", UiTheme::Error);
            } else if (!simulator) {
                statusPill(
                    simulationVehicleCountLocked_
                        ? "READY"
                        : "SETUP",
                    simulationVehicleCountLocked_
                        ? UiTheme::AccentStrong
                        : UiTheme::Warning);
            } else if (simulator->isPaused()) {
                statusPill("PAUSED", UiTheme::Warning);
            } else {
                statusPill("RUNNING", UiTheme::Success);
            }

            ImGui::TableNextColumn();
            const double simTime = simulator ? simulator->getElapsedTime()
                                             : (statistics ? statistics->totalSimulatedTime : 0.0);
            metricBlock("SIM TIME", formatTime(simTime, narrow), narrow);

            ImGui::TableNextColumn();
            metricBlock(narrow ? "ACTIVE" : "ACTIVE VEHICLES",
                        std::to_string(
                            simulator
                                ? simulator->getVehicles().size()
                                : 0u),
                        narrow);

            if (!narrow) {
                ImGui::TableNextColumn();
                metricBlock("TRIPS",
                            std::to_string(statistics ? statistics->totalCompletedTrips : 0),
                            false);
            }

            if (wide) {
                ImGui::TableNextColumn();
                char fpsBuffer[16];
                std::snprintf(fpsBuffer, sizeof(fpsBuffer), "%.0f", smoothedFps_);
                metricBlock("FPS", fpsBuffer, false);
            }

            ImGui::TableNextColumn();
            ImGui::BeginDisabled(!simulator);
            const bool paused = simulator && simulator->isPaused();
            if (UiTheme::actionButton(paused ? "Resume" : "Pause",
                                      ImVec2(narrow ? 72.0f : 84.0f, 34.0f)) && simulator) {
                if (paused) simulator->resume();
                else simulator->pause();
            }
            ImGui::EndDisabled();
            UiTheme::tooltip(paused ? "Resume the simulation" : "Pause the simulation");

            ImGui::TableNextColumn();
            ImGui::BeginDisabled(!simulator);
            const double currentSpeed = simulator ? simulator->getSpeedMultiplier() : 1.0;
            if (wide) {
                const char* labels[] = {"0.5x", "1x", "2x", "4x"};
                const double values[] = {0.5, 1.0, 2.0, 4.0};
                for (int i = 0; i < 4; ++i) {
                    if (i > 0) ImGui::SameLine(0.0f, 4.0f);
                    if (UiTheme::selectionButton(labels[i],
                                                 std::abs(currentSpeed - values[i]) < 0.01,
                                                 ImVec2(42.0f, 34.0f)) && simulator) {
                        simulator->setSpeedMultiplier(values[i]);
                    }
                }
            } else {
                const char* labels[] = {"0.5x", "1x", "2x", "4x"};
                int selected = speedIndex(currentSpeed);
                ImGui::SetNextItemWidth(narrow ? 62.0f : 72.0f);
                if (ImGui::Combo("##hud_speed", &selected, labels, 4) && simulator) {
                    const double values[] = {0.5, 1.0, 2.0, 4.0};
                    simulator->setSpeedMultiplier(values[selected]);
                }
                UiTheme::tooltip("Simulation speed multiplier");
            }
            ImGui::EndDisabled();
            ImGui::EndTable();
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void DebugConsole::drawBottomDock(sf::RenderWindow& window,
                                  std::unique_ptr<TrafficSimulator>& simulator,
                                  bool& heatMapEnabled,
                                  bool& showParkedVehicles,
                                  std::string& mapPathInput,
                                  const std::string& loadError) {
    const sf::Vector2u windowSize = window.getSize();
    const float width = static_cast<float>(windowSize.x);
    const float height = static_cast<float>(windowSize.y);
    const bool narrow = width < 650.0f;
    const bool wide = width >= 1040.0f;
    const float dockHeight = 64.0f;

    ImGui::SetNextWindowPos(ImVec2(0.0f, height - dockHeight), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(width, dockHeight), ImGuiCond_Always);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, UiTheme::Background);
    ImGui::PushStyleColor(ImGuiCol_Border, UiTheme::Border);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 11.0f));

    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse;

    if (ImGui::Begin("##bottom_control_dock", nullptr, flags)) {
        const int itemCount = narrow ? 4 : (wide ? 8 : 7);
        const float gap = 6.0f;
        const float itemWidth = std::max(58.0f,
            (ImGui::GetContentRegionAvail().x - gap * static_cast<float>(itemCount - 1))
            / static_cast<float>(itemCount));
        const ImVec2 buttonSize(itemWidth, 42.0f);

        auto next = [&]() { ImGui::SameLine(0.0f, gap); };

        if (wide) {
            if (ImGui::Button("Reset", buttonSize)) {
                if (mapPathInput.empty()) {
                    performMapLoad(true, simulator, mapPathInput, loadError);
                } else {
                    performMapLoad(false, simulator, mapPathInput, loadError);
                }
            }
            UiTheme::tooltip("Reload the current map and restart the simulation");
            next();
        }

        ImGui::BeginDisabled(!simulator);
        const bool paused = simulator && simulator->isPaused();
        if (UiTheme::actionButton(paused ? "Resume" : "Pause", buttonSize) && simulator) {
            if (paused) simulator->resume();
            else simulator->pause();
        }
        ImGui::EndDisabled();
        UiTheme::tooltip(paused ? "Resume simulation" : "Pause simulation");
        next();

        // --- Playback controls (snapshot rewind/forward) ---
        ImGui::BeginDisabled(!simulator);
        {
            auto* controller = simulator
                ? simulator->getPlaybackController()
                : nullptr;
            const std::size_t playbackIndex = controller
                ? controller->currentIndex()
                : SnapshotManager::npos;
            const bool canRewind = controller &&
                controller->available() > 1u &&
                (playbackIndex == SnapshotManager::npos ||
                 playbackIndex > 0u);
            ImGui::BeginDisabled(!canRewind);
            const std::string rewindLabel = rewindButtonLabel(controller);
            if (ImGui::Button(rewindLabel.c_str(), buttonSize) && controller) {
                controller->rewind(1);
            }
            ImGui::EndDisabled();
            UiTheme::tooltip(controller && controller->available() > 1u
                                 ? "Rewind one snapshot"
                                 : "No snapshot history available");
            next();
            const bool canForward = controller &&
                controller->currentIndex() != SnapshotManager::npos &&
                controller->currentIndex() <
                    (controller->available() > 0
                         ? controller->available() - 1
                         : 0);
            ImGui::BeginDisabled(!canForward);
            const std::string forwardLabel = forwardButtonLabel(controller);
            if (ImGui::Button(forwardLabel.c_str(), buttonSize) && controller) {
                controller->forward(1);
            }
            ImGui::EndDisabled();
            UiTheme::tooltip(controller && controller->available() > 1u
                                 ? "Forward one snapshot"
                                 : "No snapshot history available");
        }
        ImGui::EndDisabled();
        next();

        if (ImGui::Button(narrow ? "View" : "Reset View", buttonSize)) {
            resetView_();
            setNotice(NoticeTone::INFO, "Camera reset to the map bounds.");
        }
        UiTheme::tooltip("Reset camera position and zoom");
        next();

        if (!narrow) {
            if (UiTheme::toggleButton("dock_heatmap", "Heatmap", heatMapEnabled, buttonSize)) {
                heatMapEnabled = !heatMapEnabled;
                visualization_.setHeatMapEnabled(heatMapEnabled);
            }
            UiTheme::tooltip("Toggle traffic-density heatmap");
            next();

            const auto currentMode = visualization_.getLodMode();
            const auto currentLevel = visualization_.getLodLevel();
            std::string lodLabel = "LOD: Auto";
            if (currentMode == VisualizationEngine::LodMode::Full) lodLabel = "LOD: Full";
            else if (currentMode == VisualizationEngine::LodMode::Medium) lodLabel = "LOD: Med";
            else if (currentMode == VisualizationEngine::LodMode::Low) lodLabel = "LOD: Low";
            else {
                if (currentLevel == VisualizationEngine::LodLevel::Medium) lodLabel = "LOD: Auto(M)";
                else if (currentLevel == VisualizationEngine::LodLevel::Low) lodLabel = "LOD: Auto(L)";
                else lodLabel = "LOD: Auto(F)";
            }

            if (ImGui::Button(lodLabel.c_str(), buttonSize)) {
                if (currentMode == VisualizationEngine::LodMode::Auto) visualization_.setLodMode(VisualizationEngine::LodMode::Full);
                else if (currentMode == VisualizationEngine::LodMode::Full) visualization_.setLodMode(VisualizationEngine::LodMode::Medium);
                else if (currentMode == VisualizationEngine::LodMode::Medium) visualization_.setLodMode(VisualizationEngine::LodMode::Low);
                else visualization_.setLodMode(VisualizationEngine::LodMode::Auto);
            }
            UiTheme::tooltip("Level of Detail: Auto -> Full -> Med -> Low");
            next();
        } else {
            // narrow mode: no Map button
        }



        if (narrow) {
            if (UiTheme::selectionButton("Debug",
                                         drawerOpen_ && activeTab_ == DrawerTab::DEBUG,
                                         buttonSize)) {
                openDrawer(DrawerTab::DEBUG);
            }
            UiTheme::tooltip("Open advanced debug controls");
        } else {
            if (UiTheme::selectionButton("Control Center", drawerOpen_, buttonSize)) {
                if (drawerOpen_ && activeTab_ == DrawerTab::OVERVIEW) {
                    drawerOpen_ = false;
                } else {
                    openDrawer(DrawerTab::OVERVIEW);
                }
            }
            UiTheme::tooltip("Open overview, performance and advanced controls");
        }
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor(2);
}

void DebugConsole::performMapLoad(bool useDemo,
                                  std::unique_ptr<TrafficSimulator>& simulator,
                                  std::string& mapPathInput,
                                  const std::string& loadError) {
    (void)loadError;
    if (useDemo) {
        mapPathInput.clear();
    }

    pendingDemoLoad_ = useDemo;
    pendingMapPath_ = useDemo ? "" : mapPathInput;
    mapLoadPending_ = true;
    simulator.reset();
    mapPathBufferInitialized_ = false;
    setNotice(NoticeTone::INFO, useDemo ? "Loading demo map..." : "Loading map...");
}

void DebugConsole::completePendingMapLoad(
        std::unique_ptr<TrafficSimulator>& simulator,
        std::string& mapPathInput,
        const std::string& loadError) {
    const bool useDemo = pendingDemoLoad_;
    const std::string requestedPath = pendingMapPath_;
    mapLoadPending_ = false;
    pendingDemoLoad_ = false;
    pendingMapPath_.clear();

    loadAndRefresh_(useDemo ? "" : requestedPath);
    onMapChanged();

    if (useDemo) {
        mapPathInput.clear();
        lastLoadFailed_ = false;
        setNotice(
            NoticeTone::SUCCESS,
            "Built-in demo map loaded. Adjust the vehicle count and start the simulation.");
    } else if (loadError.empty()) {
        lastLoadFailed_ = false;
        setNotice(
            NoticeTone::SUCCESS,
            "Map loaded. Adjust the vehicle count and start the simulation.");
    } else {
        lastLoadFailed_ = true;
        setNotice(NoticeTone::ERROR,
                  "The map could not be loaded. The demo map was restored.");
    }
}

void DebugConsole::drawMapTab(std::unique_ptr<TrafficSimulator>& simulator,
                              std::string& mapPathInput,
                              bool usingDemoMap,
                              const std::string& loadError) {
    if (!mapPathBufferInitialized_) {
        std::fill(mapPathBuffer_.begin(), mapPathBuffer_.end(), '\0');
        const std::size_t count = std::min(mapPathInput.size(), mapPathBuffer_.size() - 1);
        std::copy_n(mapPathInput.begin(), count, mapPathBuffer_.begin());
        mapPathBufferInitialized_ = true;
    }

    ImGui::TextColored(UiTheme::TextMuted, "MAP SOURCE");
    ImGui::Text("%s", usingDemoMap ? "Built-in demo map" : "JSON map file");
    ImGui::Text("Intersections  %zu", intersectionsSnapshot_.size());
    ImGui::SameLine(200.0f);
    ImGui::Text("Roads  %zu", roadsSnapshot_.size());
    ImGui::Spacing();

    ImGui::TextColored(UiTheme::TextMuted, "FILE PATH");
    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::InputTextWithHint("##map_path", "Path to a .json map",
                                 mapPathBuffer_.data(), mapPathBuffer_.size())) {
        mapPathInput = mapPathBuffer_.data();
    }

    if (ImGui::Button("Browse...", ImVec2(100.0f, 36.0f))) {
        std::string selectedPath;
        if (openFileDialog_ && openFileDialog_(selectedPath)) {
            mapPathInput = selectedPath;
            mapPathBufferInitialized_ = false;
        } else {
            setNotice(NoticeTone::INFO,
                      "No native file picker is available. Enter the map path directly.");
        }
    }
    UiTheme::tooltip("Choose a JSON map file when a native picker is available");
    ImGui::SameLine();

    const bool canLoad = !mapPathInput.empty();
    ImGui::BeginDisabled(!canLoad);
    if (UiTheme::actionButton("Load Map", ImVec2(110.0f, 36.0f))) {
        performMapLoad(false, simulator, mapPathInput, loadError);
    }
    ImGui::EndDisabled();
    if (!canLoad) {
        UiTheme::tooltip("Enter a map path before loading");
    } else {
        UiTheme::tooltip("Replace the current map and restart the simulation");
    }
    ImGui::SameLine();
    if (ImGui::Button("Demo Map", ImVec2(110.0f, 36.0f))) {
        performMapLoad(true, simulator, mapPathInput, loadError);
    }
    UiTheme::tooltip("Load the built-in demo map");

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    if (lastLoadFailed_ && !loadError.empty()) {
        ImGui::TextColored(UiTheme::Error, "MAP LOAD FAILED");
        ImGui::TextWrapped("%s", loadError.c_str());
        ImGui::TextColored(UiTheme::TextMuted,
                           "The demo map is active. Correct the path and try again.");
    } else {
        ImGui::TextColored(UiTheme::Success, "MAP READY");
        ImGui::TextWrapped("%s", usingDemoMap
            ? "The built-in demo map is active."
            : "The selected map is loaded and ready.");
    }
}
