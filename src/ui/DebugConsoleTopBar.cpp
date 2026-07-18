#include "DebugConsole.h"

#include <algorithm>
#include <array>

#include <imgui.h>

#include "simulation/TrafficSimulator.h"
#include "visualization/VisualizationEngine.h"
#include <fstream>

void DebugConsole::drawTopBar(sf::RenderWindow& window,
                               std::unique_ptr<TrafficSimulator>& simulator,
                               sf::View& view,
                               float& zoomFactor,
                               bool& heatMapEnabled,
                               std::string& mapPathInput,
                               bool usingDemoMap,
                               const std::string& loadError) {
    static std::array<char, 1024> mapPathBuffer{};
    std::copy_n(mapPathInput.begin(), std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1), mapPathBuffer.begin());
    mapPathBuffer[std::min<std::size_t>(mapPathInput.size(), mapPathBuffer.size() - 1)] = '\0';
    ImGui::InputText("Map path", mapPathBuffer.data(), mapPathBuffer.size());
    mapPathInput = mapPathBuffer.data();

    if (ImGui::Button("Load map")) {
        std::string chosenPath;
        if (openFileDialog_ && openFileDialog_(chosenPath)) {
            mapPathInput = chosenPath;
        }
        simulator.reset();
        loadAndRefresh_(mapPathInput);
        simulator = resetSimulation_();
        onMapChanged();
    }
    ImGui::SameLine();
    if (ImGui::Button("Demo map")) {
        simulator.reset();
        loadAndRefresh_("");
        simulator = resetSimulation_();
        onMapChanged();
    }
    ImGui::SameLine();
    if (ImGui::Button(simulator && simulator->isPaused() ? "Resume" : "Pause")) {
        if (simulator) {
            if (simulator->isPaused()) simulator->resume();
            else simulator->pause();
        }
    }

    if (ImGui::Button("Reset view")) {
        zoomFactor = 1.0f;
        view = window.getDefaultView();
        clampViewToMap_();
    }
    ImGui::SameLine();
    if (ImGui::Button(heatMapEnabled ? "Heat: ON" : "Heat: OFF")) {
        heatMapEnabled = !heatMapEnabled;
        visualization_.setHeatMapEnabled(heatMapEnabled);
    }

    ImGui::TextWrapped("Status: %s", usingDemoMap ? "Demo map is active." : (mapPathInput.empty() ? "No map selected." : mapPathInput.c_str()));
    ImGui::Text("Intersections: %zu | Roads: %zu | Vehicles: %zu",
                intersectionsSnapshot_.size(), roadsSnapshot_.size(),
                simulator ? simulator->getVehicles().size() : 0u);

    if (!loadError.empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "Warning: %s", loadError.c_str());
    }
    if (intersectionsSnapshot_.empty() || roadsSnapshot_.empty()) {
        ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "The current scene is missing intersections or roads.");
    }
    if (simulator && simulator->getVehicles().empty()) {
        ImGui::TextColored(ImVec4(0.90f, 0.90f, 0.65f, 1.0f), "No vehicles are currently active.");
    }
}
