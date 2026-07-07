#include "StatsPanel.h"
#include <sstream>
#include <iomanip>

StatsPanel::StatsPanel() : m_fontLoaded(false) {
    std::vector<std::string> fontPaths = {
        "C:/Windows/Fonts/arial.ttf",           // Windows
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",  // Linux
        "/Library/Fonts/Arial.ttf"               // Mac
    };
    
    for (const auto& path : fontPaths) {
        if (m_font.loadFromFile(path)) {
            m_fontLoaded = true;
            break;
        }
    }
    
    background.setFillColor(sf::Color(20, 20, 20, 200));
    background.setOutlineColor(sf::Color(100, 100, 100));
    background.setOutlineThickness(1.5f);
    background.setPosition(10.0f, 10.0f);
    
    statsText.setFont(m_font);
    statsText.setCharacterSize(14);
    statsText.setFillColor(sf::Color::White);
    statsText.setPosition(20.0f, 20.0f);
}

void StatsPanel::update(const StatisticsSummary& summary) {
    if (!m_fontLoaded) return;
    
    std::stringstream ss;
    ss << "=== Traffic Simulation Stats ===\n"
       << "Simulated Time: " << std::fixed << std::setprecision(1) << summary.totalSimulatedTime << " s\n"
       << "Vehicles Tracked: " << summary.totalVehiclesTracked << "\n"
       << "Completed Trips: " << summary.totalCompletedTrips << "\n"
       << "Recalculations: " << summary.totalRecalculations << "\n";
       
    for (const auto& metric : summary.perAlgorithm) {
        ss << "\nAlg: " << metric.algorithmName << "\n"
           << "  - Calls: " << metric.callCount << "\n"
           << "  - Found: " << metric.pathsFound << " / Not Found: " << metric.pathsNotFound << "\n"
           << "  - Avg Time: " << std::setprecision(2) << metric.averageComputeTimeMs() << " ms\n"
           << "  - Avg Nodes: " << std::fixed << std::setprecision(0) << metric.averageNodesExplored() << "\n"
           << "  - Avg Cost: " << std::fixed << std::setprecision(2) << metric.averagePathCost() << "\n";
    }
    
    statsText.setString(ss.str());
    
    sf::FloatRect bounds = statsText.getLocalBounds();
    background.setSize(sf::Vector2f(bounds.width + 20.0f, bounds.height + 20.0f));
}

void StatsPanel::draw(sf::RenderTarget& target) const {
    if (m_fontLoaded) {
        target.draw(background);
        target.draw(statsText);
    }
}
