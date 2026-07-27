#ifndef STATSPANEL_H
#define STATSPANEL_H
 
#include "../simulation/StatisticsManager.h"
 
class StatsPanel {
public:
    StatsPanel() = default;
 
    // Embedded drawer components. They deliberately do not create their own
    // floating ImGui window; the unified HUD owns positioning and scrolling.
    void drawOverview(const StatisticsSummary& summary);
    void drawPerformance(const StatisticsSummary& summary);
};
 
#endif
