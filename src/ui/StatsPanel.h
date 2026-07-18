#ifndef STATSPANEL_H
#define STATSPANEL_H

#include <SFML/System/Vector2.hpp>
#include "../simulation/StatisticsManager.h"


class StatsPanel {
public:
    StatsPanel() = default;

    void draw(const StatisticsSummary& summary, sf::Vector2u windowSize);

private:
    bool collapsed_ = false;
};

#endif