#ifndef STATSPANEL_H
#define STATSPANEL_H

#include <SFML/Graphics.hpp>
#include "../simulation/StatisticsManager.h"

class StatsPanel {
public:
    StatsPanel();
    
    void update(const StatisticsSummary& summary);
    void draw(sf::RenderTarget& target) const;

private:
    sf::Font m_font;
    bool m_fontLoaded;
    
    sf::RectangleShape background;
    sf::Text statsText;
};

#endif
