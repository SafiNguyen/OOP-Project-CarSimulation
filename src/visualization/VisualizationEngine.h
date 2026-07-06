#ifndef VISUALIZATIONENGINE_H
#define VISUALIZATIONENGINE_H

#include <SFML/Graphics.hpp>
#include <vector>

class Graph;
class Road;

class VisualizationEngine {
public:
    VisualizationEngine(sf::Vector2u windowSize = {800u, 600u}, float margin = 24.0f);

    void prepare(const Graph& graph);
    sf::Vector2f worldToScreen(double x, double y) const;
    const std::vector<sf::Vector2f>& getRoutePoints() const;
    sf::Color colorForRoad(const Road* road) const;

    void drawGraph(sf::RenderTarget& target, const Graph& graph) const;

    void setSpriteTexture(const sf::Texture& texture,
                          const sf::IntRect& rect = sf::IntRect(),
                          const sf::Vector2f& size = {24.0f, 24.0f});
    void clearSpriteTexture();

    void setHeatMapEnabled(bool enabled);
    bool isHeatMapEnabled() const;

private:
    static sf::Color mixColor(const sf::Color& a, const sf::Color& b, float t);
    static float distanceBetween(const sf::Vector2f& a, const sf::Vector2f& b);

    void drawRoadStrip(sf::RenderTarget& target,
                       const sf::Vector2f& a,
                       const sf::Vector2f& b,
                       const sf::Color& color,
                       float thickness) const;

    sf::Vector2u windowSize_;
    float margin_;
    double minX_;
    double minY_;
    double maxX_;
    double maxY_;
    double scale_;
    std::vector<sf::Vector2f> routePoints_;
    const sf::Texture* spriteTexture_;
    sf::IntRect spriteRect_;
    sf::Vector2f spriteSize_;
    bool heatMapEnabled_;
};

#endif
