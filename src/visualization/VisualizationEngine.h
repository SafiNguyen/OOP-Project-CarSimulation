#ifndef VISUALIZATIONENGINE_H
#define VISUALIZATIONENGINE_H

#include <SFML/Graphics.hpp>
#include <cstdint>
#include <vector>
#include "TrafficLight.h"

class Graph;
class Road;
class Intersection;

class VisualizationEngine {
public:
    VisualizationEngine(sf::Vector2u windowSize = {800u, 600u}, float margin = 48.0f);

    void prepare(const Graph& graph);
    void setWindowSize(sf::Vector2u windowSize);
    sf::Vector2f worldToScreen(double x, double y) const;
    float metresToScreenPixels(double metres,
                               const Road* referenceRoad) const;
    const std::vector<sf::Vector2f>& getRoutePoints() const;
    sf::Color colorForRoad(const Road* road) const;

    void drawGraph(sf::RenderTarget& target, const Graph& graph) const;

    void setSpriteTexture(const sf::Texture& texture,
                          const sf::IntRect& rect = sf::IntRect(),
                          const sf::Vector2f& size = {24.0f, 24.0f});

    sf::Vector2f getRoadEntryPoint(const Road* road, const Intersection* intersection) const;
    sf::Vector2f getRoadCenterlineEntryPoint(const Road* road, const Intersection* intersection) const;
    void clearSpriteTexture();

    void setFont(const sf::Font& font);
    void clearFont();

    void setHeatMapEnabled(bool enabled);
    bool isHeatMapEnabled() const;
    std::uint64_t getRevision() const;

private:
    static sf::Color mixColor(const sf::Color& a, const sf::Color& b, float t);
    static float distanceBetween(const sf::Vector2f& a, const sf::Vector2f& b);

    void drawRoadStrip(sf::RenderTarget& target,
                       const sf::Vector2f& a,
                       const sf::Vector2f& b,
                       const sf::Color& color,
                       float thickness) const;

    void rasterizeBodyToMask(const sf::Vector2f& a,
                             const sf::Vector2f& b,
                             float thickness,
                             std::vector<uint8_t>& bodyMask,
                             unsigned int gridW,
                             unsigned int gridH,
                             unsigned int cellSize) const;

    void drawRoadBorderMasked(sf::RenderTarget& target,
                              const sf::Vector2f& a,
                              const sf::Vector2f& b,
                              const sf::Color& color,
                              float thickness,
                              const std::vector<uint8_t>& bodyMask,
                              unsigned int gridW,
                              unsigned int gridH,
                              unsigned int cellSize) const;
    void drawTrafficLights(sf::RenderTarget& target, const Graph& graph) const;
    void drawBusStops(sf::RenderTarget& target, const Graph& graph) const;
    void drawSidewalks(sf::RenderTarget& target, const Graph& graph) const;
    void drawCrosswalks(sf::RenderTarget& target, const Graph& graph) const;
    void drawCrosswalkSignals(sf::RenderTarget& target,
                              const Graph& graph) const;
    void drawIntersectionNode(sf::RenderTarget& target, const Intersection* intersection) const;
    
    sf::Color lightColor(LightState state) const;
    sf::Vector2f roadNormal(const sf::Vector2f& a, const sf::Vector2f& b) const;

    void drawPOIs(sf::RenderTarget& target, const Graph& graph) const;
    void drawRoadNames(sf::RenderTarget& target, const std::vector<Road*>& roads) const;
    float getIntersectionBoxHalfExtent(const Intersection* intersection) const;
    float getLaneWidthPixels(const Road* road) const;
    sf::Color getIntersectionBoxColor(const Intersection* intersection) const;

    sf::Vector2u windowSize_;
    float margin_;
    double minX_;
    double minY_;
    double maxX_;
    double maxY_;
    double scale_;
    double offsetX_;
    double offsetY_;
    std::vector<sf::Vector2f> routePoints_;
    const sf::Texture* spriteTexture_;
    sf::IntRect spriteRect_;
    sf::Vector2f spriteSize_;
    const sf::Font* font_;
    bool heatMapEnabled_;
    std::uint64_t revision_ = 0;
};

#endif
