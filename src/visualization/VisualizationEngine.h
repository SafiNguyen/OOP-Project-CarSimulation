#ifndef VISUALIZATIONENGINE_H
#define VISUALIZATIONENGINE_H

#include <SFML/Graphics.hpp>
#include <cstdint>
#include <vector>
#include "model/TrafficLight.h"

class Graph;
class Road;
class Intersection;

class VisualizationEngine {
public:
    VisualizationEngine(sf::Vector2u windowSize = {800u, 600u}, float margin = 24.0f);

    void prepare(const Graph& graph);
    void setWindowSize(sf::Vector2u windowSize);
    sf::Vector2f worldToScreen(double x, double y) const;
    const std::vector<sf::Vector2f>& getRoutePoints() const;
    sf::Color colorForRoad(const Road* road) const;

    void drawGraph(sf::RenderTarget& target, const Graph& graph) const;

    void setSpriteTexture(const sf::Texture& texture,
                          const sf::IntRect& rect = sf::IntRect(),
                          const sf::Vector2f& size = {24.0f, 24.0f});
    void clearSpriteTexture();

    void setFont(const sf::Font& font);
    void clearFont();

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
    // Mark the road's lane-fill rectangle (centreline `a` to `b` with
    // `thickness`) into `bodyMask`, a row-major grid of `gridW * gridH`
    // cells where each cell covers `cellSize * cellSize` screen pixels.
    // Used by drawGraph to keep track of which screen pixels are already
    // covered by some road's body so the border pass can skip them.
    void rasterizeBodyToMask(const sf::Vector2f& a,
                             const sf::Vector2f& b,
                             float thickness,
                             std::vector<uint8_t>& bodyMask,
                             unsigned int gridW,
                             unsigned int gridH,
                             unsigned int cellSize) const;
    // Like drawRoadStrip, but the border is drawn in 1px chunks and any
    // chunk whose centre falls on a cell marked in `bodyMask` is skipped.
    // This makes overlapping roads merge visually at intersections instead
    // of stacking dark curb strips on top of each other.
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
    void drawIntersectionNode(sf::RenderTarget& target, const Intersection* intersection) const;
    sf::Vector2f getRoadEntryPoint(const Road* road, const Intersection* intersection) const;
    sf::Color lightColor(LightState state) const;
    sf::Vector2f roadNormal(const sf::Vector2f& a, const sf::Vector2f& b) const;

    void drawPOIs(sf::RenderTarget& target, const Graph& graph) const;
    void drawRoadNames(sf::RenderTarget& target, const std::vector<Road*>& roads) const;

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
    const sf::Font* font_;
    bool heatMapEnabled_;
};

#endif
