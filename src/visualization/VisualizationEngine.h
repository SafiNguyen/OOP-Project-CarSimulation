#ifndef VISUALIZATIONENGINE_H
#define VISUALIZATIONENGINE_H

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include "TrafficLight.h"

class Graph;
class Road;
class Intersection;
class Vehicle;

class ViewportBounds {
public:
    explicit ViewportBounds(
        const sf::View& view,
        float margin = 0.0f) noexcept {
        const sf::Vector2f center = view.getCenter();
        const sf::Vector2f size = view.getSize();
        const float halfWidth = std::abs(size.x) * 0.5f;
        const float halfHeight = std::abs(size.y) * 0.5f;
        const float radians =
            view.getRotation() * 3.14159265358979323846f / 180.0f;
        const float cosine = std::abs(std::cos(radians));
        const float sine = std::abs(std::sin(radians));
        const float enclosingHalfWidth =
            cosine * halfWidth + sine * halfHeight;
        const float enclosingHalfHeight =
            sine * halfWidth + cosine * halfHeight;
        const float safeMargin = std::max(0.0f, margin);

        minimumX_ = center.x - enclosingHalfWidth - safeMargin;
        maximumX_ = center.x + enclosingHalfWidth + safeMargin;
        minimumY_ = center.y - enclosingHalfHeight - safeMargin;
        maximumY_ = center.y + enclosingHalfHeight + safeMargin;
    }

    bool containsPoint(
        const sf::Vector2f& point,
        float radius = 0.0f) const noexcept {
        const float safeRadius = std::max(0.0f, radius);
        return point.x + safeRadius >= minimumX_ &&
               point.x - safeRadius <= maximumX_ &&
               point.y + safeRadius >= minimumY_ &&
               point.y - safeRadius <= maximumY_;
    }

    bool intersectsRectangle(
        const sf::FloatRect& rectangle,
        float margin = 0.0f) const noexcept {
        const float safeMargin = std::max(0.0f, margin);
        return rectangle.left + rectangle.width + safeMargin >=
                   minimumX_ &&
               rectangle.left - safeMargin <= maximumX_ &&
               rectangle.top + rectangle.height + safeMargin >=
                   minimumY_ &&
               rectangle.top - safeMargin <= maximumY_;
    }

    bool intersectsSegment(
        const sf::Vector2f& start,
        const sf::Vector2f& end,
        float halfThickness = 0.0f) const noexcept {
        const float thickness =
            std::max(0.0f, halfThickness);
        const float left = std::min(start.x, end.x) - thickness;
        const float top = std::min(start.y, end.y) - thickness;
        const float right = std::max(start.x, end.x) + thickness;
        const float bottom = std::max(start.y, end.y) + thickness;
        return intersectsRectangle(
            sf::FloatRect(left, top, right - left, bottom - top));
    }

private:
    float minimumX_ = 0.0f;
    float maximumX_ = 0.0f;
    float minimumY_ = 0.0f;
    float maximumY_ = 0.0f;
};

class VisualizationEngine {
public:
    // Level of detail for the per-frame dynamic layer. When the frame rate
    // drops (e.g. on very large maps), the renderer can shed the most
    // expensive per-frame work — lane markings, road-name labels, traffic
    // light countdowns — while keeping the cached static layer intact.
    enum class LodLevel {
        Full,   // All dynamic detail (lane markings, road names, lights, heat tint)
        Medium, // Skip lane markings, road names, intersection heat tint, light countdowns
        Low     // Also skip traffic lights, congestion overlay, blocked-lane fills
    };

    enum class LodMode {
        Auto,   // FPS-based adaptive LOD
        Full,   // Forced Full LOD
        Medium, // Forced Medium LOD
        Low     // Forced Low LOD
    };

    VisualizationEngine(sf::Vector2u windowSize = {800u, 600u}, float margin = 48.0f);

    void prepare(const Graph& graph);
    void setWindowSize(sf::Vector2u windowSize);
    sf::Vector2f worldToScreen(double x, double y) const;
    float metresToScreenPixels(double metres,
                               const Road* referenceRoad) const;
    const std::vector<sf::Vector2f>& getRoutePoints() const;
    sf::Color colorForRoad(const Road* road) const;

    void drawGraph(sf::RenderTarget& target, const Graph& graph) const;
    void drawStaticLayer(sf::RenderTarget& target, const Graph& graph) const;
    void drawDynamicLayer(sf::RenderTarget& target, const Graph& graph) const;

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
    void setLodLevel(LodLevel level);
    LodLevel getLodLevel() const;
    void setLodMode(LodMode mode);
    LodMode getLodMode() const;
    double getScale() const { return scale_; }
    std::uint64_t getRevision() const;

    // Zoom-aware detail factor. Returns 1.0 at the default zoom level,
    // >1.0 when zoomed in, and <1.0 when zoomed out. Overlay elements
    // (POIs, bus stops, road names, traffic lights) use this to scale
    // their on-screen size with the road/intersection they belong to and
    // to hide entirely once the view is zoomed out past a threshold.
    float getDetailScale(const sf::View& view) const;

private:
    struct RoadDraw {
        Road* road;
        sf::Vector2f offsetA;
        sf::Vector2f offsetB;
        float laneWidth;
        float totalWidth;
        int laneCount;
        bool isBridge;
        bool isTunnel;
        sf::Color bodyColor;
        bool hasBorder;
        sf::Color borderColor;
        float borderWidth;
    };

    std::vector<RoadDraw> buildRoadDrawList(const Graph& graph) const;
    void drawLaneMarkings(sf::RenderTarget& target,
                          const std::vector<RoadDraw>& drawList) const;

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
    void drawBusStations(sf::RenderTarget& target, const Graph& graph) const;
    void drawSidewalks(sf::RenderTarget& target, const Graph& graph) const;
    void drawIntersectionNode(sf::RenderTarget& target, const Intersection* intersection,
                              bool tintByCongestion) const;
    void drawRoadCongestionOverlay(sf::RenderTarget& target, const Graph& graph) const;
    void drawBlockedLaneFills(sf::RenderTarget& target, const Graph& graph) const;

    sf::Color lightColor(LightState state) const;
    sf::Vector2f roadNormal(const sf::Vector2f& a, const sf::Vector2f& b) const;

    void drawPOIDriveways(sf::RenderTarget& target,
                          const Graph& graph) const;
    void drawPOIs(sf::RenderTarget& target, const Graph& graph) const;
    void drawRoadNames(sf::RenderTarget& target, const std::vector<Road*>& roads) const;
    float getIntersectionBoxHalfExtent(const Intersection* intersection) const;
    float getLaneWidthPixels(const Road* road) const;
    sf::Color getIntersectionBoxColor(const Intersection* intersection, bool tintByCongestion) const;

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
    LodLevel lodLevel_ = LodLevel::Low;
    LodMode lodMode_ = LodMode::Auto;
    std::uint64_t revision_ = 0;
};

#endif
