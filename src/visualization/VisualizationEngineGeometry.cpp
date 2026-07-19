#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

sf::Vector2f VisualizationEngine::worldToScreen(double x, double y) const {
    const float sx = static_cast<float>(margin_ + (x - minX_) * scale_);
    const float sy = static_cast<float>(windowSize_.y - margin_ - (y - minY_) * scale_);
    return {sx, sy};
}

sf::Color VisualizationEngine::mixColor(const sf::Color& a, const sf::Color& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto mix = [t](sf::Uint8 x, sf::Uint8 y) -> sf::Uint8 {
        return static_cast<sf::Uint8>(x + (y - x) * t);
    };

    return sf::Color(mix(a.r, b.r), mix(a.g, b.g), mix(a.b, b.b), mix(a.a, b.a));
}

float VisualizationEngine::distanceBetween(const sf::Vector2f& a, const sf::Vector2f& b) {
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

void VisualizationEngine::drawRoadStrip(sf::RenderTarget& target,
                                        const sf::Vector2f& a,
                                        const sf::Vector2f& b,
                                        const sf::Color& color,
                                        float thickness) const {
    const float len = distanceBetween(a, b);
    if (len <= 0.01f) {
        return;
    }

    sf::RectangleShape strip({len, thickness});
    strip.setOrigin(0.0f, thickness * 0.5f);
    strip.setPosition(a);
    strip.setRotation(std::atan2(b.y - a.y, b.x - a.x) * 180.0f / 3.14159265f);
    strip.setFillColor(color);
    target.draw(strip);
}

sf::Vector2f VisualizationEngine::roadNormal(const sf::Vector2f& a, const sf::Vector2f& b) const {
    sf::Vector2f dir = b - a;
    const float length = std::sqrt(dir.x * dir.x + dir.y * dir.y);
    if (length <= 0.01f) {
        return {0.0f, 0.0f};
    }
    return {-dir.y / length, dir.x / length};
}