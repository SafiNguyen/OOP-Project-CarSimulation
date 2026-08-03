#include "VisualizationEngine.h"

#include <algorithm>
#include <cmath>

#include "RoadGeometry.h"

sf::Vector2f VisualizationEngine::worldToScreen(double x, double y) const {
    const float sx = static_cast<float>(
        offsetX_ + (x - minX_) * scale_);
    const float sy = static_cast<float>(
        static_cast<double>(windowSize_.y) -
        offsetY_ -
        (y - minY_) * scale_);
    return {sx, sy};
}

float VisualizationEngine::metresToScreenPixels(
    double metres,
    const Road* referenceRoad) const {
    const double naturalPixelsPerMetre =
        referenceRoad != nullptr
            ? scale_ / std::max(
                   1e-6,
                   RoadGeometry::metresPerWorldUnit(*referenceRoad))
            : scale_;
    return static_cast<float>(
        std::fabs(metres) *
        naturalPixelsPerMetre);
}

float VisualizationEngine::getDetailScale(
    const sf::View& view) const {
    // The view size is windowSize * zoomFactor. At the default zoom the
    // view spans the whole window, so the ratio of the window size to the
    // current view size gives 1.0 at default zoom, >1.0 when zoomed in
    // (smaller view), and <1.0 when zoomed out (larger view). Clamp so
    // overlays never grow unboundedly when zoomed in very far.
    const float viewWidth = std::abs(view.getSize().x);
    const float viewHeight = std::abs(view.getSize().y);
    const float windowWidth =
        static_cast<float>(std::max(1u, windowSize_.x));
    const float windowHeight =
        static_cast<float>(std::max(1u, windowSize_.y));
    const float scaleX = windowWidth / std::max(1.0f, viewWidth);
    const float scaleY = windowHeight / std::max(1.0f, viewHeight);
    float scale = std::clamp(std::min(scaleX, scaleY), 0.0f, 50.0f);

    return scale * mapDetailFactor_;
}

float VisualizationEngine::getTextRenderScale(const sf::View& view) const {
    const float viewWidth = std::abs(view.getSize().x);
    const float viewHeight = std::abs(view.getSize().y);
    const float zoomScale = std::min(
        static_cast<float>(std::max(1u, windowSize_.x)) / std::max(1.0f, viewWidth),
        static_cast<float>(std::max(1u, windowSize_.y)) / std::max(1.0f, viewHeight));
    return 1.0f / std::max(1.0f, zoomScale);
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

// ---------------------------------------------------------------------------
// Overlap-aware road border rendering.
//
// When two roads cross, drawing each road's dark curb/border on top of the
// other's lane fill produces a "+" of black at every intersection. To fix
// that, drawGraph first rasterises every road's body (the coloured lane
// fill) to a CPU-side mask grid, then draws each road's border in 1px
// chunks and skips any chunk whose centre is covered by another road's
// body in the mask. This makes crossings render as a single continuous
// asphalt surface instead of a stack of overlapping dark strips.
// ---------------------------------------------------------------------------

namespace {
// Mark the rotated rectangle defined by `a..b` (centreline endpoints) with
// the given thickness into `mask` (a row-major grid of gridW x gridH
// sampled at `cellSize` pixels per cell). Cells outside the screen bounds
// are silently ignored.
void rasterizeCenterlineToMask(const sf::Vector2f& a,
                               const sf::Vector2f& b,
                               float thickness,
                               std::vector<uint8_t>& mask,
                               unsigned int gridW,
                               unsigned int gridH,
                               unsigned int cellSize) {
    if (mask.empty() || gridW == 0 || gridH == 0) {
        return;
    }

    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const float length = std::sqrt(dx * dx + dy * dy);
    if (length <= 0.01f || thickness <= 0.01f) {
        return;
    }

    const float invLen = 1.0f / length;
    const float dirX = dx * invLen;
    const float dirY = dy * invLen;
    const float normX = -dirY;
    const float normY = dirX;
    const float halfLen = length * 0.5f;
    const float halfThick = thickness * 0.5f;
    const float cx = (a.x + b.x) * 0.5f;
    const float cy = (a.y + b.y) * 0.5f;

    // Axis-aligned bounding box of the rotated rectangle.
    const float extentAlong = std::fabs(dirX) * halfLen + std::fabs(normX) * halfThick;
    const float extentAcross = std::fabs(dirY) * halfLen + std::fabs(normY) * halfThick;
    const int x0 = std::max(0, static_cast<int>(std::floor((cx - extentAlong) / cellSize)));
    const int y0 = std::max(0, static_cast<int>(std::floor((cy - extentAcross) / cellSize)));
    const int x1 = std::min(static_cast<int>(gridW) - 1,
                            static_cast<int>(std::floor((cx + extentAlong) / cellSize)));
    const int y1 = std::min(static_cast<int>(gridH) - 1,
                            static_cast<int>(std::floor((cy + extentAcross) / cellSize)));
    if (x1 < x0 || y1 < y0) {
        return;
    }

    for (int iy = y0; iy <= y1; ++iy) {
        const float py = static_cast<float>(iy * cellSize) + cellSize * 0.5f - cy;
        const std::size_t row = static_cast<std::size_t>(iy) * gridW;
        for (int ix = x0; ix <= x1; ++ix) {
            const float px = static_cast<float>(ix * cellSize) + cellSize * 0.5f - cx;
            // Express (px, py) in the rectangle's local frame (dir, norm).
            const float localAlong = px * dirX + py * dirY;
            const float localAcross = px * normX + py * normY;
            if (std::fabs(localAlong) <= halfLen && std::fabs(localAcross) <= halfThick) {
                mask[row + static_cast<std::size_t>(ix)] = 1;
            }
        }
    }
}

// Test a single screen-space point against the mask grid. Returns true when
// the point's cell is marked as belonging to some road's body.
bool isPointOnBody(float screenX,
                   float screenY,
                   const std::vector<uint8_t>& mask,
                   unsigned int gridW,
                   unsigned int gridH,
                   unsigned int cellSize) {
    if (mask.empty() || gridW == 0 || gridH == 0) {
        return false;
    }
    const int ix = static_cast<int>(screenX / cellSize);
    const int iy = static_cast<int>(screenY / cellSize);
    if (ix < 0 || iy < 0 || ix >= static_cast<int>(gridW) || iy >= static_cast<int>(gridH)) {
        return false;
    }
    return mask[static_cast<std::size_t>(iy) * gridW + static_cast<std::size_t>(ix)] != 0;
}
} // namespace

void VisualizationEngine::rasterizeBodyToMask(const sf::Vector2f& a,
                                              const sf::Vector2f& b,
                                              float thickness,
                                              std::vector<uint8_t>& bodyMask,
                                              unsigned int gridW,
                                              unsigned int gridH,
                                              unsigned int cellSize) const {

    const float maxSpan = std::max(gridW, gridH) * static_cast<float>(cellSize);
    if (distanceBetween(a, b) > maxSpan * 4.0f || thickness > maxSpan) {
        return;
    }
    rasterizeCenterlineToMask(a, b, thickness, bodyMask, gridW, gridH, cellSize);
}

void VisualizationEngine::drawRoadBorderMasked(sf::RenderTarget& target,
                                               const sf::Vector2f& a,
                                               const sf::Vector2f& b,
                                               const sf::Color& color,
                                               float thickness,
                                               const std::vector<uint8_t>& bodyMask,
                                               unsigned int gridW,
                                               unsigned int gridH,
                                               unsigned int cellSize) const {
    const float len = distanceBetween(a, b);
    if (len <= 0.01f || thickness <= 0.0f) {
        return;
    }
    if (bodyMask.empty() || gridW == 0 || gridH == 0) {
        // No mask available: fall back to the un-masked border (old behaviour).
        drawRoadStrip(target, a, b, color, thickness);
        return;
    }

    // Walk along the centreline in 1px steps. Each step we test the mask at
    // the sample point and group consecutive visible steps into runs, then
    // emit one rotated RectangleShape per run. This keeps draw-call count
    // proportional to the number of border segments, not to the number of
    // pixels.
    const float angleDeg = std::atan2(b.y - a.y, b.x - a.x) * 180.0f / 3.14159265f;
    const float invLen = 1.0f / len;
    const float step = 1.0f;

    float runStart = -1.0f;
    auto emitRun = [&](float start, float end) {
        if (end <= start) {
            return;
        }
        sf::RectangleShape strip({end - start, thickness});
        strip.setOrigin(0.0f, thickness * 0.5f);
        // The strip is anchored at the start of the run, offset along the
        // centreline direction. We place it in world space using the same
        // parametrisation as drawRoadStrip, so the two are pixel-aligned
        // when the body and the border share the same endpoints.
        const float u = start * invLen;
        const sf::Vector2f runA = a + (b - a) * u;
        strip.setPosition(runA);
        strip.setRotation(angleDeg);
        strip.setFillColor(color);
        target.draw(strip);
    };

    for (float t = 0.0f; t < len; t += step) {
        const float u = t * invLen;
        const sf::Vector2f sample = a + (b - a) * u;
        if (isPointOnBody(sample.x, sample.y, bodyMask, gridW, gridH, cellSize)) {
            // Pixel hidden behind another road's body — close the current
            // visible run (if any) and skip ahead.
            if (runStart >= 0.0f) {
                emitRun(runStart, t);
                runStart = -1.0f;
            }
        } else {
            if (runStart < 0.0f) {
                runStart = t;
            }
        }
    }
    if (runStart >= 0.0f) {
        emitRun(runStart, len);
    }
}
