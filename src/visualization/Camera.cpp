#include "Camera.h"

#include <algorithm>
#include <cmath>
#include <imgui.h>

#include "AppContext.h"
#include "visualization/VisualizationEngine.h"

void clampViewToMap(AppContext& ctx) {
    const sf::Vector2f size = ctx.view.getSize();
    const float halfWidth = size.x * 0.5f;
    const float halfHeight = size.y * 0.5f;

    const float minCenterX = ctx.mapMinX + halfWidth;
    const float maxCenterX = ctx.mapMaxX - halfWidth;
    const float minCenterY = ctx.mapMinY + halfHeight;
    const float maxCenterY = ctx.mapMaxY - halfHeight;

    sf::Vector2f center = ctx.view.getCenter();

    if (minCenterX <= maxCenterX) {
        center.x = std::clamp(center.x, minCenterX, maxCenterX);
    } else {
        center.x = (ctx.mapMinX + ctx.mapMaxX) * 0.5f;
    }

    if (minCenterY <= maxCenterY) {
        center.y = std::clamp(center.y, minCenterY, maxCenterY);
    } else {
        center.y = (ctx.mapMinY + ctx.mapMaxY) * 0.5f;
    }

    ctx.view.setCenter(center);
}

void refreshViewBounds(AppContext& ctx) {
    const auto& routePoints = ctx.visualization.getRoutePoints();
    if (routePoints.empty()) {
        ctx.mapMinX = 0.0f;
        ctx.mapMinY = 0.0f;
        ctx.mapMaxX = 100.0f;
        ctx.mapMaxY = 100.0f;
        return;
    }

    ctx.mapMinX = routePoints.front().x;
    ctx.mapMinY = routePoints.front().y;
    ctx.mapMaxX = routePoints.front().x;
    ctx.mapMaxY = routePoints.front().y;
    for (const auto& point : routePoints) {
        ctx.mapMinX = std::min(ctx.mapMinX, point.x);
        ctx.mapMinY = std::min(ctx.mapMinY, point.y);
        ctx.mapMaxX = std::max(ctx.mapMaxX, point.x);
        ctx.mapMaxY = std::max(ctx.mapMaxY, point.y);
    }

    const float cameraPadding = 60.0f;
    ctx.mapMinX -= cameraPadding;
    ctx.mapMinY -= cameraPadding;
    ctx.mapMaxX += cameraPadding;
    ctx.mapMaxY += cameraPadding;
    clampViewToMap(ctx);
}

void resetView(AppContext& ctx) {
    ctx.zoomFactor = 1.0f;
    ctx.view = ctx.window.getDefaultView();
    clampViewToMap(ctx);
}

void zoomBy(AppContext& ctx, float factor) {
    ctx.zoomFactor = std::clamp(ctx.zoomFactor * factor, 0.35f, 2.5f);
    ctx.view.setSize(static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                      static_cast<float>(ctx.windowH) * ctx.zoomFactor);
    clampViewToMap(ctx);
}

void zoomBy(AppContext& ctx, float factor, sf::Vector2f zoomCenter) {
    const float oldZoomFactor = ctx.zoomFactor;
    ctx.zoomFactor = std::clamp(ctx.zoomFactor * factor, 0.35f, 2.5f);
    const float actualFactor = ctx.zoomFactor / oldZoomFactor;

    ctx.view.setSize(static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                      static_cast<float>(ctx.windowH) * ctx.zoomFactor);

    sf::Vector2f center = ctx.view.getCenter();
    center = zoomCenter - (zoomCenter - center) * actualFactor;
    ctx.view.setCenter(center);
    clampViewToMap(ctx);
}

void updateCamera(AppContext& ctx, float dt) {
    if (!ctx.window.hasFocus()) {
        return;
    }

    sf::Vector2f movement(0.0f, 0.0f);

    // 1. Keyboard Panning (WASD or Arrow Keys)
    if (!ImGui::GetIO().WantCaptureKeyboard) {
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) || sf::Keyboard::isKeyPressed(sf::Keyboard::Up)) {
            movement.y -= 1.0f;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) || sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
            movement.y += 1.0f;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) || sf::Keyboard::isKeyPressed(sf::Keyboard::Left)) {
            movement.x -= 1.0f;
        }
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) || sf::Keyboard::isKeyPressed(sf::Keyboard::Right)) {
            movement.x += 1.0f;
        }
    }

    // 2. Edge Scrolling (only if not currently dragging with mouse)
    if (!ImGui::GetIO().WantCaptureMouse && !ctx.isDragging) {
        sf::Vector2i mousePos = sf::Mouse::getPosition(ctx.window);
        sf::Vector2u winSize = ctx.window.getSize();

        // Check if mouse is inside window bounds
        if (mousePos.x >= 0 && mousePos.x <= static_cast<int>(winSize.x) &&
            mousePos.y >= 0 && mousePos.y <= static_cast<int>(winSize.y)) {

            const int edgeThreshold = 10; // pixels from edge

            if (mousePos.x <= edgeThreshold) {
                movement.x -= 1.0f;
            }
            if (mousePos.x >= static_cast<int>(winSize.x) - edgeThreshold) {
                movement.x += 1.0f;
            }
            if (mousePos.y <= edgeThreshold) {
                movement.y -= 1.0f;
            }
            if (mousePos.y >= static_cast<int>(winSize.y) - edgeThreshold) {
                movement.y += 1.0f;
            }
        }
    }

    // Normalize and apply movement
    if (movement.x != 0.0f || movement.y != 0.0f) {
        float length = std::sqrt(movement.x * movement.x + movement.y * movement.y);
        movement /= length;

        const float panSpeed = 400.0f; // units per second
        float actualSpeed = panSpeed * ctx.zoomFactor * dt;

        ctx.view.move(movement * actualSpeed);
        clampViewToMap(ctx);
    }
}
