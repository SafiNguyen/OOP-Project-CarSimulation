#include "Camera.h"

#include <algorithm>
#include <cmath>
#include <imgui.h>

#include "AppContext.h"
#include "Graph.h"
#include "model/vehicle/Vehicle.h"
#include "simulation/TrafficSimulator.h"
#include "visualization/VisualizationEngine.h"

namespace {
constexpr float FOLLOW_ZOOM_FACTOR = 0.18f;
constexpr float FOLLOW_RESPONSE = 10.0f;

const Vehicle* findActiveVehicleById(
    const TrafficSimulator* simulator,
    int vehicleId) {
    if (simulator == nullptr || vehicleId < 0) {
        return nullptr;
    }

    for (const Vehicle* vehicle : simulator->getVehicles()) {
        if (vehicle != nullptr && vehicle->getId() == vehicleId) {
            return vehicle;
        }
    }
    return nullptr;
}

float followBlend(float dt) {
    const float safeDt = std::clamp(dt, 0.0f, 0.1f);
    return 1.0f - std::exp(-FOLLOW_RESPONSE * safeDt);
}
} // namespace

void clampViewToMap(AppContext& ctx) {
    const sf::Vector2f size = ctx.view.getSize();
    const float halfWidth = size.x * 0.5f;
    const float halfHeight = size.y * 0.5f;

    sf::Vector2f center = ctx.view.getCenter();
    const float mapWidth = ctx.mapMaxX - ctx.mapMinX;
    const float mapHeight = ctx.mapMaxY - ctx.mapMinY;

    if (mapWidth > size.x) {
        const float minCenterX = ctx.mapMinX + halfWidth;
        const float maxCenterX = ctx.mapMaxX - halfWidth;
        center.x = std::clamp(center.x, minCenterX, maxCenterX);
    }

    if (mapHeight > size.y) {
        const float minCenterY = ctx.mapMinY + halfHeight;
        const float maxCenterY = ctx.mapMaxY - halfHeight;
        center.y = std::clamp(center.y, minCenterY, maxCenterY);
    }

    ctx.view.setCenter(center);
}

void refreshViewBounds(AppContext& ctx) {
    const auto& routePoints =
        ctx.visualization.getRoutePoints();
    if (routePoints.empty()) {
        ctx.mapMinX = 0.0f;
        ctx.mapMinY = 0.0f;
        ctx.mapMaxX = 100.0f;
        ctx.mapMaxY = 100.0f;
        clampViewToMap(ctx);
        return;
    }

    ctx.mapMinX = routePoints.front().x;
    ctx.mapMinY = routePoints.front().y;
    ctx.mapMaxX = routePoints.front().x;
    ctx.mapMaxY = routePoints.front().y;

    for (const sf::Vector2f& point : routePoints) {
        ctx.mapMinX = std::min(ctx.mapMinX, point.x);
        ctx.mapMinY = std::min(ctx.mapMinY, point.y);
        ctx.mapMaxX = std::max(ctx.mapMaxX, point.x);
        ctx.mapMaxY = std::max(ctx.mapMaxY, point.y);
    }

    const float cameraPadding = 160.0f;
    ctx.mapMinX -= cameraPadding;
    ctx.mapMinY -= cameraPadding;
    ctx.mapMaxX += cameraPadding;
    ctx.mapMaxY += cameraPadding;
    clampViewToMap(ctx);
}

void resetView(AppContext& ctx) {
    stopFollowingVehicle(ctx);
    ctx.zoomFactor = DEFAULT_MAP_ZOOM_FACTOR;
    ctx.view = ctx.window.getDefaultView();
    ctx.view.setSize(
        static_cast<float>(ctx.windowW) *
            ctx.zoomFactor,
        static_cast<float>(ctx.windowH) *
            ctx.zoomFactor);
    clampViewToMap(ctx);
}

void startFollowingVehicle(AppContext& ctx, int vehicleId) {
    if (vehicleId < 0) {
        return;
    }
    ctx.followedVehicleId = vehicleId;
    ctx.isDragging = false;
}

void stopFollowingVehicle(AppContext& ctx) {
    ctx.followedVehicleId = -1;
    ctx.isDragging = false;
}

bool isFollowingVehicle(const AppContext& ctx) {
    return ctx.followedVehicleId >= 0;
}

bool isFollowingVehicle(const AppContext& ctx, int vehicleId) {
    return vehicleId >= 0 && ctx.followedVehicleId == vehicleId;
}

void zoomBy(AppContext& ctx, float factor) {
    const float oldZoomFactor = ctx.zoomFactor;
    const float newZoomFactor = std::clamp(oldZoomFactor * factor, 0.02f, 5.0f);
    if (std::abs(newZoomFactor - oldZoomFactor) < 0.0001f) {
        return;
    }

    ctx.zoomFactor = newZoomFactor;
    ctx.view.setSize(static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                      static_cast<float>(ctx.windowH) * ctx.zoomFactor);
}

void zoomBy(AppContext& ctx, float factor, sf::Vector2f zoomCenter) {
    const float oldZoomFactor = ctx.zoomFactor;
    const float newZoomFactor = std::clamp(oldZoomFactor * factor, 0.02f, 5.0f);
    if (std::abs(newZoomFactor - oldZoomFactor) < 0.0001f) {
        return;
    }

    const float actualFactor = (oldZoomFactor > 0.0001f) ? (newZoomFactor / oldZoomFactor) : 1.0f;
    const sf::Vector2f previousCenter = ctx.view.getCenter();

    ctx.zoomFactor = newZoomFactor;
    ctx.view.setSize(static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                      static_cast<float>(ctx.windowH) * ctx.zoomFactor);

    const sf::Vector2f newCenter = zoomCenter - (zoomCenter - previousCenter) * actualFactor;
    ctx.view.setCenter(newCenter);
}

void updateCamera(AppContext& ctx,
                  const TrafficSimulator* simulator,
                  float dt) {
    if (isFollowingVehicle(ctx)) {
        const Vehicle* target = findActiveVehicleById(
            simulator,
            ctx.followedVehicleId);
        if (target == nullptr || target->getCurrentRoad() == nullptr) {
            stopFollowingVehicle(ctx);
        } else {
            const Pose2D pose = target->getPose();
            const sf::Vector2f targetCenter =
                ctx.visualization.worldToScreen(
                    pose.position.x,
                    pose.position.y);
            const float blend = followBlend(dt);

            ctx.zoomFactor +=
                (FOLLOW_ZOOM_FACTOR - ctx.zoomFactor) * blend;
            if (std::abs(FOLLOW_ZOOM_FACTOR - ctx.zoomFactor) < 0.0001f) {
                ctx.zoomFactor = FOLLOW_ZOOM_FACTOR;
            }
            ctx.view.setSize(
                static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                static_cast<float>(ctx.windowH) * ctx.zoomFactor);

            const sf::Vector2f center = ctx.view.getCenter();
            ctx.view.setCenter(
                center + (targetCenter - center) * blend);
            return;
        }
    }

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
    }
}
