#include "InputHandling.h"

#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "visualization/Camera.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"

void handleEvent(const sf::Event& event, AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator) {
    ImGui::SFML::ProcessEvent(ctx.window, event);

    if (event.type == sf::Event::Closed) {
        ctx.window.close();
        return;
    }

    // 1. Process camera dragging BEFORE ImGui checks, so dragging is never interrupted or stuck,
    // and use exact event-based coordinates to eliminate mouse lag/jitter.
    if (event.type == sf::Event::MouseButtonPressed && 
        (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right)) {
        if (!ImGui::GetIO().WantCaptureMouse) {
            ctx.isDragging = true;
            ctx.lastMousePixel = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
        }
    }
    if (event.type == sf::Event::MouseButtonReleased && 
        (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right)) {
        ctx.isDragging = false;
    }
    if (event.type == sf::Event::MouseMoved && ctx.isDragging) {
        sf::Vector2i mousePixel(event.mouseMove.x, event.mouseMove.y);
        sf::Vector2i delta = mousePixel - ctx.lastMousePixel;
        ctx.lastMousePixel = mousePixel;

        ctx.view.move(-static_cast<float>(delta.x) * ctx.zoomFactor,
                       -static_cast<float>(delta.y) * ctx.zoomFactor);
        clampViewToMap(ctx);
        return; // Consumed
    }

    // 2. Early return for ImGui capture for other interactions (e.g. left click, keys)
    if (ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard) {
        return;
    }

    if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
        ctx.window.close();
    }
    if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space) {
        if (simulator && simulator->isPaused()) {
            simulator->resume();
        } else if (simulator) {
            simulator->pause();
        }
    }
    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left
        && debugConsole.isPicking()) {
        const sf::Vector2i pixel(event.mouseButton.x, event.mouseButton.y);
        const sf::Vector2f worldPixel = ctx.window.mapPixelToCoords(pixel, ctx.view);
        debugConsole.handleMapClick(ctx.graph, ctx.visualization, worldPixel);
    }
    if (event.type == sf::Event::MouseWheelScrolled) {
        const float factor = (event.mouseWheelScroll.delta > 0.0f) ? 0.9f : 1.1f;
        const sf::Vector2i mousePixel(event.mouseWheelScroll.x, event.mouseWheelScroll.y);
        const sf::Vector2f worldPos = ctx.window.mapPixelToCoords(mousePixel, ctx.view);
        zoomBy(ctx, factor, worldPos);
    }
    if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Add || event.key.code == sf::Keyboard::Equal) {
            zoomBy(ctx, 0.9f);
        }
        if (event.key.code == sf::Keyboard::Subtract || event.key.code == sf::Keyboard::Hyphen) {
            zoomBy(ctx, 1.1f);
        }
        if (event.key.code == sf::Keyboard::R) {
            resetView(ctx);
        }
    }
}
