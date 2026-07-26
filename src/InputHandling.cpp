#include "InputHandling.h"

#include <imgui-SFML.h>
#include <imgui.h>

#include "AppContext.h"
#include "visualization/Camera.h"
#include "visualization/VisualizationEngine.h"
#include "simulation/TrafficSimulator.h"
#include "ui/DebugConsole.h"
#include "ui/VehicleInspector.h"

void handleEvent(const sf::Event& event, AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator,
                  VehicleInspector& vehicleInspector) {
    ImGui::SFML::ProcessEvent(ctx.window, event);

    if (event.type == sf::Event::Closed) {
        ctx.window.close();
        return;
    }

    if (event.type == sf::Event::Resized) {
        ctx.windowW = event.size.width;
        ctx.windowH = event.size.height;

        ctx.visualization.setWindowSize({ctx.windowW, ctx.windowH});
        ctx.visualization.prepare(ctx.graph);

        const sf::Vector2f center = ctx.view.getCenter();
        ctx.view.setSize(static_cast<float>(ctx.windowW) * ctx.zoomFactor,
                         static_cast<float>(ctx.windowH) * ctx.zoomFactor);
        ctx.view.setCenter(center);
        clampViewToMap(ctx);
        ctx.window.setView(ctx.view);
        return;
    }

    // 1. Early return for ImGui capture for all interactions (UI controls, buttons, text inputs)
    // Quan trọng cho vehicle-picking: nếu click đang rơi vào một ImGui window
    // (kể cả chính VehicleInspector panel đang mở), ta return ở đây TRƯỚC
    // khi chạy tới đoạn vehicle-picking bên dưới. Nhờ vậy bấm nút "Close"
    // hoặc kéo route-list bên trong panel không bị hiểu lầm thành
    // "click ra khoảng trống trên map" và vô tình bỏ chọn xe.
    if (ImGui::GetIO().WantCaptureMouse || ImGui::GetIO().WantCaptureKeyboard) {
        ctx.isDragging = false;
        return;
    }

    if (event.type == sf::Event::MouseButtonPressed && 
        (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right || event.mouseButton.button == sf::Mouse::Left)) {
        // Only allow left click drag if not picking
        if (event.mouseButton.button == sf::Mouse::Left && debugConsole.isPicking()) {
            // Do nothing here, it's handled below
        } else {
            ctx.isDragging = true;
            ctx.lastMousePixel = sf::Vector2i(event.mouseButton.x, event.mouseButton.y);
        }
    }
    if (event.type == sf::Event::MouseButtonReleased && 
        (event.mouseButton.button == sf::Mouse::Middle || event.mouseButton.button == sf::Mouse::Right || event.mouseButton.button == sf::Mouse::Left)) {
        ctx.isDragging = false;
    }
    if (event.type == sf::Event::MouseMoved && ctx.isDragging) {
        sf::Vector2i mousePixel(event.mouseMove.x, event.mouseMove.y);
        sf::Vector2i delta = mousePixel - ctx.lastMousePixel;
        ctx.lastMousePixel = mousePixel;

        ctx.view.move(-static_cast<float>(delta.x) * ctx.zoomFactor,
                      -static_cast<float>(delta.y) * ctx.zoomFactor);
        return; // Consumed
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

    // --- Vehicle Inspector picking ---
    // Đặt SAU đoạn debugConsole.isPicking() ở trên và có điều kiện
    // !debugConsole.isPicking() để hai cơ chế picking không giẫm chân nhau:
    // khi user đang bấm "Pick" để chọn Start/End cho Add Road hoặc Spawn
    // Vehicle, một click trên map phải đi vào handleMapClick() ở trên,
    // KHÔNG được vô tình chọn/bỏ chọn xe.
    //
    // tryPickVehicle tự xử lý cả hai chiều:
    //   - click trúng xe        -> chọn xe đó (mở/thay nội dung panel)
    //   - click vào khoảng trống -> bỏ chọn (đóng panel nếu đang mở)
    if (event.type == sf::Event::MouseButtonPressed
        && event.mouseButton.button == sf::Mouse::Left
        && !debugConsole.isPicking()) {
        const sf::Vector2i pixel(event.mouseButton.x, event.mouseButton.y);
        const sf::Vector2f worldPixel = ctx.window.mapPixelToCoords(pixel, ctx.view);
        vehicleInspector.tryPickVehicle(simulator.get(), ctx.visualization, worldPixel);
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
