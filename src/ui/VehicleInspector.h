#ifndef VEHICLE_INSPECTOR_H
#define VEHICLE_INSPECTOR_H

#include <SFML/Graphics.hpp>

class TrafficSimulator;
class VisualizationEngine;
class Vehicle;
struct AppContext;

// Panel hiển thị thông tin chi tiết của MỘT xe được người dùng click chọn.
// Chỉ lưu vehicle ID (không lưu con trỏ Vehicle*) để tránh dangling pointer
// khi simulator bị reset (Load map / Demo map) — giống cách DebugConsole
// tránh giữ raw pointer xuyên suốt các frame.
class VehicleInspector {
public:
    // Gọi khi user click chuột trái vào map (đã qua mapPixelToCoords) và
    // không đang trong chế độ picking khác (Add Road / Spawn Vehicle) của
    // DebugConsole, và không click vào một ImGui window nào (WantCaptureMouse).
    //
    // Hành vi:
    //   - Click trúng xe (trong bán kính pickRadiusPixels)  -> chọn xe đó.
    //   - Click không trúng xe nào ("khoảng trống" trên map) -> bỏ chọn hiện tại.
    void tryPickVehicle(TrafficSimulator* simulator,
                         const VisualizationEngine& visualization,
                         const sf::Vector2f& screenPos,
                         float pickRadiusPixels = 14.0f);

    void clearSelection() { selectedVehicleId_ = -1; }
    bool hasSelection() const { return selectedVehicleId_ != -1; }
    int getSelectedId() const { return selectedVehicleId_; }

    // Vẽ ImGui panel nếu đang có xe được chọn. Tự động cập nhật số liệu
    // real-time (vì luôn đọc lại Vehicle* mới nhất mỗi frame).
    void draw(TrafficSimulator* simulator,
              const VisualizationEngine& visualization,
              AppContext& ctx);

private:
    Vehicle* findSelectedVehicle(TrafficSimulator* simulator) const;

    int selectedVehicleId_ = -1;
};

#endif
