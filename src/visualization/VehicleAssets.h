#ifndef VEHICLE_ASSETS_H
#define VEHICLE_ASSETS_H

#include <SFML/Graphics.hpp>

#include <array>

#include "model/vehicle/VehicleTypes.h"

class VehicleAssets {
public:
    static VehicleAssets& instance();

    const sf::Texture* textureFor(VehicleKind kind);
    bool hasTexture(VehicleKind kind);

private:
    VehicleAssets() = default;

    void loadOnce();
    static std::size_t indexFor(VehicleKind kind);

    bool loadAttempted_ = false;
    std::array<sf::Texture, 4> textures_;
    std::array<bool, 4> loaded_{{false, false, false, false}};
};

#endif
