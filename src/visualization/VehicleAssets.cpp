#include "VehicleAssets.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace {

constexpr sf::Uint8 kVisibleAlphaThreshold = 8;

struct VehicleAssetFile {
    VehicleKind kind;
    const char* name;
};

constexpr std::array<VehicleAssetFile, 4> kVehicleAssetFiles{{
    {VehicleKind::Car, "car.png"},
    {VehicleKind::Bus, "bus.png"},
    {VehicleKind::Motorbike, "motorbike.png"},
    {VehicleKind::Emergency, "ambulance.png"}
}};

sf::IntRect visibleBounds(const sf::Image& image) {
    const sf::Vector2u size = image.getSize();
    int minX = static_cast<int>(size.x);
    int minY = static_cast<int>(size.y);
    int maxX = -1;
    int maxY = -1;

    for (unsigned int y = 0; y < size.y; ++y) {
        for (unsigned int x = 0; x < size.x; ++x) {
            if (image.getPixel(x, y).a <=
                kVisibleAlphaThreshold) {
                continue;
            }
            minX = std::min(minX, static_cast<int>(x));
            minY = std::min(minY, static_cast<int>(y));
            maxX = std::max(maxX, static_cast<int>(x));
            maxY = std::max(maxY, static_cast<int>(y));
        }
    }

    if (maxX < minX || maxY < minY) {
        return {};
    }
    return {
        minX,
        minY,
        maxX - minX + 1,
        maxY - minY + 1
    };
}

std::vector<std::filesystem::path> assetDirectories() {
    const std::filesystem::path current =
        std::filesystem::current_path();
    return {
        current / "assets",
        current / "assets" / "vehicles",
        current.parent_path() / "assets",
        current.parent_path() / "assets" / "vehicles",
        current.parent_path().parent_path() / "assets",
        current.parent_path().parent_path() /
            "assets" / "vehicles"
    };
}

} // namespace

VehicleAssets& VehicleAssets::instance() {
    static VehicleAssets assets;
    return assets;
}

std::size_t VehicleAssets::indexFor(VehicleKind kind) {
    switch (kind) {
        case VehicleKind::Bus:
            return 1u;
        case VehicleKind::Motorbike:
            return 2u;
        case VehicleKind::Emergency:
            return 3u;
        case VehicleKind::Car:
        default:
            return 0u;
    }
}

void VehicleAssets::loadOnce() {
    if (loadAttempted_) {
        return;
    }
    loadAttempted_ = true;

    const std::vector<std::filesystem::path> directories =
        assetDirectories();
    int loadedCount = 0;
    for (const VehicleAssetFile& asset : kVehicleAssetFiles) {
        std::filesystem::path selectedPath;
        for (const std::filesystem::path& directory :
             directories) {
            const std::filesystem::path candidate =
                directory / asset.name;
            std::error_code error;
            if (std::filesystem::is_regular_file(
                    candidate, error)) {
                selectedPath = candidate;
                break;
            }
        }
        if (selectedPath.empty()) {
            continue;
        }

        sf::Image image;
        if (!image.loadFromFile(selectedPath.string())) {
            continue;
        }
        const sf::IntRect bounds = visibleBounds(image);
        if (bounds.width <= 0 || bounds.height <= 0) {
            continue;
        }

        const std::size_t index = indexFor(asset.kind);
        if (!textures_[index].loadFromImage(image, bounds)) {
            continue;
        }
        textures_[index].setSmooth(true);
        textures_[index].generateMipmap();
        loaded_[index] = true;
        ++loadedCount;
    }

    if (loadedCount !=
        static_cast<int>(kVehicleAssetFiles.size())) {
        std::cerr
            << "Warning: loaded " << loadedCount << "/"
            << kVehicleAssetFiles.size()
            << " vehicle sprites; missing types use the "
               "rectangle fallback."
            << std::endl;
    }
}

const sf::Texture* VehicleAssets::textureFor(
    VehicleKind kind) {
    loadOnce();
    const std::size_t index = indexFor(kind);
    return loaded_[index] ? &textures_[index] : nullptr;
}

bool VehicleAssets::hasTexture(VehicleKind kind) {
    return textureFor(kind) != nullptr;
}
