#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <SFML/Graphics.hpp>
#include <cstdint>
#include <string>

class Graph;
class VisualizationEngine;

// Bundles the camera/view and map-loading state that used to live as plain
// locals + captured lambdas inside main(). Passed by reference into the
// free functions in Camera.h / MapLoading.h / InputHandling.h / Rendering.h
// so none of them need a dozen separate parameters each.
struct AppContext {
    AppContext(Graph& graphRef, VisualizationEngine& visualizationRef,
               sf::RenderWindow& windowRef, unsigned int w, unsigned int h)
        : graph(graphRef), visualization(visualizationRef), window(windowRef),
          windowW(w), windowH(h) {}

    Graph& graph;
    VisualizationEngine& visualization;
    sf::RenderWindow& window;

    unsigned int windowW;
    unsigned int windowH;

    sf::View view;
    float zoomFactor = 1.0f;
    bool isDragging = false;
    sf::Vector2i lastMousePixel;

    float mapMinX = 0.0f;
    float mapMinY = 0.0f;
    float mapMaxX = 100.0f;
    float mapMaxY = 100.0f;

    std::string mapPathInput;
    bool usingDemoMap = false;
    std::string loadError;
    bool heatMapEnabled = true;
    bool showParkedVehicles = true;

    // drawGraph() builds hundreds of shapes and a CPU road-overlap mask.
    // Keep its result on the GPU and only rebuild it when the visualization
    // geometry changes or when the lower-frequency map-state refresh is due.
    sf::RenderTexture mapCacheTexture;
    sf::Sprite mapCacheSprite;
    sf::Clock mapCacheRefreshClock;
    std::uint64_t mapCacheRevision = 0;
    bool mapCacheReady = false;
};

#endif
