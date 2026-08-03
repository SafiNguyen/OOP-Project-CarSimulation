#include "MapLoading.h"

#include <vector>

#include "AppContext.h"
#include "visualization/Camera.h"
#include "DemoMap.h"
#include "Mapload.h"
#include "Graph.h"
#include "visualization/VisualizationEngine.h"

bool loadGraphFromPath(AppContext& ctx, const std::string& requestedPath) {
    if (requestedPath.empty()) {
        populateDemoGraph(ctx.graph);
        ctx.usingDemoMap = true;
        ctx.loadError.clear();
        return true;
    }

    std::string searchPath = requestedPath;
    if (searchPath.length() < 5 || searchPath.substr(searchPath.length() - 5) != ".json") {
        searchPath += ".json";
    }

    std::string fileLoadError;
    std::vector<std::string> searchPaths = {
        searchPath,
        "../" + searchPath,
        "../../" + searchPath,
        "../../../" + searchPath
    };

    for (const auto& p : searchPaths) {
        if (MapLoad::loadGraphFromJsonFile(p, ctx.graph, &fileLoadError)) {
            ctx.usingDemoMap = false;
            ctx.loadError.clear();
            ctx.mapPathInput = requestedPath;
            return true;
        }
    }

    populateDemoGraph(ctx.graph);
    ctx.usingDemoMap = true;
    ctx.loadError = fileLoadError;
    return false;
}

void loadAndRefresh(AppContext& ctx, const std::string& requestedPath) {
    try {
        loadGraphFromPath(ctx, requestedPath);
        ctx.visualization.prepare(ctx.graph);
        refreshViewBounds(ctx);
        resetView(ctx);
    } catch (const std::exception& ex) {
        populateDemoGraph(ctx.graph);
        ctx.usingDemoMap = true;
        ctx.loadError = std::string("Exception while loading map: ") + ex.what();
        ctx.visualization.prepare(ctx.graph);
        refreshViewBounds(ctx);
        resetView(ctx);
    } catch (...) {
        populateDemoGraph(ctx.graph);
        ctx.usingDemoMap = true;
        ctx.loadError = "Unknown exception while loading map";
        ctx.visualization.prepare(ctx.graph);
        refreshViewBounds(ctx);
        resetView(ctx);
    }
}
