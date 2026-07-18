#ifndef MAP_LOADING_H
#define MAP_LOADING_H

#include <string>

struct AppContext;

// Attempts to load `requestedPath` (trying a few relative-path fallbacks,
// same as before) into ctx.graph. On success, updates ctx.usingDemoMap /
// mapPathInput / loadError and returns true. On failure - including an
// empty requestedPath - falls back to the built-in demo graph and returns
// false.
bool loadGraphFromPath(AppContext& ctx, const std::string& requestedPath);

// loadGraphFromPath(), then re-prepares the visualization, recomputes the
// camera bounds, and resets the view - the one-stop entry point used both
// at startup and from the DebugConsole's Load map / Demo map buttons.
void loadAndRefresh(AppContext& ctx, const std::string& requestedPath);

#endif
