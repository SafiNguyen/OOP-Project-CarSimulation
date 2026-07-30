#ifndef CAMERA_H
#define CAMERA_H

#include <SFML/System/Vector2.hpp>

struct AppContext;

inline constexpr float DEFAULT_MAP_ZOOM_FACTOR = 0.82f;

// View/zoom helpers factored out of main(). All operate on the shared
// AppContext (view, zoomFactor, map bounds) instead of a pile of captured
// locals.

// Clamps the current view's center so it never scrolls past the known map
// bounds (mapMinX/Y .. mapMaxX/Y in AppContext).
void clampViewToMap(AppContext& ctx);

// Recomputes the map bounds from the visualization's current route points
// (with padding), falling back to a default 100x100 box when there are no
// points yet. Also re-clamps the view afterward.
void refreshViewBounds(AppContext& ctx);

// Resets to the default close map view and clamps it to the map bounds.
void resetView(AppContext& ctx);

// Multiplies zoomFactor by `factor` (clamped to [0.35, 2.5]), resizes the
// view to match, and re-clamps.
void zoomBy(AppContext& ctx, float factor);

// Overload that zooms centered on a specific world coordinates position.
void zoomBy(AppContext& ctx, float factor, sf::Vector2f zoomCenter);

// Frame-rate independent RTS camera update (Edge scroll, WASD & Arrow Key panning).
void updateCamera(AppContext& ctx, float dt);

#endif
