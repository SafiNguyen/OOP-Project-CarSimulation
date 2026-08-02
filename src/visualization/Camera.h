#ifndef CAMERA_H
#define CAMERA_H

#include <SFML/System/Vector2.hpp>

struct AppContext;
class TrafficSimulator;

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
// Also leaves vehicle-follow mode, without changing simulation state.
void resetView(AppContext& ctx);

// Vehicle follow state is stored by id so camera updates never retain a
// Vehicle pointer across simulation frames or snapshot restores.
void startFollowingVehicle(AppContext& ctx, int vehicleId);
void stopFollowingVehicle(AppContext& ctx);
bool isFollowingVehicle(const AppContext& ctx);
bool isFollowingVehicle(const AppContext& ctx, int vehicleId);

// Multiplies zoomFactor by `factor` (clamped to [0.35, 2.5]), resizes the
// view to match, and re-clamps.
void zoomBy(AppContext& ctx, float factor);

// Overload that zooms centered on a specific world coordinates position.
void zoomBy(AppContext& ctx, float factor, sf::Vector2f zoomCenter);

// Updates either the vehicle-follow camera or the normal frame-rate
// independent RTS camera (edge scroll, WASD and arrow-key panning).
void updateCamera(AppContext& ctx,
                  const TrafficSimulator* simulator,
                  float dt);

#endif
