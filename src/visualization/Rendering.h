#ifndef RENDERING_H
#define RENDERING_H

#include <memory>

struct AppContext;
class DebugConsole;
class StatsPanel;
class TrafficSimulator;

// Draws one full frame: the map/graph, vehicle sprites, failed-recalc
// markers, the "parked" boxes of finished vehicles grouped by destination,
// the stats panel, and the debug console - then renders ImGui and presents
// the window. Mirrors the per-frame drawing block that used to live
// directly inside main()'s loop, after simulator->update(dt).
void renderFrame(AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator, StatsPanel& statsPanel, float dt);

#endif
