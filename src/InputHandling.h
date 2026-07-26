#ifndef INPUT_HANDLING_H
#define INPUT_HANDLING_H

#include <memory>

#include <SFML/Graphics.hpp>

struct AppContext;
class DebugConsole;
class TrafficSimulator;
class VehicleInspector;


void handleEvent(const sf::Event& event, AppContext& ctx, DebugConsole& debugConsole,
                  std::unique_ptr<TrafficSimulator>& simulator,
                  VehicleInspector& vehicleInspector);

#endif
