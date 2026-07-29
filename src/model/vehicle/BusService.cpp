#include "BusService.h"

#include <utility>

BusService::BusService(
    int id,
    std::string code,
    std::string name,
    const BusStation& originStation,
    const BusStation& destinationStation,
    std::vector<Road*> roadRoute,
    std::vector<const BusStop*> orderedStops)
    : id_(id),
      code_(std::move(code)),
      name_(std::move(name)),
      originStation_(&originStation),
      destinationStation_(&destinationStation),
      roadRoute_(std::move(roadRoute)),
      orderedStops_(std::move(orderedStops)) {
}
