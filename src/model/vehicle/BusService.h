#ifndef BUS_SERVICE_H
#define BUS_SERVICE_H

#include <string>
#include <vector>

class BusStation;
class BusStop;
class Road;

class BusService {
public:
    BusService(int id,
               std::string code,
               std::string name,
               const BusStation& originStation,
               const BusStation& destinationStation,
               std::vector<Road*> roadRoute,
               std::vector<const BusStop*> orderedStops);

    BusService(const BusService&) = delete;
    BusService& operator=(const BusService&) = delete;
    BusService(BusService&&) = delete;
    BusService& operator=(BusService&&) = delete;

    int getId() const { return id_; }
    const std::string& getCode() const { return code_; }
    const std::string& getName() const { return name_; }
    const BusStation& getOriginStation() const {
        return *originStation_;
    }
    const BusStation& getDestinationStation() const {
        return *destinationStation_;
    }
    const std::vector<Road*>& getRoadRoute() const {
        return roadRoute_;
    }
    const std::vector<const BusStop*>& getOrderedStops() const {
        return orderedStops_;
    }

private:
    int id_;
    std::string code_;
    std::string name_;
    const BusStation* originStation_;
    const BusStation* destinationStation_;
    std::vector<Road*> roadRoute_;
    std::vector<const BusStop*> orderedStops_;
};

#endif
