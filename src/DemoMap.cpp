#include "DemoMap.h"

#include "model/Graph.h"
#include "model/Intersection.h"
#include "model/Road.h"

void populateDemoGraph(Graph& graph) {
    graph.clearGraph();

    Intersection* a = new Intersection(1, 15.0, 18.0);
    Intersection* b = new Intersection(2, 84.0, 20.0);
    Intersection* c = new Intersection(3, 90.0, 60.0);
    Intersection* d = new Intersection(4, 55.0, 90.0);
    Intersection* e = new Intersection(5, 16.0, 70.0);

    graph.addIntersection(a);
    graph.addIntersection(b);
    graph.addIntersection(c);
    graph.addIntersection(d);
    graph.addIntersection(e);

    graph.addRoad(new Road(1, "Road 1", a, b, 70.0, 50.0, 1.0));
    graph.addRoad(new Road(2, "Road 2", b, c, 45.0, 45.0, 1.8));
    graph.addRoad(new Road(3, "Road 3", c, d, 48.0, 40.0, 3.0));
    graph.addRoad(new Road(4, "Road 4", d, e, 42.0, 35.0, 2.1));
    graph.addRoad(new Road(5, "Road 5", e, a, 52.0, 50.0, 1.2));
    graph.addRoad(new Road(6, "Road 6", b, d, 68.0, 40.0, 4.0));
}
