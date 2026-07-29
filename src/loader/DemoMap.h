#ifndef DEMO_MAP_H
#define DEMO_MAP_H

class Graph;

// Populates `graph` with a small built-in 5-intersection / 6-road map, used
// whenever no external map JSON is available (no path given, file missing,
// or failed to parse).
void populateDemoGraph(Graph& graph);

#endif
