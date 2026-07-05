import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("bench_results.csv")
tp = pd.read_csv("throughput_results.csv")

algo_short = {
    "BFS (Basic - fewest roads)": "BFS",
    "Dijkstra (Congestion-aware)": "Dijkstra",
    "A* (Speed-optimized)": "A*",
}
df["algo"] = df["algorithm"].map(algo_short)

colors = {"BFS": "#4C72B0", "Dijkstra": "#DD8452", "A*": "#55A868"}

# ---- Chart 1: avg compute time per call, Grid20x20, Normal vs Heavy ----
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
for ax, scenario, title in zip(axes, ["Grid20x20-Normal", "Grid20x20-Heavy"],
                                ["Normal traffic", "Heavy congestion"]):
    sub = df[df.scenario == scenario]
    for algo in ["BFS", "Dijkstra", "A*"]:
        s = sub[sub.algo == algo].sort_values("vehicleCount")
        ax.plot(s.vehicleCount, s.avgTimeMs, marker="o", label=algo, color=colors[algo])
    ax.set_title(f"{title} (20x20 grid)")
    ax.set_xlabel("Number of vehicles")
    ax.set_xticks([100, 500, 1000])
axes[0].set_ylabel("Avg time per findPath() call (ms)")
axes[0].legend()
plt.tight_layout()
plt.savefig("chart_time_grid.png", dpi=150)
plt.close()

# ---- Chart 2: avg nodes explored, Grid20x20 ----
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
for ax, scenario, title in zip(axes, ["Grid20x20-Normal", "Grid20x20-Heavy"],
                                ["Normal traffic", "Heavy congestion"]):
    sub = df[df.scenario == scenario]
    for algo in ["BFS", "Dijkstra", "A*"]:
        s = sub[sub.algo == algo].sort_values("vehicleCount")
        ax.plot(s.vehicleCount, s.avgNodesExplored, marker="o", label=algo, color=colors[algo])
    ax.set_title(f"{title} (20x20 grid)")
    ax.set_xlabel("Number of vehicles")
    ax.set_xticks([100, 500, 1000])
axes[0].set_ylabel("Avg intersections explored per call")
axes[0].legend()
plt.tight_layout()
plt.savefig("chart_nodes_grid.png", dpi=150)
plt.close()

# ---- Chart 3: found-path rate on real map.json (heavy blocks small graph) ----
fig, ax = plt.subplots(figsize=(6.5, 4.5))
sub = df[df.scenario.isin(["RealMap-Normal", "RealMap-Heavy"])]
width = 0.25
xs = [0, 1, 2]
labels = ["n=100", "n=500", "n=1000"]
for i, algo in enumerate(["BFS", "Dijkstra", "A*"]):
    normal = sub[(sub.scenario == "RealMap-Normal") & (sub.algo == algo)].sort_values("vehicleCount")
    heavy = sub[(sub.scenario == "RealMap-Heavy") & (sub.algo == algo)].sort_values("vehicleCount")
    rate = (heavy.pathsFound.values / heavy.vehicleCount.values) * 100
    ax.bar([x + i * width for x in xs], rate, width=width, label=algo, color=colors[algo])
ax.set_xticks([x + width for x in xs])
ax.set_xticklabels(labels)
ax.set_ylabel("% of routes still findable under heavy congestion\n(real 5-node map.json)")
ax.set_title("Route availability under heavy traffic (real map)")
ax.legend()
plt.tight_layout()
plt.savefig("chart_realmap_found.png", dpi=150)
plt.close()

# ---- Chart 4: simulation throughput (logic FPS) ----
fig, ax = plt.subplots(figsize=(6.5, 4.5))
ax.plot(tp.vehicleCount, tp.logicFps, marker="o", color="#8172B2")
ax.set_xlabel("Number of vehicles in simulation")
ax.set_ylabel("Simulation throughput (logic ticks/sec)")
ax.set_title("Core-logic throughput vs vehicle count\n(no rendering, 20x20 grid, A* routing)")
ax.set_xticks([100, 500, 1000])
for x, y in zip(tp.vehicleCount, tp.logicFps):
    ax.annotate(f"{y:,.0f}", (x, y), textcoords="offset points", xytext=(0, 8), ha="center")
plt.tight_layout()
plt.savefig("chart_throughput.png", dpi=150)
plt.close()

print("Charts written.")
