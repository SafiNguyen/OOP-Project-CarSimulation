# Experiment Results — Stress Test & Algorithm Comparison

**Người thực hiện:** TV2 – Data Analyst
**Ngày:** Chủ Nhật (tuần build cuối)
**File nguồn:** `tests/stress_test.cpp` (chương trình benchmark độc lập, không phụ thuộc SFML)
**Dữ liệu thô:** `bench_results.csv`, `throughput_results.csv`

## 1. Mục tiêu

Theo kế hoạch tuần, nhiệm vụ hôm nay là:

- Sinh 100 → 1000 xe, đo hiệu năng hệ thống.
- So sánh BFS / Dijkstra / A* trong điều kiện giao thông bình thường **và** kẹt xe nặng.
- Tổng hợp bảng/biểu đồ làm nguyên liệu cho phần "Experimentation" của báo cáo cuối kỳ.

Vì bản đồ legacy 5 intersection / 6 road quá nhỏ để thấy khác biệt rõ giữa các thuật toán ở quy mô 1000 xe, bài đo dùng **hai bản đồ**:

1. **RealMap** — bản đồ legacy 5 intersection / 6 road được nhúng trực tiếp trong `stress_test.cpp`, dùng để xác nhận hành vi trên đồ thị nhỏ.
2. **Grid20x20** — lưới tổng hợp 20×20 (400 intersection, ~1520 road một chiều) sinh riêng cho stress test, để có đủ "không gian" cho 1000 cặp điểm đi/đến khác nhau và số liệu có ý nghĩa thống kê.

Hai kịch bản giao thông:
- **Normal**: `congestionLevel = 1.0` toàn bộ, không road nào bị chặn.
- **Heavy**: ~35% road bị gán `congestionLevel` ngẫu nhiên 3.0–6.0, ~5% road bị `blockRoad()` (mô phỏng tai nạn/đóng đường).

Mỗi thuật toán được benchmark bằng đúng cơ chế đo mà TV2 đã viết ở `StatisticsManager::measurePathfinding()` (dùng `std::chrono::steady_clock`), gọi trực tiếp `findPath()` cho từng cặp start/goal — không đi qua `TrafficSimulator`, đúng như thiết kế "benchmark độc lập" của Thứ 5.

## 2. Thời gian tính toán trung bình mỗi lần `findPath()`

![Compute time per pathfinding call](chart_time_grid.png)

Trên bản đồ lưới 20×20:

| Thuật toán | Normal (n=1000) | Heavy (n=1000) |
|---|---|---|
| BFS | 0.0376 ms | 0.0395 ms |
| Dijkstra | 0.0840 ms | 0.0996 ms |
| A* | 0.0606 ms | 0.0738 ms |

BFS luôn nhanh nhất vì chỉ đếm số hop, bỏ qua chi phí cạnh (ngoại trừ road bị `blocked` — vẫn bị loại). Dijkstra chậm nhất vì duyệt đầy đủ theo chi phí thời gian thật. A* nằm giữa: chậm hơn BFS (phải tính heuristic + duy trì gScore) nhưng nhanh hơn Dijkstra đáng kể nhờ heuristic Euclidean cắt bớt không gian tìm kiếm.

## 3. Số intersection duyệt qua trung bình (hiệu quả thuật toán)

![Nodes explored per call](chart_nodes_grid.png)

| Thuật toán | Normal (n=1000) | Heavy (n=1000) |
|---|---|---|
| BFS | 182.2 | 182.7 |
| Dijkstra | 200.5 | 201.2 |
| A* | 69.7 | 84.4 |

Đây là số liệu rõ ràng nhất cho thấy lợi ích của A*: **so với Dijkstra, A* duyệt ít hơn ~65% số intersection** để tìm đường tối ưu như nhau (heuristic admissible nên A* vẫn cho `totalCost` giống hệt Dijkstra — có thể kiểm chứng trong `bench_results.csv`, cột `avgPathCost` bằng nhau giữa hai thuật toán ở mọi dòng). Khi kẹt xe nặng, A* phải duyệt thêm vì heuristic (dựa trên tốc độ tối đa lý thuyết) trở nên kém sát với chi phí thật hơn — nhưng vẫn duyệt ít hơn Dijkstra khoảng 2.4 lần.

## 4. Khả năng tìm được đường khi kẹt xe nặng (trên RealMap thật)

![Route availability real map](chart_realmap_found.png)

Trên bản đồ legacy, road duy nhất nối `5 → 3` (id 106) vốn đã bị đánh dấu `blocked`. Khi mô phỏng "Heavy" (random block thêm ~5% road trên đồ thị chỉ có 6 road), có kịch bản ngẫu nhiên khiến road `2 → 5` (id 105) — con đường duy nhất dẫn tới node 5 — cũng bị chặn, khiến **45% cặp start/goal không còn đường đi** ở cả 3 thuật toán như nhau (found=55/100). Điều này minh họa đúng tính chất: **BFS/Dijkstra/A* đều là thuật toán đúng đắn (correct)** — khi đồ thị thực sự mất kết nối, không thuật toán nào "tìm ra đường" được, khác biệt giữa chúng chỉ nằm ở *tốc độ* và *chất lượng lộ trình* (chi phí), không phải ở việc có tìm ra hay không.

## 5. Chi phí lộ trình tìm được (đường có bị "xấu đi" khi kẹt xe không?)

Trên RealMap, so `avgPathCost` giữa Normal và Heavy (n=1000):

| Thuật toán | Normal | Heavy | Chênh lệch |
|---|---|---|---|
| BFS | 2.22 (đơn vị: số hop) | 2.22 | không đổi — BFS không quan tâm chi phí |
| Dijkstra | 4.75 s | 11.61 s | **+144%** |
| A* | 4.75 s | 11.61 s | **+144%** |

BFS luôn chọn đường ít hop nhất bất kể kẹt xe (nó không nhìn thấy congestion), nên chi phí thời gian thật của lộ trình BFS chọn có thể tệ hơn nhiều so với Dijkstra/A* trong điều kiện kẹt xe — đây là lý do dự án dùng Dijkstra làm "Congestion-aware" router mặc định cho việc routing thật, còn BFS chỉ dùng để tham khảo/so sánh.

## 6. Thông lượng mô phỏng (Simulation Throughput / "logic FPS")

![Simulation throughput](chart_throughput.png)

Đo bằng cách chạy `TrafficSimulator::update(dt)` 300 tick liên tiếp (không vẽ, không SFML) với số xe tăng dần trên lưới 20×20, dùng A* làm chiến lược định tuyến mặc định:

| Số xe | Thời gian trung bình / tick | Thông lượng logic (tick/giây) |
|---|---|---|
| 100 | 0.0035 ms | ~284,000 |
| 500 | 0.0092 ms | ~109,000 |
| 1000 | 0.0183 ms | ~54,500 |

Thông lượng giảm gần tuyến tính theo số xe (đúng như kỳ vọng vì mỗi tick lặp qua toàn bộ danh sách `vehicles`). Ở mức 1000 xe, phần lõi logic vẫn xử lý được **~54,500 tick/giây** — nói cách khác, so với khung hình 60 FPS mà SFML cần, phần `TrafficSimulator::update()` chỉ tốn khoảng **0.018 ms trong ngân sách 16.6 ms mỗi khung hình** (~0.1%). Kết luận: **nếu FPS thực tế của bản build SFML bị tụt ở 1000 xe, nguyên nhân gần như chắc chắn nằm ở phần vẽ (`VisualizationEngine` / `VehicleSprite` / SFML draw calls), không phải ở lõi mô phỏng `StatisticsManager`/`TrafficSimulator`.** Đây là thông tin quan trọng để báo lại cho TV3/TV4 nếu buổi stress test bằng SFML thật (sinh 50 xe ở bản Thứ 7) cho thấy tụt FPS.

## 7. Tóm tắt khuyến nghị cho báo cáo cuối kỳ

- **BFS**: nhanh nhất, đúng nếu mục tiêu là "ít giao lộ nhất", nhưng bỏ qua hoàn toàn tốc độ/kẹt xe → không phù hợp làm router mặc định cho xe thường.
- **Dijkstra**: chính xác về chi phí thời gian thật, có tính đến congestion, nhưng duyệt nhiều node nhất → chậm nhất trong 3 thuật toán.
- **A***: giữ được độ chính xác như Dijkstra (cùng `avgPathCost`) nhưng duyệt ít hơn 55–65% số node nhờ heuristic Euclidean/maxSpeed → lựa chọn cân bằng tốt nhất, đúng như lý do dự án đang dùng A* làm mặc định trong `main.cpp` và `TrafficSimulator`.
- Lõi mô phỏng (`StatisticsManager` + `TrafficSimulator`) không phải là nút thắt cổ chai ở quy mô 1000 xe — có thể yên tâm khi TV4 chạy stress test FPS với SFML thật.

## 8. Cách tái tạo kết quả

```bash
g++ -std=c++17 -O2 -Isrc tests/stress_test.cpp \
  src/model/Graph.cpp src/model/Intersection.cpp src/model/Road.cpp src/model/Vehicle.cpp \
  src/algorithm/BFSStrategy.cpp src/algorithm/DijkstraStrategy.cpp src/algorithm/AStarStrategy.cpp \
  src/simulation/StatisticsManager.cpp src/simulation/EventManager.cpp src/simulation/TrafficSimulator.cpp \
  -o stress_test
./stress_test
python3 make_charts.py   # cần pandas + matplotlib
```

File `tests/stress_test.cpp` không phụ thuộc SFML/imgui nên build cực nhanh và có thể chạy trên máy bất kỳ thành viên nào để double-check số liệu trước khi đưa vào báo cáo.
