# CNC Project — Context cho session mới (Tuần 3)

## Tổng quan project
Dự án CNC 6 trục — viết phần mềm mô phỏng điều khiển bằng C + Python.
- **Folder chính:** `D:\02.personal\cnc-lab\Tuần 3\Trí\`
- **Repo:** `D:\02.personal\cnc-lab\` (git)

---

## Files quan trọng

### `main.c` — Core C program
- **Compile:** `gcc main.c -o main -lm`
- **Chạy:** `./main test_g9091.gcode`
- **Output:** `trajectory.csv` (format: `t_ms,x,y,v`)

**Cấu trúc:**
```
Constants:        MAX_ACCELERATION=500 mm/s², RAPID_FEEDRATE=3000 mm/min, DT_MS=1ms
Structs:          GCommand (union LinearCmd/ArcCmd), TrapezoidProfile
parse_line()      → đọc 1 dòng G-code: xử lý N number, comment, modal G-code, G90/G91
read_gcode_file() → đọc toàn bộ file → mảng GCommand[]
compute_trapezoid() → tính profil hình thang (v_entry, v_peak, v_exit, t_accel, t_cruise, t_decel)
trapezoid_state() → tại thời điểm t, trả về s(mm) và v(mm/s) bằng công thức analytic
interpolate_linear() → nội suy đường thẳng, ghi CSV mỗi 1ms, return total_ms
interpolate_arc()    → nội suy cung tròn (hỗ trợ I,J và R), ghi CSV, return total_ms
main()            → đọc file, loop qua commands, gọi interpolate_*(), ghi trajectory.csv
```

**Trạng thái hiện tại:**
- Đã implement Trapezoidal Velocity Profile hoàn chỉnh
- v_entry=0, v_exit=0 cho mọi đoạn (chưa có look-ahead)
- Comment chi tiết tiếng Việt (không dấu) trong toàn bộ file

### `visualize.py` — Python visualizer
- **Chạy:** `python visualize.py trajectory.csv`
- **Output:** `trajectory_plot.png` (3 subplots)
- **Subplots:** XY trajectory (plasma colormap theo thời gian), Position vs Time X(t)/Y(t), Velocity Profile v(t)
- Đọc CSV format `t_ms,x,y,v` — có fallback nếu thiếu cột v

### `test_g9091.gcode` — File test
```
N40 G90 G00 X0 Y0
N50 G01 X-10 Y-20 R8 (P1)
N60 G01 X-50 R10 (P2)
N70 Y10 (P3)           ← modal G1
N80 X-19.97 Y25.01 (P4)
N90 G03 X7.97 Y38.99 R18 (P5)
N100 G01 X30 Y50 (P6)
N110 G91 X10.1 Y-10.1(P7)
N120 G90 G02 X59.9 Y20.1 R14 (P8)
N130 G01 X70 Y10 (P9)
N140 Y-20 R10(P10)
N150 X50(P11)
N160 G03 X30 R10 (P12)
N170 G01 X10 R8(P13)
N180 X0 Y0
```

---

## Kiến thức nền đã có

### Velocity Profiles
- **Constant** (version cũ): chỉ di đều ở feedrate, không tăng/giảm tốc
- **Trapezoidal** (hiện tại): 3 pha: tăng tốc / di đều / giảm tốc
  - Công thức v_peak khi đoạn ngắn: `v_peak = sqrt(a*s + (v_entry² + v_exit²)/2)`
  - Dùng analytic formula (không numerical integration) để tránh lỗi tích lũy
- **S-curve** (tương lai): 7 pha, giới hạn jerk J (m/s³), mượt hơn
- **Look-ahead** (bước tiếp theo): tính junction velocity, 2-pass forward+backward

### Look-ahead — đã nghiên cứu (paper Chen et al. 2013)
Công thức junction velocity (eq 34):
```
v_junction ≤ a_max * Ts / (2 * sin(α/2))
```
Trong đó α = góc giữa 2 vector tiếp tuyến tại điểm nối:
```
cos(α) = τ_i · τ_{i+1} / (|τ_i| * |τ_{i+1}|)
```

**Thuật toán look-ahead đơn giản (sẽ implement):**
1. Tính v_junction tại mỗi điểm nối (công thức trên)
2. Backward pass: từ cuối về đầu, giới hạn v_exit[i] để đoạn i giảm kịp về v_junction[i+1]
   - `v_exit[i] = min(v_junction[i+1], sqrt(v_exit[i+1]² + 2*a*L[i]))`
3. Forward pass: từ đầu về cuối, giới hạn v_entry[i] để đoạn i tăng kịp từ v_junction[i]
   - `v_entry[i+1] = min(v_junction[i+1], sqrt(v_entry[i]² + 2*a*L[i]))`
4. Truyền v_entry[i], v_exit[i] vào interpolate_*() thay vì hardcode 0

**Kết quả mong đợi:** v(t) không còn về 0 tại mỗi junction, giảm tổng thời gian ~30-50%

### G-code features đã hỗ trợ
- G0 (rapid), G1 (linear), G2 (arc CW), G3 (arc CCW)
- G90 (absolute), G91 (incremental)
- N numbers, comments (...), modal G-code (không cần lặp G)
- R parameter cho arc (thay cho I,J)
- Nhiều G trên 1 dòng (G90 G01 X10)

---

## Design decisions quan trọng

| Quyết định | Lý do |
|---|---|
| C interpolates → CSV → Python visualize | Python không parse G-code lại (tránh duplicate logic) |
| Analytic trapezoid_state() | Không dùng numerical integration → tránh lỗi tích lũy float |
| interpolate_*() return total_ms | main() không tự tính thời gian, dùng giá trị thực từ profil |
| read_cursor/write_cursor/scan_pos | Đặt tên rõ thay vì src/dst/p |
| MAX_ACCELERATION không phải ACCEL_MM_S2 | Dễ đọc hơn |

---

## Convention code

- Comment tiếng Việt không dấu trong file C (tránh encoding issues khi dùng bash heredoc)
- Type hints trong Python
- Hằng số ALLCAPS
- Section headers: `/* === TÊN === */`
- File Python: dùng bash heredoc để write (tránh truncation bug khi dùng Write tool với tiếng Việt có dấu)

---

## Bước tiếp theo: Look-Ahead

Thêm vào `main()` một bước pre-process trước khi gọi interpolate_*():

```c
// Arrays cần thêm:
float v_entry[100];   // van toc vao cua moi doan
float v_exit[100];    // van toc ra cua moi doan
float junction_v[101]; // van toc tai moi junction (0..n)

// Buoc 1: tinh junction velocity tu goc chuyen huong
// Buoc 2: backward pass
// Buoc 3: forward pass
// Buoc 4: goi interpolate_*() voi v_entry/v_exit da tinh
```

Cần thêm helper:
- `segment_tangent()` — vector tiep tuyen cuoi doan (line: (dx,dy)/|d|, arc: tiep tuyen tai end point)
- `junction_velocity()` — tinh v_junction tu 2 tangent vectors va a_max

---

## Output hiện tại (verified working)
```
9865 points | Total time: 9843.1 ms (9.84 s)
X: [-50.00, 70.00] mm
Y: [-20.00, 50.00] mm
v_max = 50.00 mm/s (3000 mm/min)
```
