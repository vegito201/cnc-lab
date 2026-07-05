# Ghi chú: Implement Look-Ahead vs bài báo Chen et al. 2013

> Paper: "Look-Ahead Algorithm with Whole S-Curve Acceleration and Deceleration",
> Advances in Mechanical Engineering, 2013.
> Code: `main.c` (commit 8e8d5d9, look-ahead trên trapezoid profile).

## 1. Những gì ĐÃ lấy từ paper

| Paper | Nội dung | Trong code |
|---|---|---|
| eq (34) | v_junction ≤ a_max·T / (2·sin(α/2)) | `junction_velocity()` — dùng nguyên công thức |
| eq (35) | v = min(v_t, v_m) | `compute_lookahead()` bước 1 — cap vj[i] bởi feedrate |
| eq (36) | cos α = τ_i·τ_{i+1} / (\|τ_i\|·\|τ_{i+1}\|) | dot product trong `junction_velocity()` |
| Section 3 (tinh thần) | Quét xuôi/ngược đảm bảo tốc độ junction khả thi | backward pass + forward pass trong `compute_lookahead()` |
| Hình 3 | 4 kiểu điểm rẽ (line/arc × line/arc) | `build_segments()` lưu tangent in/out cho từng đoạn → 1 phép dot chung |

## 2. Những gì KHÔNG implement (và vì sao)

| Paper | Nội dung | Lý do bỏ qua |
|---|---|---|
| 2.1, 2.1.1 | S-curve 7 pha, tính n_a, n_b, n_c, k (tick nguyên) | Giữ trapezoid 3 pha sẵn có (`compute_trapezoid`). S-curve là bước nâng cấp sau |
| 2.1.2–2.1.4 | Chỉnh v_m′/v_start′/v_end′ + jerk J′ để khử sai số làm tròn tick | Mô phỏng chạy thời gian liên tục (float), không bị ràng buộc tick nguyên; cuối đoạn ghi thẳng điểm đích vào CSV |
| 2.2 | 7 loại profile (Type 1–7) theo chiều dài L | Công thức trapezoid `v_peak = min(v_max, sqrt(a·L + (v_entry²+v_exit²)/2))` tự bao hết các case |
| Section 3 | Break path (Type 7), chia block, longest perfect path, Algorithm A/B/C | Chỉ cần khi profile là S-curve tick nguyên. Với trapezoid, điều kiện khả thi thu gọn về v² = v₀² + 2aL |

## 3. Khác biệt có chủ đích so với paper

1. **Profile: trapezoid thay vì S-curve.** Gia tốc nhảy bậc (jerk vô hạn) — chấp nhận được
   cho mô phỏng, máy thật sẽ rung. Nâng cấp sau: viết `compute_scurve()` + `scurve_state()`
   thay 2 hàm trapezoid, lúc đó BẮT BUỘC cần 2.1.1, 2.2, 2.1.2–2.1.4 của paper.

2. **T trong eq 34: JUNCTION_T = 20ms thay vì Ts = 1ms.**
   - Paper dùng Ts (toàn bộ cú đổi hướng gói trong 1 tick) — bảo thủ tuyệt đối.
     Với a=500, Ts=1ms: góc 90° chỉ được 0.35 mm/s (coi như vẫn dừng).
   - Code nới thành JUNCTION_T=20ms → góc 90° qua ở 7.1 mm/s.
   - KHÔNG vi phạm gia tốc: từ eq 34 suy ra Δv/T ≤ a_max với mọi T.
     Cái giá là sai lệch hình học tại góc (dao lượn cung ~v·T thay vì gãy khúc sắc).
   - JUNCTION_T là nút vặn: thời gian gia công ↔ độ sắc góc. Đo được:
     T=1ms → 9822ms; T=20ms → 9462ms; T=50ms → 9022ms; T=100ms → 8633ms.

3. **eq 35: min trực tiếp với feedrate CẢ 2 đoạn liền kề.** Paper viết min với một v_m
   (2 phía được đảm bảo gián tiếp qua Algorithm A/B ở Section 3). Code min tại chỗ — cùng kết quả.

4. **eq 36: bỏ phép chia độ lớn.** Tangent đã chuẩn hóa thành vector đơn vị trong
   `build_segments()` nên mẫu số luôn = 1.

5. **Điều kiện khả thi trong 2 pass: v² = v₀² + 2aL (trapezoid) thay vì phân loại Type.**
   - Backward: v_exit[i] khả thi ⇔ vj[i] ≤ sqrt(vj[i+1]² + 2aL[i])
   - Forward:  v_entry khả thi ⇔ vj[i+1] ≤ sqrt(vj[i]² + 2aL[i])
   - Sau 2 pass: |v[i+1]² − v[i]²| ≤ 2aL[i] với mọi i → compute_trapezoid luôn hợp lệ.

## 4. Tham số giả định (KHÔNG có trong đề bài — tự chọn)

| Tham số | Giá trị | Nguồn gốc |
|---|---|---|
| MAX_ACCELERATION | 500 mm/s² | Giả định "máy CNC hobby trung bình" (từ tuần trước). Máy thật: lấy từ datasheet động cơ + khối lượng + chạy thử |
| JUNCTION_T | 20 ms | Chọn tay theo kinh nghiệm (a·T = 10 mm/s là dải có ích so với v_max=50). GRBL/Marlin cũng để tham số tương đương cho người dùng tune |
| RAPID_FEEDRATE | 3000 mm/min | Giả định |
| DT_MS | 1 ms | Chu kỳ nội suy giả định (khớp Ts của paper) |

Hướng làm "sạch đề bài" hơn: thay JUNCTION_T bằng mô hình junction deviation của GRBL —
spec là dung sai góc δ (mm, đo được): v² = a_max·δ·sin(α/2)/(1−sin(α/2)).

## 5. Kết quả đo (test_g9091.gcode, JUNCTION_T=20ms)

- Trước look-ahead: 9843.1 ms. Sau: 9461.9 ms (−4%).
- Gain nhỏ vì file test toàn góc gắt 60–90°; trần lý thuyết của đường này ~8.5s.
- Đường nhiều đoạn ngắn gần thẳng hàng (CAD/CAM xấp xỉ curve) sẽ gain lớn hơn nhiều.
- Verify: v ≤ 50 mm/s, về đúng (0,0), bước max 0.05mm/sample, không NaN,
  v(t) chỉ chạm 0 ở đầu/cuối chương trình.
