# CNC Lab — Dự án CNC 6 Trục

Mô phỏng điều khiển chuyển động CNC: parser G-code + S-curve velocity profile + look-ahead (C), visualize quỹ đạo (Python).

## Cấu trúc
Folder code chứa toàn bộ code chính vận hành
-config.h:thông số máy(TS=1ms, A=D=40 m/s², J=10 m/s³, F=2 m/min
-gcode.c/.h: Đọc Gcode  
-motion.c/.h:
+hình học (build_segments, bo góc);
+vận tốc góc nối eq (34) (junction_velocity)
+look-ahead 2 lượt quét ngược-xuôi (scurve_lookahead)
+chia block Type-7 + kẹp eq (33) tỉ lệ (divide_blocks)
+kế hoạch từng đoạn (plan_C_doan, scurve_plan_all)
-profile.c/.h
+nhịp nguyên eq (1)-(7) (scurve_steps)
+scurve_nc làm tròn nc, scurve_vm_fix sẽ bù error cho nc (còn có thể chỉnh Vstart hoặc Vend)
+scurve_zone_dist -- quang duong toi thieu de doi van toc v_lo -> v_hi
+scurve_reach -- nguoc lai: tu v0, trong quang duong L, S-curve len tới
   được bao nhiêu (khong > v_cap)
+scurve_peak-- dinh van toc cao nhat p de CA HAI cu doi van toc
   (v_start -> p roi p -> v_end) lot vua trong quang duong L
+phân loại Type eq (26)-(33) (scurve_type)
+phát nhịp eq (17)-(18) (scurve_v_tick)
-main_scurve:cứ tick 1ms: đọc → plan → mỗi tick một vận tốc → tọa độ → trajectory.csv
-test.gcode: lấy test bài báo 
-so_sanh_fig7b: so sánh đồ thị feedrate giữa mình và báo
## Thành viên
- Hoàng Trọng Trí
