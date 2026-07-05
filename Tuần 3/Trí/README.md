# CNC Lab — Dự án CNC 6 Trục

Mô phỏng điều khiển chuyển động CNC: parser G-code + S-curve velocity profile + look-ahead (C), visualize quỹ đạo (Python).

## Cấu trúc

- `main.c` — parser G-code + S-curve velocity profile + look-ahead (Chen et al. 2013)
- `visualize.py` — vẽ quỹ đạo XY, position, velocity profile từ `trajectory.csv`
- `test.gcode`, — file test
- `CONTEXT_WEEK3.md` — ghi chú context cho session làm việc
- `IMPLEMENTATION_VS_PAPER.md` — đối chiếu code với paper Chen 2013
- `*.pdf` — tài liệu tham khảo (paper Chen 2013, sách CNC Programming Tutorials)

## Build & chạy

```bash
gcc main.c -o main -lm
./main test.gcode              # -> trajectory.csv
python visualize.py trajectory.csv   # -> trajectory_plot.png
## Thành viên
- Hoàng Trọng Trí
