# CNC Lab — Dự án CNC 6 Trục

Mô phỏng điều khiển chuyển động CNC: parser G-code + S-curve velocity profile + look-ahead (C), visualize quỹ đạo (Python).

## Cấu trúc

- `main.c` — parser G-code + S-curve velocity profile + look-ahead (Chen et al. 2013)
- `visualize.py` — vẽ quỹ đạo XY, position, velocity profile từ `trajectory.csv`
- `test.gcode` — file test
- `*.pdf`, `*.docx` — tài liệu tham khảo (paper Chen 2013, sách CNC Programming Tutorials, CNC Motion Control Theory)

## Build & chạy

```bash
gcc main.c -o main -lm
./main test.gcode                    # -> trajectory.csv
python visualize.py trajectory.csv   # -> trajectory_plot.png
```

## Thành viên

- Cao Tuấn Minh
- Hoàng Trọng Trí
