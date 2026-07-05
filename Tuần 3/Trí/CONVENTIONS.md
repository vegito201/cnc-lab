# Quy ước code — CNC Lab (rút từ main.c bản trapezoid, giữ cho mọi code mới)

> File này là "luật" khi viết/sửa code trong repo. AI (Claude Code/Cowork) và người
> đều phải theo. Rút ra từ việc so bản trapezoid (dễ đọc) với bản S-curve đầu tiên.

## 1. Comment

- Tiếng Việt KHÔNG dấu trong file .c (tránh lỗi encoding khi ghi file bằng tool).
- Mỗi hằng số tunable phải có comment dạng "knob": nghĩa là gì, chỉnh NHỎ thì sao,
  chỉnh LỚN thì sao. Ví dụ chuẩn: comment của MAX_ACCELERATION, JUNCTION_T.
- Block comment đầu mỗi section có:
  a) ASCII art nếu là khái niệm hình học / profile (hình thang, S-curve, bo góc)
  b) Công thức dẫn xuất TỪNG BƯỚC, không thả công thức cuối cùng trơ trọi
  c) VÍ DỤ SỐ THẬT (worked example): "doan 50mm, F3000, v_entry=0 -> v_peak=...,
     t_accel=0.1s, tong=1.1s". Đây là thứ làm bản trapezoid dễ hiểu — BẮT BUỘC
     cho mọi hàm toán mới.
- Comment giải thích TẠI SAO chọn cách này, không tả lại code ("tăng i lên 1" = rác).
- Doc comment đầu hàm: hàm làm gì, tham số, và đặc điểm cần lưu ý của bài toán.

## 2. Đặt tên

- Hằng số ALLCAPS, tên đầy đủ dễ đọc: MAX_ACCELERATION (không phải ACCEL_MM_S2).
- Biến/field mang nghĩa vật lý + đơn vị ghi trong comment: v_entry, v_peak, v_exit
  (mm/s); t_accel, t_cruise (s); angle_start, sweep (rad).
- KHÔNG dùng tên cụt kiểu v0/v1/A/T khi field đó sống lâu trong struct —
  chấp nhận được cho biến local vòng đời ngắn trong 1 công thức.
- Tên nói rõ vai trò: read_cursor/write_cursor thay cho src/dst/p.

## 3. Cấu trúc

- Section header: /* === TÊN === */, thứ tự file: constants → parsing → profile →
  geometry → look-ahead → interpolation → main.
- Struct phẳng, field tự giải thích. Nếu buộc lồng struct (Ramp trong SCurveProfile)
  thì mỗi field lồng phải có comment nói nó đóng vai gì trong tổng thể.
- Mẹo code gọn (đảo ngược thời gian, dùng lại hàm 2 chiều) PHẢI kèm comment
  "Meo:" giải thích + vẽ/viết ví dụ, vì gọn cho code là đắt cho người đọc.
- 1 hàm = 1 khái niệm. Hàm toán tách khỏi hàm I/O.

## 4. Toán

- Analytic (tính thẳng từ t) thay vì cộng dồn mỗi tick — tránh tích lũy lỗi float.
  Ghi lý do này ở nơi dùng.
- Khi thay công thức đóng bằng binary search / numeric: comment phải nêu
  (a) vì sao không có công thức đóng, (b) tính đơn điệu nào bảo đảm search đúng.

## 5. Tài liệu đi kèm code

- CONTEXT_WEEK<n>.md: trạng thái project để mở session AI mới là nắm được ngay.
- IMPLEMENTATION_VS_PAPER.md: lấy gì / bỏ gì / khác gì so với paper, kèm SỐ công thức.
- Cập nhật 2 file này TRONG CÙNG commit với code thay đổi, không để sau.

## 6. Quy trình

- File Python/file có tiếng Việt có dấu: ghi bằng bash heredoc (Write tool bị bug cắt file).
- Trước commit: gcc -Wall -Wextra sạch warning, chạy test_g9091.gcode,
  verify từ trajectory.csv: v ≤ vmax, |a| ≤ a_max, về đúng điểm cuối, không NaN.
