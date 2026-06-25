/*
 * GCode Trajectory Calculator — viết bằng C thuần
 * Compile:  gcc main.c -o main.exe -lm
 * Chay:     ./main.exe test.gcode
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════
   1. STRUCT + UNION

   Tại sao dùng union?
   - G0/G1 (đường thẳng): chỉ cần x, y, z, f
   - G2/G3 (cung tròn): cần thêm i, j (offset đến tâm)
   - Union cho 2 loại lệnh dùng chung 1 vùng nhớ,
     tránh lãng phí khi G0/G1 không dùng i, j.
   ═══════════════════════════════════════ */

/* Lệnh đường thẳng: G0, G1 */
struct LinearCmd {
    float x, y, z;        /* tọa độ đích (mm) */
    float f;              /* feedrate (mm/phút) */
    int   has_x, has_y;   /* cờ: có X/Y trong dòng G-code không */
    int   has_f;          /* cờ: có F trong dòng G-code không */
};

/* Lệnh cung tròn: G2 (CW), G3 (CCW) */
struct ArcCmd {
    float x, y, z;        /* tọa độ đích (mm) */
    float f;              /* feedrate (mm/phút) */
    float i, j;           /* offset từ điểm đầu đến tâm vòng tròn (mm) */
    int   has_x, has_y;
    int   has_i, has_j;
    int   has_f;
};

/* GCommand: lưu 1 lệnh G-code bất kỳ */
struct GCommand {
    char code[4];   /* tên lệnh: "G0", "G1", "G2", "G3" */
    union {
        struct LinearCmd linear;  /* dùng khi G0/G1 */
        struct ArcCmd    arc;     /* dùng khi G2/G3 */
    } data;
};

/* ═══════════════════════════════════════
   2. print_command
   Input:  cmd — con trỏ đến GCommand cần in
   Output: in ra terminal dạng [G1] X=... Y=... F=...
   ═══════════════════════════════════════ */
void print_command(struct GCommand *cmd) {
    int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');

    printf("[%s]", cmd->code);

    if (is_arc) {
        if (cmd->data.arc.has_x) printf(" X=%.2f", cmd->data.arc.x);
        if (cmd->data.arc.has_y) printf(" Y=%.2f", cmd->data.arc.y);
        if (cmd->data.arc.has_i) printf(" I=%.2f", cmd->data.arc.i);
        if (cmd->data.arc.has_j) printf(" J=%.2f", cmd->data.arc.j);
        if (cmd->data.arc.has_f) printf(" F=%.2f", cmd->data.arc.f);
    } else {
        if (cmd->data.linear.has_x) printf(" X=%.2f", cmd->data.linear.x);
        if (cmd->data.linear.has_y) printf(" Y=%.2f", cmd->data.linear.y);
        if (cmd->data.linear.has_f) printf(" F=%.2f", cmd->data.linear.f);
    }

    printf("\n");
}

/* ═══════════════════════════════════════
   3. parse_line
   Input:  line — 1 dòng text từ file G-code (vd: "G1 X20 Y0 F800")
   Output: GCommand chứa loại lệnh và các tham số đã parse
           Nếu dòng không hợp lệ → trả về GCommand với code[0] = '\0'
   ═══════════════════════════════════════ */
struct GCommand parse_line(char *line) {
    struct GCommand cmd;
    int is_arc;
    char *p;
    int g_num;

    memset(&cmd, 0, sizeof(cmd));

    /* Tìm G — dùng strchr() vì tìm 1 ký tự đơn */
    p = strchr(line, 'G');
    if (!p) return cmd;
    sscanf(p, "G%d", &g_num);
    if (g_num < 0 || g_num > 3) return cmd;
    snprintf(cmd.code, 4, "G%d", g_num);

    is_arc = (g_num == 2 || g_num == 3);

    if (is_arc) {
        p = strchr(line, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.arc.x); cmd.data.arc.has_x = 1; }

        p = strchr(line, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.arc.y); cmd.data.arc.has_y = 1; }

        p = strchr(line, 'I');
        if (p) { sscanf(p, "I%f", &cmd.data.arc.i); cmd.data.arc.has_i = 1; }

        p = strchr(line, 'J');
        if (p) { sscanf(p, "J%f", &cmd.data.arc.j); cmd.data.arc.has_j = 1; }

        p = strchr(line, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.arc.f); cmd.data.arc.has_f = 1; }
    } else {
        p = strchr(line, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.linear.x); cmd.data.linear.has_x = 1; }

        p = strchr(line, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.linear.y); cmd.data.linear.has_y = 1; }

        p = strchr(line, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.linear.f); cmd.data.linear.has_f = 1; }
    }

    return cmd;
}

/* ═══════════════════════════════════════
   4. read_gcode_file
   Input:  filepath  — đường dẫn file G-code
           commands  — mảng để lưu các lệnh đọc được
           max_cmd   — giới hạn số lệnh tối đa
   Output: số lệnh đọc được (0 nếu không mở được file)
   ═══════════════════════════════════════ */
int read_gcode_file(const char *filepath,
                    struct GCommand *commands,
                    int max_cmd) {
    FILE *f = fopen(filepath, "r");
    if (!f) {
        printf("Loi: Khong mo duoc file %s\n", filepath);
        return 0;
    }

    char line[256];
    int  cmd_count = 0;

    while (fgets(line, 256, f)) {
        if (line[0] == ';') continue;   /* bỏ qua dòng comment */
        if (line[0] == '\n') continue;  /* bỏ qua dòng trống */

        struct GCommand cmd = parse_line(line);
        if (cmd.code[0] == '\0') continue;

        commands[cmd_count] = cmd;
        cmd_count++;
        if (cmd_count >= max_cmd) break;
    }

    fclose(f);
    return cmd_count;
}

/* ═══════════════════════════════════════
   5. interpolate_linear
   Nội suy tuyến tính từ điểm đầu đến điểm đích, in tọa độ mỗi 100ms.
   Input:  x0, y0     — tọa độ điểm đầu (mm)
           x1, y1     — tọa độ điểm đích (mm)
           feedrate   — tốc độ di chuyển (mm/phút)
           t_start_ms — thời điểm bắt đầu lệnh (ms)
   Output: in ra terminal tọa độ X, Y tại từng mốc thời gian
   ═══════════════════════════════════════ */
void interpolate_linear(float x0, float y0,
                        float x1, float y1,
                        float feedrate,
                        float t_start_ms) {
    float dx       = x1 - x0;
    float dy       = y1 - y0;
    float dist     = sqrt(dx*dx + dy*dy);

    if (dist < 0.001f) return;

    float speed    = feedrate / 60.0f / 1000.0f;  /* mm/ms */
    float total_ms = dist / speed;

    int t;
    for (t = 0; t <= (int)total_ms; t += 100) {
        float alpha = (float)t / total_ms;
        float x = x0 + alpha * dx;
        float y = y0 + alpha * dy;
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n", t_start_ms + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_start_ms + total_ms, x1, y1);
}

/* ═══════════════════════════════════════
   6. interpolate_arc
   Nội suy cung tròn từ điểm đầu đến điểm đích, in tọa độ mỗi 100ms.
   Input:  x0, y0     — tọa độ điểm đầu (mm)
           x1, y1     — tọa độ điểm đích (mm)
           i_off      — offset X từ điểm đầu đến tâm vòng tròn (mm)
           j_off      — offset Y từ điểm đầu đến tâm vòng tròn (mm)
           clockwise  — hướng quay: 1=G2 chiều kim đồng hồ, 0=G3 ngược chiều
           feedrate   — tốc độ di chuyển (mm/phút)
           t_start_ms — thời điểm bắt đầu lệnh (ms)
   Output: in ra terminal tọa độ X, Y tại từng mốc thời gian
   ═══════════════════════════════════════ */
void interpolate_arc(float x0, float y0,
                     float x1, float y1,
                     float i_off, float j_off,
                     int clockwise,
                     float feedrate,
                     float t_start_ms) {
    float center_x = x0 + i_off;
    float center_y = y0 + j_off;

    float R = sqrt(i_off*i_off + j_off*j_off);
    if (R < 0.001f) return;

    float angle_start = atan2(y0 - center_y, x0 - center_x);
    float angle_end   = atan2(y1 - center_y, x1 - center_x);

    float sweep;
    if (clockwise) {
        sweep = angle_end - angle_start;
        if (sweep > 0) sweep -= 2.0f * 3.14159f;
    } else {
        sweep = angle_end - angle_start;
        if (sweep < 0) sweep += 2.0f * 3.14159f;
    }

    float arc_length = fabs(sweep) * R;
    float speed      = feedrate / 60.0f / 1000.0f;
    float total_ms   = arc_length / speed;

    int t;
    for (t = 0; t <= (int)total_ms; t += 100) {
        float alpha = (float)t / total_ms;
        float angle = angle_start + alpha * sweep;
        float x     = center_x + R * cos(angle);
        float y     = center_y + R * sin(angle);
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n", t_start_ms + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_start_ms + total_ms, x1, y1);
}

/* ═══════════════════════════════════════
   7. MAIN
   ═══════════════════════════════════════ */
int main(int argc, char *argv[]) {
    const char *filepath = (argc > 1) ? argv[1] : "test.gcode";

    printf("Doc file: %s\n\n", filepath);

    struct GCommand commands[100];
    int cmd_count = read_gcode_file(filepath, commands, 100);
    printf("Tim thay %d lenh:\n\n", cmd_count);

    float cx = 0.0f, cy = 0.0f;
    float t_ms   = 0.0f;
    float feedrate = 3000.0f;  /* tốc độ mặc định G0: 3000 mm/phút (quy ước máy) */

    int i;
    for (i = 0; i < cmd_count; i++) {
        struct GCommand *cmd = &commands[i];
        int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');

        /* cập nhật feedrate nếu lệnh có F */
        if (is_arc) {
            if (cmd->data.arc.has_f) feedrate = cmd->data.arc.f;
        } else {
            if (cmd->data.linear.has_f) feedrate = cmd->data.linear.f;
        }

        /* điểm đích — nếu không có X/Y thì giữ nguyên vị trí */
        float target_x, target_y;
        if (is_arc) {
            target_x = cmd->data.arc.has_x ? cmd->data.arc.x : cx;
            target_y = cmd->data.arc.has_y ? cmd->data.arc.y : cy;
        } else {
            target_x = cmd->data.linear.has_x ? cmd->data.linear.x : cx;
            target_y = cmd->data.linear.has_y ? cmd->data.linear.y : cy;
        }

        printf("--- Lenh %d: ", i+1);
        print_command(cmd);

        /* tính quỹ đạo */
        if (cmd->code[1] == '2') {
            interpolate_arc(cx, cy, target_x, target_y,
                            cmd->data.arc.i, cmd->data.arc.j,
                            1, feedrate, t_ms);
        } else if (cmd->code[1] == '3') {
            interpolate_arc(cx, cy, target_x, target_y,
                            cmd->data.arc.i, cmd->data.arc.j,
                            0, feedrate, t_ms);
        } else {
            interpolate_linear(cx, cy, target_x, target_y, feedrate, t_ms);
        }

        /* cập nhật đồng hồ */
        float speed = feedrate / 60.0f / 1000.0f;
        if (is_arc) {
            float i_off = cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f;
            float j_off = cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f;
            float R        = sqrt(i_off*i_off + j_off*j_off);
            float center_x = cx + i_off;
            float center_y = cy + j_off;
            float angle_start = atan2(cy - center_y, cx - center_x);
            float angle_end   = atan2(target_y - center_y, target_x - center_x);
            float sweep;
            if (cmd->code[1] == '2') {
                sweep = angle_end - angle_start;
                if (sweep > 0) sweep -= 2.0f * 3.14159f;
            } else {
                sweep = angle_end - angle_start;
                if (sweep < 0) sweep += 2.0f * 3.14159f;
            }
            t_ms += fabs(sweep) * R / speed;
        } else {
            float dist = sqrt((target_x-cx)*(target_x-cx) + (target_y-cy)*(target_y-cy));
            if (dist > 0.001f) t_ms += dist / speed;
        }

        cx = target_x;
        cy = target_y;

        printf("\n");
    }

    printf("Tong thoi gian: %.1f ms (%.2f giay)\n", t_ms, t_ms/1000.0f);

    return 0;
}
