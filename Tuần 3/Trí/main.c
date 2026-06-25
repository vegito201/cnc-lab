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
   ═══════════════════════════════════════

   Tại sao dùng union?
   - G0/G1 (đường thẳng): chỉ cần x, y, z, f
   - G2/G3 (cung tròn): cần thêm i, j (offset đến tâm)
   - Union cho 2 loại lệnh dùng chung 1 vùng nhớ,
     tránh lãng phí khi G0/G1 không dùng i, j.
*/

/* Lệnh đường thẳng: G0, G1 */
struct LinearCmd {
    float x, y, z;        /* tọa độ đích */
    float f;              /* feedrate mm/phút */
    int   has_x, has_y;   /* cờ: có X/Y trong dòng G-code không */
    int   has_f;          /* cờ: có F trong dòng G-code không */
};

/* Lệnh cung tròn: G2 (CW), G3 (CCW) */
struct ArcCmd {
    float x, y, z;              /* tọa độ đích */
    float f;                    /* feedrate mm/phút */
    float i, j;                 /* offset từ điểm đầu đến tâm vòng tròn */
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
   2. HÀM IN 1 LỆNH
   ═══════════════════════════════════════ */
void in_lenh(struct GCommand *cmd) {
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
   3. HÀM ĐỌC 1 DÒNG GCODE
   ═══════════════════════════════════════ */
struct GCommand doc_mot_dong(char *dong) {
    struct GCommand cmd;
    int is_arc;
    char *p;
    int g_num;

    /* khởi tạo toàn bộ vùng data = 0 */
    memset(&cmd, 0, sizeof(cmd));

    /* Tìm G — dùng strchr() vì tìm 1 ký tự đơn */
    p = strchr(dong, 'G');
    if (!p) return cmd;
    sscanf(p, "G%d", &g_num);
    if (g_num < 0 || g_num > 3) return cmd;
    snprintf(cmd.code, 4, "G%d", g_num);

    is_arc = (g_num == 2 || g_num == 3);

    if (is_arc) {
        /* G2/G3: đọc vào data.arc */
        p = strchr(dong, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.arc.x); cmd.data.arc.has_x = 1; }

        p = strchr(dong, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.arc.y); cmd.data.arc.has_y = 1; }

        p = strchr(dong, 'I');
        if (p) { sscanf(p, "I%f", &cmd.data.arc.i); cmd.data.arc.has_i = 1; }

        p = strchr(dong, 'J');
        if (p) { sscanf(p, "J%f", &cmd.data.arc.j); cmd.data.arc.has_j = 1; }

        p = strchr(dong, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.arc.f); cmd.data.arc.has_f = 1; }
    } else {
        /* G0/G1: đọc vào data.linear */
        p = strchr(dong, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.linear.x); cmd.data.linear.has_x = 1; }

        p = strchr(dong, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.linear.y); cmd.data.linear.has_y = 1; }

        p = strchr(dong, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.linear.f); cmd.data.linear.has_f = 1; }
    }

    return cmd;
}

/* ═══════════════════════════════════════
   4. HÀM ĐỌC FILE GCODE
   ═══════════════════════════════════════ */
int doc_file(const char *ten_file,
             struct GCommand *commands,
             int max_lenh) {
    /*
     * Trả về: số lệnh đọc được
     * commands: mảng để lưu kết quả
     * max_lenh: tối đa bao nhiêu lệnh
     */

    FILE *f = fopen(ten_file, "r");
    if (!f) {
        printf("Loi: Khong mo duoc file %s\n", ten_file);
        return 0;
    }

    char dong[256];
    int  so_lenh = 0;

    while (fgets(dong, 256, f)) {
        if (dong[0] == ';') continue;   /* bỏ qua dòng comment */
        if (dong[0] == '\n') continue;  /* bỏ qua dòng trống */

        struct GCommand cmd = doc_mot_dong(dong);
        if (cmd.code[0] == '\0') continue;

        commands[so_lenh] = cmd;
        so_lenh++;
        if (so_lenh >= max_lenh) break;
    }

    fclose(f);
    return so_lenh;
}

/* ═══════════════════════════════════════
   5. HÀM TÍNH QUỸ ĐẠO G1 (đường thẳng)
   ═══════════════════════════════════════ */
void tinh_g1(float x0, float y0,   /* điểm đầu */
             float x1, float y1,   /* điểm đích */
             float feedrate,        /* tốc độ mm/phút */
             float t_bat_dau) {     /* đồng hồ bắt đầu (ms) */

    float dx   = x1 - x0;
    float dy   = y1 - y0;
    float dist = sqrt(dx*dx + dy*dy);

    if (dist < 0.001f) return;

    float toc_do  = feedrate / 60.0f / 1000.0f;  /* mm/ms */
    float tong_ms = dist / toc_do;

    int t;
    for (t = 0; t <= (int)tong_ms; t += 100) {
        float alpha = (float)t / tong_ms;
        float x = x0 + alpha * dx;
        float y = y0 + alpha * dy;
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n", t_bat_dau + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_bat_dau + tong_ms, x1, y1);
}

/* ═══════════════════════════════════════
   6. HÀM TÍNH QUỸ ĐẠO G2/G3 (cung tròn)
   ═══════════════════════════════════════ */
void tinh_g2g3(float x0, float y0,
               float x1, float y1,
               float i_off, float j_off,  /* offset đến tâm */
               int clockwise,             /* 1=G2 CW, 0=G3 CCW */
               float feedrate,
               float t_bat_dau) {

    float tam_x = x0 + i_off;
    float tam_y = y0 + j_off;

    float R = sqrt(i_off*i_off + j_off*j_off);
    if (R < 0.001f) return;

    float goc_dau  = atan2(y0 - tam_y, x0 - tam_x);
    float goc_cuoi = atan2(y1 - tam_y, x1 - tam_x);

    float quet;
    if (clockwise) {
        quet = goc_cuoi - goc_dau;
        if (quet > 0) quet -= 2.0f * 3.14159f;
    } else {
        quet = goc_cuoi - goc_dau;
        if (quet < 0) quet += 2.0f * 3.14159f;
    }

    float arc_length = fabs(quet) * R;
    float toc_do     = feedrate / 60.0f / 1000.0f;
    float tong_ms    = arc_length / toc_do;

    int t;
    for (t = 0; t <= (int)tong_ms; t += 100) {
        float alpha = (float)t / tong_ms;
        float goc   = goc_dau + alpha * quet;
        float x     = tam_x + R * cos(goc);
        float y     = tam_y + R * sin(goc);
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n", t_bat_dau + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_bat_dau + tong_ms, x1, y1);
}

/* ═══════════════════════════════════════
   7. MAIN
   ═══════════════════════════════════════ */
int main(int argc, char *argv[]) {
    const char *ten_file = (argc > 1) ? argv[1] : "test.gcode";

    printf("Doc file: %s\n\n", ten_file);

    struct GCommand commands[100];
    int so_lenh = doc_file(ten_file, commands, 100);
    printf("Tim thay %d lenh:\n\n", so_lenh);

    float cx = 0.0f, cy = 0.0f;
    float t_ms = 0.0f;
    float feedrate = 3000.0f;  /* tốc độ mặc định G0: 3000 mm/phút (quy ước máy) */

    int i;
    for (i = 0; i < so_lenh; i++) {
        struct GCommand *cmd = &commands[i];
        int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');

        /* cập nhật feedrate nếu lệnh có F */
        if (is_arc) {
            if (cmd->data.arc.has_f) feedrate = cmd->data.arc.f;
        } else {
            if (cmd->data.linear.has_f) feedrate = cmd->data.linear.f;
        }

        /* điểm đích — nếu không có X/Y thì giữ nguyên vị trí */
        float tx, ty;
        if (is_arc) {
            tx = cmd->data.arc.has_x ? cmd->data.arc.x : cx;
            ty = cmd->data.arc.has_y ? cmd->data.arc.y : cy;
        } else {
            tx = cmd->data.linear.has_x ? cmd->data.linear.x : cx;
            ty = cmd->data.linear.has_y ? cmd->data.linear.y : cy;
        }

        printf("--- Lenh %d: ", i+1);
        in_lenh(cmd);

        /* tính quỹ đạo */
        if (cmd->code[1] == '2') {
            tinh_g2g3(cx, cy, tx, ty,
                      cmd->data.arc.i, cmd->data.arc.j,
                      1, feedrate, t_ms);
        } else if (cmd->code[1] == '3') {
            tinh_g2g3(cx, cy, tx, ty,
                      cmd->data.arc.i, cmd->data.arc.j,
                      0, feedrate, t_ms);
        } else {
            tinh_g1(cx, cy, tx, ty, feedrate, t_ms);
        }

        /* cập nhật đồng hồ */
        float toc_do = feedrate / 60.0f / 1000.0f;
        if (is_arc) {
            float i_off = cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f;
            float j_off = cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f;
            float R     = sqrt(i_off*i_off + j_off*j_off);
            float tam_x = cx + i_off;
            float tam_y = cy + j_off;
            float goc_dau  = atan2(cy - tam_y, cx - tam_x);
            float goc_cuoi = atan2(ty - tam_y, tx - tam_x);
            float quet;
            if (cmd->code[1] == '2') {
                quet = goc_cuoi - goc_dau;
                if (quet > 0) quet -= 2.0f * 3.14159f;
            } else {
                quet = goc_cuoi - goc_dau;
                if (quet < 0) quet += 2.0f * 3.14159f;
            }
            t_ms += fabs(quet) * R / toc_do;
        } else {
            float dist = sqrt((tx-cx)*(tx-cx) + (ty-cy)*(ty-cy));
            if (dist > 0.001f) t_ms += dist / toc_do;
        }

        cx = tx;
        cy = ty;

        printf("\n");
    }

    printf("Tong thoi gian: %.1f ms (%.2f giay)\n", t_ms, t_ms/1000.0f);

    return 0;
}
