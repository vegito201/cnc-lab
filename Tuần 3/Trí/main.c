/*
 * GCode Trajectory Calculator — viết bằng C thuần
 * Compile:  gcc main.c -o main.exe -lm
 * Chay:     ./main.exe test.gcode
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ═══════════════════════════════════════
   1. STRUCT
   ═══════════════════════════════════════ */
struct GCommand {
    char  code[4];
    float x, y, z;
    float i, j;
    float f;
    int   has_x, has_y;
    int   has_i, has_j;
    int   has_f;
};

/* ═══════════════════════════════════════
   2. HÀM IN 1 LỆNH
   ═══════════════════════════════════════ */
void in_lenh(struct GCommand *cmd) {
    printf("[%s]", cmd->code);
    if (cmd->has_x) printf(" X=%.2f", cmd->x);
    if (cmd->has_y) printf(" Y=%.2f", cmd->y);
    if (cmd->has_i) printf(" I=%.2f", cmd->i);
    if (cmd->has_j) printf(" J=%.2f", cmd->j);
    if (cmd->has_f) printf(" F=%.2f", cmd->f);
    printf("\n");
}

/* ═══════════════════════════════════════
   3. HÀM ĐỌC 1 DÒNG GCODE
   ═══════════════════════════════════════ */
struct GCommand doc_mot_dong(char *dong) {
    struct GCommand cmd;
    /* khởi tạo tất cả = chưa có gì */
    cmd.has_x = cmd.has_y = 0;
    cmd.has_i = cmd.has_j = 0;
    cmd.has_f = 0;
    cmd.x = cmd.y = cmd.z = 0;
    cmd.i = cmd.j = cmd.f = 0;
    cmd.code[0] = '\0';

    char *p;
    int g_num;

    /* Tìm G */
    p = strstr(dong, "G");
    if (!p) return cmd;  /* không có G → bỏ qua */
    sscanf(p, "G%d", &g_num);
    if (g_num < 0 || g_num > 3) return cmd;  /* chỉ xử lý G0-G3 */
    snprintf(cmd.code, 4, "G%d", g_num);

    /* Tìm X, Y, I, J, F */
    p = strstr(dong, "X");
    if (p) { sscanf(p, "X%f", &cmd.x); cmd.has_x = 1; }

    p = strstr(dong, "Y");
    if (p) { sscanf(p, "Y%f", &cmd.y); cmd.has_y = 1; }

    p = strstr(dong, "I");
    if (p) { sscanf(p, "I%f", &cmd.i); cmd.has_i = 1; }

    p = strstr(dong, "J");
    if (p) { sscanf(p, "J%f", &cmd.j); cmd.has_j = 1; }

    p = strstr(dong, "F");
    if (p) { sscanf(p, "F%f", &cmd.f); cmd.has_f = 1; }

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
     * commands: mảng hộp để lưu kết quả
     * max_lenh: tối đa bao nhiêu lệnh
     */

    FILE *f = fopen(ten_file, "r");  /* mở file chế độ đọc "r" */
    if (!f) {
        printf("Loi: Khong mo duoc file %s\n", ten_file);
        return 0;
    }

    char dong[256];   /* buffer chứa 1 dòng, tối đa 256 ký tự */
    int  so_lenh = 0;

    /* fgets đọc từng dòng cho đến hết file */
    while (fgets(dong, 256, f)) {
        /* bỏ qua dòng comment (bắt đầu bằng ;) */
        if (dong[0] == ';') continue;
        /* bỏ qua dòng trống */
        if (dong[0] == '\n') continue;

        struct GCommand cmd = doc_mot_dong(dong);

        /* bỏ qua nếu không parse được G */
        if (cmd.code[0] == '\0') continue;

        /* lưu vào mảng */
        commands[so_lenh] = cmd;
        so_lenh++;

        /* tránh tràn mảng */
        if (so_lenh >= max_lenh) break;
    }

    fclose(f);  /* đóng file — quan trọng! */
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

    if (dist < 0.001f) return;  /* không cần đi */

    float toc_do  = feedrate / 60.0f / 1000.0f;  /* mm/ms */
    float tong_ms = dist / toc_do;

    int t;
    for (t = 0; t <= (int)tong_ms; t += 100) {  /* in mỗi 100ms */
        float alpha = (float)t / tong_ms;
        float x = x0 + alpha * dx;
        float y = y0 + alpha * dy;
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n",
               t_bat_dau + t, x, y);
    }

    /* in điểm cuối chính xác */
    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_bat_dau + tong_ms, x1, y1);
}

/* ═══════════════════════════════════════
   6. HÀM TÍNH QUỸ ĐẠO G2/G3 (cung tròn)
   ═══════════════════════════════════════ */
void tinh_g2g3(float x0, float y0,
               float x1, float y1,
               float i_off, float j_off,
               int clockwise,
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
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n",
               t_bat_dau + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- dich\n",
           t_bat_dau + tong_ms, x1, y1);
}

/* ═══════════════════════════════════════
   7. MAIN
   ═══════════════════════════════════════ */
int main(int argc, char *argv[]) {
    /*
     * argc = số tham số khi chạy
     * argv = mảng các tham số
     * argv[0] = tên chương trình
     * argv[1] = tên file GCode (nếu có)
     */

    const char *ten_file;
    if (argc > 1) {
        ten_file = argv[1];           /* dùng file truyền vào */
    } else {
        ten_file = "test.gcode";      /* mặc định */
    }

    printf("Doc file: %s\n\n", ten_file);

    /* Mảng chứa tối đa 100 lệnh */
    struct GCommand commands[100];
    int so_lenh = doc_file(ten_file, commands, 100);

    printf("Tim thay %d lenh:\n\n", so_lenh);

    /* Vị trí hiện tại và đồng hồ */
    float cx = 0.0f, cy = 0.0f;
    float t_ms = 0.0f;
    float feedrate = 3000.0f;  /* tốc độ mặc định G0 */

    int i;
    for (i = 0; i < so_lenh; i++) {
        struct GCommand *cmd = &commands[i];

        /* cập nhật feedrate nếu lệnh có F */
        if (cmd->has_f) feedrate = cmd->f;

        /* điểm đích — nếu không có X/Y thì giữ nguyên */
        float tx = cmd->has_x ? cmd->x : cx;
        float ty = cmd->has_y ? cmd->y : cy;

        printf("--- Lenh %d: ", i+1);
        in_lenh(cmd);

        /* tính quỹ đạo */
        if (cmd->code[1] == '2') {
            tinh_g2g3(cx, cy, tx, ty,
                      cmd->i, cmd->j,
                      1, feedrate, t_ms);
        } else if (cmd->code[1] == '3') {
            tinh_g2g3(cx, cy, tx, ty,
                      cmd->i, cmd->j,
                      0, feedrate, t_ms);
        } else {
            tinh_g1(cx, cy, tx, ty, feedrate, t_ms);
        }

        /* cập nhật đồng hồ */
        float toc_do = feedrate / 60.0f / 1000.0f;
        if (cmd->code[1] == '2' || cmd->code[1] == '3') {
            /* G2/G3: thời gian = độ dài cung / tốc độ */
            float i_off = cmd->has_i ? cmd->i : 0.0f;
            float j_off = cmd->has_j ? cmd->j : 0.0f;
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
            /* G0/G1: thời gian = quãng đường / tốc độ */
            float dist = sqrt((tx-cx)*(tx-cx) + (ty-cy)*(ty-cy));
            if (dist > 0.001f) t_ms += dist / toc_do;
        }

        /* cập nhật vị trí hiện tại */
        cx = tx;
        cy = ty;

        printf("\n");
    }

    printf("Tong thoi gian: %.1f ms (%.2f giay)\n", t_ms, t_ms/1000.0f);

    return 0;
}
