/*
 * =============================================================
 * GCode Trajectory Calculator -- Trapezoidal Velocity Profile
 * =============================================================
 *
 * LUONG XU LY:
 *   file.gcode
 *      |
 *      v  parse_line()       
 *   GCommand[]
 *      |
 *      v  interpolate_*()    <- tinh quy dao theo thoi gian
 *   trajectory.csv           <- t_ms, x, y, v (moi 1ms 1 dong)
 *      |
 *      v  visualize.py       <- ve do thi
 *   trajectory_plot.png
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

/* =============================================================
   HANG SO
   ============================================================= */

/* Gia toc toi da cua may (mm/s^2).
   Giu gia tri nho -> may tang/giam toc cham, an toan cho co khi.
   Giu gia tri lon -> may phan ung nhanh hon nhung rung nhieu hon.
   500 mm/s^2 la gia tri trung binh cho may CNC hobby. */
#define MAX_ACCELERATION  500.0f

/* Toc do G0 (rapid move) tinh bang mm/min.
   3000 mm/min = 50 mm/s. */
#define RAPID_FEEDRATE   3000.0f

/* Buoc thoi gian noi suy: 1ms.
   Moi 1ms tinh 1 diem (x, y, v) va ghi vao CSV.
   Tren phan cung thuc te, day la tan so cua Interpolator ISR:
   bo dinh thi (timer) ngat moi 1ms va goi ham tinh vi tri moi. */
#define DT_MS               1.0f

#define MODE_ABSOLUTE        90
#define MODE_INCREMENTAL     91

/* ---- LOOK-AHEAD (Chen et al. 2013, eq 34) ----
   Van toc toi da tai diem noi giua 2 doan:
     v_junction <= a_max * T / (2 * sin(alpha/2))
   alpha = goc giua 2 vector tiep tuyen tai diem noi.

   Y nghia: khi qua goc re alpha voi toc do v, vector van toc doi huong
   dot ngot, do bien thien |dv| = 2*v*sin(alpha/2). Bien thien nay phai
   thuc hien duoc voi gia toc <= a_max trong khoang thoi gian T.

   Dat T = Ts = 1ms dung nhu paper: toan bo cu doi huong phai xong
   trong 1 tick -> bao thu, goc sac tuyet doi nhung qua goc gat rat
   cham (90 do ~ 0.35 mm/s). Muon nhanh hon co the noi T len,
   doi lai goc bi bo tron. */
#define JUNCTION_T        0.001f  /* 1ms = Ts, giong paper */

#define MAX_SEGMENTS       100

/* =============================================================
   STRUCT DATA
   ============================================================= */

/* Lenh G-code tuyen tinh (G0, G1): chi can dich den X, Y va toc do F. */
struct LinearCmd {
    float x, y, z;
    float f;
    int has_x, has_y;   /* 1 neu dong G-code co gia tri X / Y */
    int has_f;
};

/* Lenh G-code cung tron (G2, G3): them cac tham so I, J (offset tam),
   hoac R (ban kinh). */
struct ArcCmd {
    float x, y, z;
    float f;
    float i, j;   /* offset tu diem dau den tam cung: cx = x0+I, cy = y0+J */
    float r;      /* ban kinh (thay the I,J neu nguoi dung dung cach ngan) */
    int has_x, has_y;
    int has_i, has_j;
    int has_f, has_r;
};

/* Mot lenh G-code day du.
   Dung union de tiet kiem bo nho: moi lenh chi la linear HOAC arc,
   khong bao gio ca hai cung luc. */
struct GCommand {
    char code[4];      /* "G0", "G1", "G2", "G3", hoac "" neu khong co */
    int  mode_change;  /* 0 = khong doi, 90 = G90 absolute, 91 = G91 incremental */
    union {
        struct LinearCmd linear;
        struct ArcCmd    arc;
    } data;
};

/* =============================================================
   TRAPEZOID PROFILE
   =============================================================
   
   Profil hinh thang chia doan duong di thanh 3 giai doan:
   
       v (mm/s)
       |         ___________
   v_peak|        /           \
       |       /             \
   v_entry|____/               \____v_exit
       |
       +-----|-----------|-----|---> t (s)
         t_accel   t_cruise  t_decel
   
   Cong thuc tinh v_peak (khi doan ngan, khong dat duoc feedrate):
     v_peak = sqrt(a * s + (v_entry^2 + v_exit^2) / 2)
   
   Giai thich: dung phuong trinh dong nang (energy equation).
   Nang luong tich luy khi tang toc tu v_entry len v_peak:
     E_accel = (v_peak^2 - v_entry^2) / (2*a)   [= quang duong tang toc]
   Nang luong giai phong khi giam toc tu v_peak xuong v_exit:
     E_decel = (v_peak^2 - v_exit^2) / (2*a)
   Tong = do dai doan s:
     s = E_accel + E_decel
     => giai ra v_peak theo s, v_entry, v_exit.
*/
struct TrapezoidProfile {
    float v_entry;      /* van toc vao dau doan  (mm/s) */
    float v_peak;       /* van toc dinh           (mm/s) */
    float v_exit;       /* van toc ra cuoi doan   (mm/s) */
    float t_accel;      /* thoi gian tang toc     (s)    */
    float t_cruise;     /* thoi gian di deu       (s)    */
    float t_decel;      /* thoi gian giam toc     (s)    */
    float total_time;   /* tong thoi gian cua doan (s)   */
    float s_accel;      /* quang duong tang toc   (mm)   */
    float s_cruise;     /* quang duong di deu     (mm)   */
};

/*
 * compute_trapezoid: tinh day du thong so profil cho mot doan.
 *
 * Vi du: doan 50mm, F3000 mm/min, v_entry=0, v_exit=0
 *   v_max = 3000/60 = 50 mm/s
 *   v_peak_achievable = sqrt(500*50 + 0) = sqrt(25000) = 158 mm/s > 50 -> dung v_max
 *   v_peak = 50 mm/s
 *   t_accel = (50-0)/500 = 0.1 s
 *   s_accel = 50^2/(2*500) = 2.5 mm
 *   s_cruise = 50 - 2.5 - 2.5 = 45 mm
 *   t_cruise = 45/50 = 0.9 s
 *   total = 0.1 + 0.9 + 0.1 = 1.1 s = 1100 ms
 */
struct TrapezoidProfile compute_trapezoid(float segment_length,
                                          float feedrate,
                                          float v_entry,
                                          float v_exit) {
    struct TrapezoidProfile prof;
    float v_max = feedrate / 60.0f;  /* mm/min -> mm/s */
    float a     = MAX_ACCELERATION;
    float s_decel;

    /* dam bao khong vuot gioi han feedrate */
    if (v_entry > v_max) v_entry = v_max;
    if (v_exit  > v_max) v_exit  = v_max;

    /* tinh v_peak theo cong thuc energy equation.
       Neu doan du dai -> v_peak_achievable > v_max -> dung v_max (profil day du 3 pha).
       Neu doan ngan    -> v_peak_achievable < v_max -> dung v_peak_achievable (profil tam giac). */
    float v_peak_achievable = sqrtf(a * segment_length
                                    + (v_entry*v_entry + v_exit*v_exit) / 2.0f);
    float v_peak = (v_peak_achievable < v_max) ? v_peak_achievable : v_max;

    prof.v_entry  = v_entry;
    prof.v_peak   = v_peak;
    prof.v_exit   = v_exit;

    /* thoi gian tang toc: v = v_entry + a*t  =>  t = (v_peak - v_entry) / a */
    prof.t_accel  = (v_peak - v_entry) / a;
    prof.t_decel  = (v_peak - v_exit)  / a;

    /* quang duong tang toc: s = (v_peak^2 - v_entry^2) / (2*a)
       (phuong trinh dong hoc: v^2 = v0^2 + 2*a*s) */
    prof.s_accel  = (v_peak*v_peak - v_entry*v_entry) / (2.0f * a);
    s_decel       = (v_peak*v_peak - v_exit*v_exit)   / (2.0f * a);

    prof.s_cruise = segment_length - prof.s_accel - s_decel;
    if (prof.s_cruise < 0.0f) prof.s_cruise = 0.0f;

    /* t_cruise = s_cruise / v_peak  (chuyen dong deu: s = v*t) */
    prof.t_cruise    = (prof.s_cruise > 0.0f) ? prof.s_cruise / v_peak : 0.0f;
    prof.total_time  = prof.t_accel + prof.t_cruise + prof.t_decel;

    return prof;
}

/*
 * trapezoid_state: tra ve vi tri s (mm) va van toc v (mm/s)
 * tai thoi diem t_sec (giay) trong profil.
 *
 * Dung cong thuc dong hoc (analytic, khong tich phan so):
 *   Pha 1 tang toc:  v = v_entry + a*t
 *                    s = v_entry*t + 0.5*a*t^2
 *   Pha 2 di deu:    v = v_peak
 *                    s = s_accel + v_peak*(t - t_accel)
 *   Pha 3 giam toc:  v = v_peak - a*t_d      (t_d = thoi gian ke tu luc bat dau giam)
 *                    s = s_accel + s_cruise + v_peak*t_d - 0.5*a*t_d^2
 *
 * Tai sao dung analytic thay vi numerical integration?
 * Neu cong don dv moi 1ms: loi lam tron tich luy theo thoi gian.
 * Analytic: tinh truc tiep tu t -> khong co loi tich luy.
 */
void trapezoid_state(float t_sec,
                     const struct TrapezoidProfile *prof,
                     float *out_s,
                     float *out_v) {
    float a = MAX_ACCELERATION;

    if (t_sec <= prof->t_accel) {
        /* giai doan 1: tang toc */
        *out_v = prof->v_entry + a * t_sec;
        *out_s = prof->v_entry * t_sec + 0.5f * a * t_sec * t_sec;

    } else if (t_sec <= prof->t_accel + prof->t_cruise) {
        /* giai doan 2: di deu */
        float t_in_cruise = t_sec - prof->t_accel;
        *out_v = prof->v_peak;
        *out_s = prof->s_accel + prof->v_peak * t_in_cruise;

    } else {
        /* giai doan 3: giam toc */
        float t_in_decel = t_sec - prof->t_accel - prof->t_cruise;
        *out_v = prof->v_peak - a * t_in_decel;
        if (*out_v < prof->v_exit) *out_v = prof->v_exit; /* khong xuong duoi v_exit */
        *out_s = prof->s_accel + prof->s_cruise
                 + prof->v_peak * t_in_decel
                 - 0.5f * a * t_in_decel * t_in_decel;
    }
}

/* =============================================================
   print_command: debug helper
   ============================================================= */
void print_command(struct GCommand *cmd) {
    int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');
    printf("[%s]", cmd->code);
    if (is_arc) {
        if (cmd->data.arc.has_x) printf(" X=%.2f", cmd->data.arc.x);
        if (cmd->data.arc.has_y) printf(" Y=%.2f", cmd->data.arc.y);
        if (cmd->data.arc.has_i) printf(" I=%.2f", cmd->data.arc.i);
        if (cmd->data.arc.has_j) printf(" J=%.2f", cmd->data.arc.j);
        if (cmd->data.arc.has_r) printf(" R=%.2f", cmd->data.arc.r);
        if (cmd->data.arc.has_f) printf(" F=%.2f", cmd->data.arc.f);
    } else {
        if (cmd->data.linear.has_x) printf(" X=%.2f", cmd->data.linear.x);
        if (cmd->data.linear.has_y) printf(" Y=%.2f", cmd->data.linear.y);
        if (cmd->data.linear.has_f) printf(" F=%.2f", cmd->data.linear.f);
    }
    printf("\n");
}

/* =============================================================
   parse_line: doc 1 dong G-code, tra ve GCommand
   =============================================================
   
   G-code co nhieu dac diem can xu ly:
   
   1. N number: so dong tuy chon o dau moi dong (vi du "N40 G01 X10")
      -> bo qua, khong anh huong lenh.
   
   2. Comment trong ngoac don: "G01 X10 (day la comment)"
      -> loai bo truoc khi parse.
   
   3. Modal G-code: neu dong khong co G0-G3, dung lenh cuoi cung con hieu luc.
      Vi du: "N70 Y10" -> khong co G, dung modal_code tu dong truoc (G1).
      Day la dac diem quan trong cua G-code: mot khi dat G1, no "nho" den
      khi bi thay the boi lenh G khac.
   
   4. Nhieu G tren 1 dong: "G90 G01 X10" -> xu ly tat ca.
   
   Tham so:
     line       - chuoi ki tu cua 1 dong G-code
     modal_code - G code dang hieu luc tu dong truoc (G0/G1/G2/G3)
*/
struct GCommand parse_line(char *line, const char *modal_code) {
    struct GCommand cmd;
    char clean[256];
    char *read_cursor;   /* vi tri dang doc trong chuoi input goc  */
    char *write_cursor;  /* vi tri dang ghi trong buffer clean     */
    char *scan_pos;      /* vi tri dang quet tim ki tu cu the      */
    int depth, g_num, is_arc;

    memset(&cmd, 0, sizeof(cmd));

    /* --- Buoc 1: Bo qua N number --- */
    read_cursor = line;
    while (*read_cursor == ' ' || *read_cursor == '\t') read_cursor++;
    if (*read_cursor == 'N') {
        read_cursor++;
        while (*read_cursor >= '0' && *read_cursor <= '9') read_cursor++;
        while (*read_cursor == ' '  || *read_cursor == '\t') read_cursor++;
    }

    /* --- Buoc 2: Xoa comment (...) bang cach copy qua buffer moi ---
       depth dem so ngoac mo chua dong.
       Ki tu ben trong ngoac (depth>0) khong duoc copy sang clean. */
    write_cursor = clean;
    depth = 0;
    while (*read_cursor) {
        if (*read_cursor == '(') { depth++; read_cursor++; continue; }
        if (*read_cursor == ')') { depth--; read_cursor++; continue; }
        if (depth == 0) *write_cursor++ = *read_cursor;
        read_cursor++;
    }
    *write_cursor = '\0';

    /* --- Buoc 3: Tim tat ca G code trong dong ---
       strchr tim ki tu 'G' tiep theo. sscanf doc so nguyen sau G.
       G90/G91: cap nhat mode_change.
       G0-G3:   cap nhat cmd.code. */
    scan_pos = clean;
    while ((scan_pos = strchr(scan_pos, 'G')) != NULL) {
        sscanf(scan_pos, "G%d", &g_num);
        if      (g_num == 90) cmd.mode_change = 90;
        else if (g_num == 91) cmd.mode_change = 91;
        else if (g_num >= 0 && g_num <= 3)
            snprintf(cmd.code, 4, "G%d", g_num);
        scan_pos++;
    }

    /* --- Buoc 4: Ap dung modal G-code neu can ---
       Neu dong nay khong co G0-G3 tuong minh nhung co X/Y/F,
       su dung lenh cuoi cung con hieu luc (modal_code). */
    if (cmd.code[0] == '\0' && modal_code && modal_code[0] != '\0')
        strncpy(cmd.code, modal_code, 4);

    if (cmd.code[0] == '\0') return cmd;

    /* --- Buoc 5: Parse cac tham so X, Y, I, J, R, F ---
       strchr tim vi tri cua ki tu, sau do sscanf doc so thap phan. */
    is_arc = (cmd.code[1] == '2' || cmd.code[1] == '3');
    if (is_arc) {
        scan_pos = strchr(clean, 'X');
        if (scan_pos) { sscanf(scan_pos, "X%f", &cmd.data.arc.x); cmd.data.arc.has_x = 1; }
        scan_pos = strchr(clean, 'Y');
        if (scan_pos) { sscanf(scan_pos, "Y%f", &cmd.data.arc.y); cmd.data.arc.has_y = 1; }
        scan_pos = strchr(clean, 'I');
        if (scan_pos) { sscanf(scan_pos, "I%f", &cmd.data.arc.i); cmd.data.arc.has_i = 1; }
        scan_pos = strchr(clean, 'J');
        if (scan_pos) { sscanf(scan_pos, "J%f", &cmd.data.arc.j); cmd.data.arc.has_j = 1; }
        scan_pos = strchr(clean, 'R');
        if (scan_pos) { sscanf(scan_pos, "R%f", &cmd.data.arc.r); cmd.data.arc.has_r = 1; }
        scan_pos = strchr(clean, 'F');
        if (scan_pos) { sscanf(scan_pos, "F%f", &cmd.data.arc.f); cmd.data.arc.has_f = 1; }
    } else {
        scan_pos = strchr(clean, 'X');
        if (scan_pos) { sscanf(scan_pos, "X%f", &cmd.data.linear.x); cmd.data.linear.has_x = 1; }
        scan_pos = strchr(clean, 'Y');
        if (scan_pos) { sscanf(scan_pos, "Y%f", &cmd.data.linear.y); cmd.data.linear.has_y = 1; }
        scan_pos = strchr(clean, 'F');
        if (scan_pos) { sscanf(scan_pos, "F%f", &cmd.data.linear.f); cmd.data.linear.has_f = 1; }
    }

    return cmd;
}

/* =============================================================
   read_gcode_file: doc toan bo file, tra ve mang GCommand
   ============================================================= */
int read_gcode_file(const char *filepath,
                    struct GCommand *commands,
                    int max_cmd) {
    FILE *file = fopen(filepath, "r");
    if (!file) { printf("Error: Cannot open %s\n", filepath); return 0; }

    char line[256];
    int  cmd_count = 0;
    char modal_code[4] = "";  /* luu G code cuoi cung con hieu luc */

    while (fgets(line, 256, file)) {
        if (line[0] == ';') continue;   /* dong comment (;...) */
        if (line[0] == '\n') continue;  /* dong trong */

        struct GCommand cmd = parse_line(line, modal_code);

        /* cap nhat modal_code de dong tiep theo ke thua */
        if (cmd.code[0] != '\0')
            strncpy(modal_code, cmd.code, 4);

        /* bo qua dong khong co gi */
        if (cmd.code[0] == '\0' && cmd.mode_change == 0) continue;

        commands[cmd_count++] = cmd;
        if (cmd_count >= max_cmd) break;
    }

    fclose(file);
    return cmd_count;
}

/* =============================================================
   interpolate_linear: noi suy duong thang voi trapezoid velocity
   =============================================================
   
   Thuat toan:
   1. Tinh do dai doan: L = sqrt(dx^2 + dy^2)
   2. Tinh profil hinh thang cho doan do (compute_trapezoid)
   3. Vong lap moi 1ms:
        a. Tinh s(t) va v(t) tu profil (trapezoid_state)
        b. Tinh alpha = s / L  (vi tri tren doan, 0..1)
        c. x = x0 + alpha*dx,  y = y0 + alpha*dy
        d. Ghi vao CSV: t_ms, x, y, v
   4. Ghi diem cuoi chinh xac (x1, y1)
   
   Tra ve: thoi gian thuc hien (ms) de main() cap nhat t_ms tong.
   
   Tham so v_entry, v_exit hien tai = 0 (moi doan dung lai hoan toan).
   Khi co look-ahead: truyen vao van toc goc tai junction.
*/
float interpolate_linear(float x0, float y0,
                         float x1, float y1,
                         float feedrate,
                         float v_entry, float v_exit,
                         float t_start_ms,
                         FILE *csv) {
    float dx             = x1 - x0;
    float dy             = y1 - y0;
    float segment_length = sqrtf(dx*dx + dy*dy);
    if (segment_length < 0.001f) return 0.0f;  /* doan qua ngan, bo qua */

    struct TrapezoidProfile prof = compute_trapezoid(segment_length, feedrate,
                                                     v_entry, v_exit);
    float total_ms = prof.total_time * 1000.0f;

    float t_ms;
    for (t_ms = 0.0f; t_ms < total_ms; t_ms += DT_MS) {
        float s, v;
        trapezoid_state(t_ms / 1000.0f, &prof, &s, &v);

        /* alpha = phan tram do dai da di duoc (0 = dau, 1 = cuoi) */
        float alpha = s / segment_length;
        if (alpha > 1.0f) alpha = 1.0f;

        /* noi suy tuyen tinh: P = P0 + alpha*(P1-P0) */
        float x = x0 + alpha * dx;
        float y = y0 + alpha * dy;

        if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n",
                         t_start_ms + t_ms, x, y, v);
    }

    /* dam bao diem cuoi chinh xac (tranh loi lam tron float) */
    if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n",
                     t_start_ms + total_ms, x1, y1, prof.v_exit);

    printf("  v_entry=%.1f  v_peak=%.1f  v_exit=%.1f mm/s  |  %.0f ms\n",
           prof.v_entry, prof.v_peak, prof.v_exit, total_ms);

    return total_ms;
}

/* =============================================================
   interpolate_arc: noi suy cung tron voi trapezoid velocity
   =============================================================
   
   Cung tron duoc xac dinh boi:
     - Diem dau (x0, y0), diem cuoi (x1, y1)
     - Tam: cx = x0 + I,  cy = y0 + J   (neu dung I,J)
     - Hoac ban kinh R (neu dung R: tinh cx, cy tu R)
   
   Tham so R: co 2 cung tron noi (x0,y0) va (x1,y1) voi ban kinh R.
   Chon cung nao tuy thuoc dau cua R va chieu quay G2/G3:
     R > 0: cung nho hon 180 do
     R < 0: cung lon hon 180 do
   
   Thuat toan noi suy cung:
   1. Tinh cx, cy (neu dung R)
   2. Tinh angle_start, angle_end (atan2)
   3. Tinh sweep angle (bao nhieu radian quay duoc)
   4. arc_length = |sweep| * R
   5. Tinh trapezoid tren arc_length
   6. Moi 1ms: tinh s(t) -> alpha -> angle = angle_start + alpha*sweep
              -> x = cx + R*cos(angle),  y = cy + R*sin(angle)
*/
/* prototype: dinh nghia o phan LOOK-AHEAD phia duoi,
   nhung interpolate_arc cung can dung */
void arc_sweep(float x0, float y0, float x1, float y1,
               float cx, float cy, int clockwise,
               float *out_angle_start, float *out_sweep);

/*
 * arc_center_from_r: tinh offset tam (i_off, j_off) tu ban kinh R.
 *
 * Co 2 cung tron noi (x0,y0) va (x1,y1) voi ban kinh R.
 * Chon cung nao tuy thuoc dau cua R va chieu quay G2/G3:
 *   R > 0: cung nho hon 180 do
 *   R < 0: cung lon hon 180 do
 *
 * Phuong phap: tam nam tren duong trung truc cua day cung,
 * cach trung diem mot khoang h = sqrt(R^2 - (d/2)^2).
 *
 * Tra ve 1 neu hop le, 0 neu hinh hoc sai (d qua lon so voi R).
 */
int arc_center_from_r(float x0, float y0,
                      float x1, float y1,
                      float r, int clockwise,
                      float *out_i, float *out_j) {
    float dx = x1 - x0;
    float dy = y1 - y0;
    float d  = sqrtf(dx*dx + dy*dy);  /* khoang cach hai diem */
    if (d < 0.001f || d > 2.0f * fabsf(r) + 0.01f) {
        printf("  [WARN] arc geometry invalid: d=%.2f R=%.2f\n", d, fabsf(r));
        return 0;
    }
    float h_sq = r*r - (d/2.0f)*(d/2.0f);
    float h    = (h_sq > 0.0f) ? sqrtf(h_sq) : 0.0f;
    float mx   = (x0 + x1) / 2.0f;
    float my   = (y0 + y1) / 2.0f;
    /* vector vuong goc voi day cung (chieu quay 90 do) */
    float px   = -dy / d;
    float py   =  dx / d;
    /* sign xac dinh tam o phia nao cua day cung (R>0 hay R<0) */
    float sign = (r > 0) ? 1.0f : -1.0f;
    float center_x, center_y;
    if (clockwise) {
        center_x = mx + sign * h * px;
        center_y = my + sign * h * py;
    } else {
        center_x = mx - sign * h * px;
        center_y = my - sign * h * py;
    }
    *out_i = center_x - x0;
    *out_j = center_y - y0;
    return 1;
}

float interpolate_arc(float x0, float y0,
                      float x1, float y1,
                      float i_off, float j_off,
                      float r,
                      int clockwise,
                      float feedrate,
                      float v_entry, float v_exit,
                      float t_start_ms,
                      FILE *csv) {

    /* --- Tinh cx, cy tu R neu can --- */
    if (r != 0.0f) {
        if (!arc_center_from_r(x0, y0, x1, y1, r, clockwise, &i_off, &j_off))
            return 0.0f;
    }

    float center_x   = x0 + i_off;
    float center_y   = y0 + j_off;
    float arc_radius  = sqrtf(i_off*i_off + j_off*j_off);
    if (arc_radius < 0.001f) return 0.0f;

    /* goc bat dau va goc quet (dau cua sweep theo chieu quay G2/G3) */
    float angle_start, sweep;
    arc_sweep(x0, y0, x1, y1, center_x, center_y, clockwise,
              &angle_start, &sweep);

    /* do dai cung = ban kinh * goc quay (radian) */
    float arc_length = fabsf(sweep) * arc_radius;
    struct TrapezoidProfile prof = compute_trapezoid(arc_length, feedrate,
                                                     v_entry, v_exit);
    float total_ms = prof.total_time * 1000.0f;

    float t_ms;
    for (t_ms = 0.0f; t_ms < total_ms; t_ms += DT_MS) {
        float s, v;
        trapezoid_state(t_ms / 1000.0f, &prof, &s, &v);

        float alpha = s / arc_length;
        if (alpha > 1.0f) alpha = 1.0f;
        float angle = angle_start + alpha * sweep;

        /* diem tren cung: P = tam + R*(cos, sin) */
        float x = center_x + arc_radius * cosf(angle);
        float y = center_y + arc_radius * sinf(angle);

        if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n",
                         t_start_ms + t_ms, x, y, v);
    }

    if (csv) fprintf(csv, "%.1f,%.4f,%.4f,%.3f\n",
                     t_start_ms + total_ms, x1, y1, prof.v_exit);

    printf("  v_entry=%.1f  v_peak=%.1f  v_exit=%.1f mm/s  |  %.0f ms\n",
           prof.v_entry, prof.v_peak, prof.v_exit, total_ms);

    return total_ms;
}

/* =============================================================
   LOOK-AHEAD (Chen et al. 2013)
   =============================================================

   Truoc khi noi suy, doc truoc TOAN BO cac doan de biet hinh dang
   duong chay -> tinh van toc tai moi diem noi (junction) -> moi doan
   khong can dung ve 0 nua.

   3 buoc:
     1. Tinh v_junction tai moi diem noi tu goc chuyen huong (eq 34)
     2. Backward pass: tu cuoi ve dau, dam bao doan i GIAM kip
        tu v[i] xuong v[i+1] trong chieu dai L[i]
     3. Forward pass: tu dau ve cuoi, dam bao doan i TANG kip
        tu v[i] len v[i+1] trong chieu dai L[i]

   Sau 2 pass, moi cap (v[i], v[i+1]) deu thoa man
     |v[i+1]^2 - v[i]^2| <= 2*a*L[i]
   nen compute_trapezoid() luon dung duoc profil hop le.
*/

/* Mot doan chuyen dong da resolve day du hinh hoc.
   Duoc xay truoc khi noi suy de look-ahead co the "nhin" toan bo. */
struct Segment {
    int   is_arc;
    int   clockwise;             /* chi co nghia khi is_arc = 1 */
    float x0, y0, x1, y1;
    float i_off, j_off;          /* offset tam (da resolve tu R neu can) */
    float feedrate;              /* mm/min */
    float length;                /* mm */
    float tan_in_x,  tan_in_y;   /* vector tiep tuyen don vi tai diem DAU */
    float tan_out_x, tan_out_y;  /* vector tiep tuyen don vi tai diem CUOI */
    float v_entry, v_exit;       /* ket qua look-ahead (mm/s) */
};

/*
 * arc_sweep: tinh goc bat dau va goc quet cua cung tron.
 * Tach rieng vi ca build_segments() va interpolate_arc() deu can.
 */
void arc_sweep(float x0, float y0, float x1, float y1,
               float cx, float cy, int clockwise,
               float *out_angle_start, float *out_sweep) {
    float angle_start = atan2f(y0 - cy, x0 - cx);
    float angle_end   = atan2f(y1 - cy, x1 - cx);
    float sweep = angle_end - angle_start;
    if (clockwise) {
        if (sweep > 0) sweep -= 2.0f * 3.14159265f;  /* phai am */
    } else {
        if (sweep < 0) sweep += 2.0f * 3.14159265f;  /* phai duong */
    }
    *out_angle_start = angle_start;
    *out_sweep       = sweep;
}

/*
 * junction_velocity: van toc toi da khi qua diem noi (eq 34).
 *
 * Input: 2 vector tiep tuyen don vi (ra khoi doan truoc, vao doan sau).
 * Goc alpha giua chung tinh bang dot product (eq 36):
 *   cos(alpha) = t0 . t1
 *
 * Goc cang gat -> sin(alpha/2) cang lon -> phai di cang cham.
 * Duong gan thang (alpha ~ 0) -> khong gioi han (tra ve so rat lon,
 * se bi cap boi feedrate o buoc sau).
 */
float junction_velocity(float t0x, float t0y, float t1x, float t1y) {
    float dot = t0x*t1x + t0y*t1y;
    if (dot >  1.0f) dot =  1.0f;   /* chong loi lam tron float */
    if (dot < -1.0f) dot = -1.0f;
    float alpha = acosf(dot);
    if (alpha < 0.001f) return 1.0e9f;  /* gan thang: khong can giam toc */
    return MAX_ACCELERATION * JUNCTION_T / (2.0f * sinf(alpha * 0.5f));
}

/*
 * compute_lookahead: dien v_entry / v_exit cho moi doan.
 *
 * vj[i] = van toc tai diem noi giua doan i-1 va doan i.
 * vj[0] = 0 (bat dau tu dung yen), vj[n] = 0 (ket thuc dung yen).
 */
void compute_lookahead(struct Segment *segs, int n) {
    float vj[MAX_SEGMENTS + 1];
    float a = MAX_ACCELERATION;
    int i;

    /* --- Buoc 1: junction velocity tu goc chuyen huong --- */
    vj[0] = 0.0f;
    vj[n] = 0.0f;
    for (i = 1; i < n; i++) {
        float v = junction_velocity(segs[i-1].tan_out_x, segs[i-1].tan_out_y,
                                    segs[i].tan_in_x,    segs[i].tan_in_y);
        /* khong vuot feedrate cua ca 2 doan lien ke (eq 35: v = min(vt, vm)) */
        float vmax_prev = segs[i-1].feedrate / 60.0f;
        float vmax_next = segs[i].feedrate   / 60.0f;
        if (v > vmax_prev) v = vmax_prev;
        if (v > vmax_next) v = vmax_next;
        vj[i] = v;
    }

    /* --- Buoc 2: backward pass ---
       Di tu cuoi ve dau. Doan i phai GIAM duoc tu vj[i] xuong vj[i+1]
       trong chieu dai L[i]. Toc do vao lon nhat cho phep:
         v^2 = vj[i+1]^2 + 2*a*L[i]   (phuong trinh dong hoc)
       Neu vj[i] lon hon -> ha xuong. */
    for (i = n - 1; i >= 0; i--) {
        float v_reachable = sqrtf(vj[i+1]*vj[i+1] + 2.0f * a * segs[i].length);
        if (vj[i] > v_reachable) vj[i] = v_reachable;
    }

    /* --- Buoc 3: forward pass ---
       Di tu dau ve cuoi. Doan i phai TANG duoc tu vj[i] len vj[i+1].
       Toc do ra lon nhat dat duoc:
         v^2 = vj[i]^2 + 2*a*L[i] */
    for (i = 0; i < n; i++) {
        float v_reachable = sqrtf(vj[i]*vj[i] + 2.0f * a * segs[i].length);
        if (vj[i+1] > v_reachable) vj[i+1] = v_reachable;
    }

    /* --- Ghi ket qua vao tung doan --- */
    for (i = 0; i < n; i++) {
        segs[i].v_entry = vj[i];
        segs[i].v_exit  = vj[i+1];
    }
}

/*
 * build_segments: chay qua mang GCommand, resolve toan bo hinh hoc
 * (toa do dich, tam arc, chieu dai, tiep tuyen) thanh mang Segment.
 *
 * Logic modal (G90/G91, feedrate, vi tri hien tai) giong het vong lap
 * noi suy cu trong main() — chi khac la KHONG noi suy, chi thu thap.
 *
 * Tiep tuyen:
 *   Line: (dx, dy) / L  — giong nhau o dau va cuoi doan.
 *   Arc:  vuong goc voi ban kinh tai diem do.
 *         Vi tri tren cung: P(theta) = tam + R*(cos, sin)
 *         CCW (G3): tiep tuyen = (-sin(theta),  cos(theta))
 *         CW  (G2): tiep tuyen = ( sin(theta), -cos(theta))
 */
int build_segments(struct GCommand *commands, int cmd_count,
                   struct Segment *segs) {
    float current_x  = 0.0f;
    float current_y  = 0.0f;
    float feedrate   = RAPID_FEEDRATE;
    int   coord_mode = MODE_ABSOLUTE;
    int   n = 0;
    int   i;

    for (i = 0; i < cmd_count; i++) {
        struct GCommand *cmd = &commands[i];
        int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');

        if (cmd->mode_change == 90) coord_mode = MODE_ABSOLUTE;
        else if (cmd->mode_change == 91) coord_mode = MODE_INCREMENTAL;

        if (cmd->code[0] == '\0') continue;

        if (is_arc) {
            if (cmd->data.arc.has_f)    feedrate = cmd->data.arc.f;
        } else {
            if (cmd->data.linear.has_f) feedrate = cmd->data.linear.f;
        }

        /* toa do dich (giong logic cu trong main) */
        float target_x, target_y;
        if (is_arc) {
            if (coord_mode == MODE_INCREMENTAL) {
                target_x = current_x + (cmd->data.arc.has_x ? cmd->data.arc.x : 0.0f);
                target_y = current_y + (cmd->data.arc.has_y ? cmd->data.arc.y : 0.0f);
            } else {
                target_x = cmd->data.arc.has_x ? cmd->data.arc.x : current_x;
                target_y = cmd->data.arc.has_y ? cmd->data.arc.y : current_y;
            }
        } else {
            if (coord_mode == MODE_INCREMENTAL) {
                target_x = current_x + (cmd->data.linear.has_x ? cmd->data.linear.x : 0.0f);
                target_y = current_y + (cmd->data.linear.has_y ? cmd->data.linear.y : 0.0f);
            } else {
                target_x = cmd->data.linear.has_x ? cmd->data.linear.x : current_x;
                target_y = cmd->data.linear.has_y ? cmd->data.linear.y : current_y;
            }
        }

        struct Segment *s = &segs[n];
        memset(s, 0, sizeof(*s));
        s->is_arc   = is_arc;
        s->x0       = current_x;
        s->y0       = current_y;
        s->x1       = target_x;
        s->y1       = target_y;
        s->feedrate = feedrate;

        if (is_arc) {
            s->clockwise = (cmd->code[1] == '2');
            float i_off = cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f;
            float j_off = cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f;
            if (cmd->data.arc.has_r) {
                if (!arc_center_from_r(current_x, current_y, target_x, target_y,
                                       cmd->data.arc.r, s->clockwise,
                                       &i_off, &j_off)) {
                    /* arc loi hinh hoc: bo qua nhu interpolate_arc cu */
                    current_x = target_x;
                    current_y = target_y;
                    continue;
                }
            }
            s->i_off = i_off;
            s->j_off = j_off;

            float cx = current_x + i_off;
            float cy = current_y + j_off;
            float radius = sqrtf(i_off*i_off + j_off*j_off);
            if (radius < 0.001f) {
                current_x = target_x;
                current_y = target_y;
                continue;
            }

            float angle_start, sweep;
            arc_sweep(current_x, current_y, target_x, target_y,
                      cx, cy, s->clockwise, &angle_start, &sweep);
            s->length = fabsf(sweep) * radius;

            float angle_end = angle_start + sweep;
            float dir = s->clockwise ? -1.0f : 1.0f;
            s->tan_in_x  = dir * -sinf(angle_start);
            s->tan_in_y  = dir *  cosf(angle_start);
            s->tan_out_x = dir * -sinf(angle_end);
            s->tan_out_y = dir *  cosf(angle_end);
        } else {
            float dx = target_x - current_x;
            float dy = target_y - current_y;
            float L  = sqrtf(dx*dx + dy*dy);
            if (L < 0.001f) {
                /* doan khong di chuyen (vd G90 G00 X0 Y0 tu goc): bo qua */
                current_x = target_x;
                current_y = target_y;
                continue;
            }
            s->length    = L;
            s->tan_in_x  = dx / L;
            s->tan_in_y  = dy / L;
            s->tan_out_x = s->tan_in_x;
            s->tan_out_y = s->tan_in_y;
        }

        n++;
        current_x = target_x;
        current_y = target_y;
        if (n >= MAX_SEGMENTS) break;
    }

    return n;
}

/* =============================================================
   MAIN
   ============================================================= */
int main(int argc, char *argv[]) {
    const char *filepath = (argc > 1) ? argv[1] : "test.gcode";

    printf("Reading file: %s\n\n", filepath);

    struct GCommand commands[100];
    int cmd_count = read_gcode_file(filepath, commands, 100);
    printf("Found %d commands:\n\n", cmd_count);

    /* mo file CSV de ghi quy dao.
       Format: t_ms,x,y,v
       Moi dong = 1 diem mau (cach nhau DT_MS = 1ms). */
    FILE *csv = fopen("trajectory.csv", "w");
    if (csv) fprintf(csv, "t_ms,x,y,v\n");

    /* --- Buoc 1: resolve toan bo hinh hoc thanh mang Segment --- */
    struct Segment segments[MAX_SEGMENTS];
    int seg_count = build_segments(commands, cmd_count, segments);
    printf("Built %d motion segments\n\n", seg_count);

    /* --- Buoc 2: look-ahead --- */
    compute_lookahead(segments, seg_count);

    printf("Look-ahead junction velocities (JUNCTION_T = %.0f ms):\n",
           JUNCTION_T * 1000.0f);
    int i;
    for (i = 0; i < seg_count; i++) {
        printf("  seg %2d  %s  L=%6.2f mm  v_entry=%5.1f  v_exit=%5.1f mm/s\n",
               i + 1,
               segments[i].is_arc ? (segments[i].clockwise ? "G2 " : "G3 ")
                                  : "lin",
               segments[i].length,
               segments[i].v_entry, segments[i].v_exit);
    }
    printf("\n");

    /* --- Buoc 3: noi suy tung doan voi v_entry/v_exit da tinh --- */
    float t_ms = 0.0f;   /* thoi gian tich luy (ms) */
    for (i = 0; i < seg_count; i++) {
        struct Segment *s = &segments[i];
        printf("--- Segment %d:\n", i + 1);

        float elapsed_ms;
        if (s->is_arc) {
            /* tam da resolve trong build_segments -> truyen i/j, r=0 */
            elapsed_ms = interpolate_arc(
                s->x0, s->y0, s->x1, s->y1,
                s->i_off, s->j_off, 0.0f,
                s->clockwise, s->feedrate,
                s->v_entry, s->v_exit, t_ms, csv);
        } else {
            elapsed_ms = interpolate_linear(
                s->x0, s->y0, s->x1, s->y1,
                s->feedrate,
                s->v_entry, s->v_exit, t_ms, csv);
        }

        t_ms += elapsed_ms;
    }

    printf("Total time: %.1f ms (%.2f sec)\n", t_ms, t_ms / 1000.0f);
    if (csv) {
        fclose(csv);
        printf("Saved: trajectory.csv  ->  python visualize.py trajectory.csv\n");
    }

    return 0;
}
