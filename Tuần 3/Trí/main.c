/*
 * GCode Trajectory Calculator
 * Compile: gcc main.c -o main.exe -lm
 * Run:     ./main.exe test_g9091.gcode
 * Output:  trajectory.csv  (t_ms, x, y) -> dung visualize.py de ve hinh
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

/* ===================================================
   1. STRUCT + UNION
   =================================================== */

struct LinearCmd {
    float x, y, z;
    float f;
    int has_x, has_y;
    int has_f;
};

struct ArcCmd {
    float x, y, z;
    float f;
    float i, j;
    float r;
    int has_x, has_y;
    int has_i, has_j;
    int has_f, has_r;
};

struct GCommand {
    char code[4];
    int  mode_change;  /* 0=none, 90=G90, 91=G91 */
    union {
        struct LinearCmd linear;
        struct ArcCmd    arc;
    } data;
};

#define MODE_ABSOLUTE    90
#define MODE_INCREMENTAL 91

/* ===================================================
   2. print_command
   =================================================== */
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

/* ===================================================
   3. parse_line
   Input:  line       - mot dong G-code
           modal_code - G code cuoi cung con hieu luc (G0/G1/G2/G3).
                        Vi du: "N70 Y10" khong co G code tuong minh,
                        nen dung modal_code cua dong truoc (G1) -> G1 Y10.
                        Day goi la "modal G-code" trong chuan G-code CNC.
   Output: GCommand voi code, mode_change, va cac tham so.
           code[0] = '\0' neu khong co lenh di chuyen
   =================================================== */
struct GCommand parse_line(char *line, const char *modal_code) {
    struct GCommand cmd;
    char clean[256];
    char *src, *dst, *p;
    int depth, g_num, is_arc;

    memset(&cmd, 0, sizeof(cmd));

    /* Step 1: bo qua N number o dau dong */
    src = line;
    while (*src == ' ' || *src == '\t') src++;
    if (*src == 'N') {
        src++;
        while (*src >= '0' && *src <= '9') src++;
        while (*src == ' ' || *src == '\t') src++;
    }

    /* Step 2: xoa comment (...), copy phan con lai vao clean */
    dst = clean;
    depth = 0;
    while (*src) {
        if (*src == '(') { depth++; src++; continue; }
        if (*src == ')') { depth--; src++; continue; }
        if (depth == 0) *dst++ = *src;
        src++;
    }
    *dst = '\0';

    /* Step 3: tim tat ca G code trong dong */
    p = clean;
    while ((p = strchr(p, 'G')) != NULL) {
        sscanf(p, "G%d", &g_num);
        if      (g_num == 90) cmd.mode_change = 90;
        else if (g_num == 91) cmd.mode_change = 91;
        else if (g_num >= 0 && g_num <= 3)
            snprintf(cmd.code, 4, "G%d", g_num);
        p++;
    }

    /* Modal G-code: neu dong khong co G0-G3 tuong minh,
       ap dung modal_code cua dong truoc (neu co).
       Vi du: "N70 Y10" -> G1 (modal) Y=10 */
    if (cmd.code[0] == '\0' && modal_code && modal_code[0] != '\0') {
        strncpy(cmd.code, modal_code, 4);
    }

    if (cmd.code[0] == '\0') return cmd;

    /* Step 4: parse X, Y, I, J, R, F */
    is_arc = (cmd.code[1] == '2' || cmd.code[1] == '3');

    if (is_arc) {
        p = strchr(clean, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.arc.x); cmd.data.arc.has_x = 1; }
        p = strchr(clean, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.arc.y); cmd.data.arc.has_y = 1; }
        p = strchr(clean, 'I');
        if (p) { sscanf(p, "I%f", &cmd.data.arc.i); cmd.data.arc.has_i = 1; }
        p = strchr(clean, 'J');
        if (p) { sscanf(p, "J%f", &cmd.data.arc.j); cmd.data.arc.has_j = 1; }
        p = strchr(clean, 'R');
        if (p) { sscanf(p, "R%f", &cmd.data.arc.r); cmd.data.arc.has_r = 1; }
        p = strchr(clean, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.arc.f); cmd.data.arc.has_f = 1; }
    } else {
        p = strchr(clean, 'X');
        if (p) { sscanf(p, "X%f", &cmd.data.linear.x); cmd.data.linear.has_x = 1; }
        p = strchr(clean, 'Y');
        if (p) { sscanf(p, "Y%f", &cmd.data.linear.y); cmd.data.linear.has_y = 1; }
        p = strchr(clean, 'F');
        if (p) { sscanf(p, "F%f", &cmd.data.linear.f); cmd.data.linear.has_f = 1; }
    }

    return cmd;
}

/* ===================================================
   4. read_gcode_file
   =================================================== */
int read_gcode_file(const char *filepath,
                    struct GCommand *commands,
                    int max_cmd) {
    FILE *f = fopen(filepath, "r");
    if (!f) {
        printf("Error: Cannot open file %s\n", filepath);
        return 0;
    }

    char line[256];
    int  cmd_count = 0;
    char modal_code[4] = "";  /* track G code cuoi cung co chuyen dong */

    while (fgets(line, 256, f)) {
        if (line[0] == ';') continue;
        if (line[0] == '\n') continue;

        struct GCommand cmd = parse_line(line, modal_code);

        /* cap nhat modal_code neu dong nay co G code tuong minh */
        if (cmd.code[0] != '\0') {
            strncpy(modal_code, cmd.code, 4);
        }

        if (cmd.code[0] == '\0' && cmd.mode_change == 0) continue;

        commands[cmd_count] = cmd;
        cmd_count++;
        if (cmd_count >= max_cmd) break;
    }

    fclose(f);
    return cmd_count;
}

/* ===================================================
   5. interpolate_linear
   Input:  x0,y0      - diem bat dau (mm)
           x1,y1      - diem ket thuc (mm)
           feedrate   - toc do (mm/min)
           t_start_ms - thoi gian bat dau (ms)
           csv        - FILE* ghi trajectory.csv (NULL = bo qua)
   =================================================== */
void interpolate_linear(float x0, float y0,
                        float x1, float y1,
                        float feedrate,
                        float t_start_ms,
                        FILE *csv) {
    float dx       = x1 - x0;
    float dy       = y1 - y0;
    float dist     = sqrt(dx*dx + dy*dy);

    if (dist < 0.001f) return;

    float speed    = feedrate / 60.0f / 1000.0f;
    float total_ms = dist / speed;

    int t;
    for (t = 0; t <= (int)total_ms; t += 100) {
        float alpha = (float)t / total_ms;
        float x = x0 + alpha * dx;
        float y = y0 + alpha * dy;
        printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f\n", t_start_ms + t, x, y);
        if (csv) fprintf(csv, "%.1f,%.4f,%.4f\n", t_start_ms + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- end\n",
           t_start_ms + total_ms, x1, y1);
    if (csv) fprintf(csv, "%.1f,%.4f,%.4f\n", t_start_ms + total_ms, x1, y1);
}

/* ===================================================
   6. interpolate_arc
   Input:  x0,y0      - diem bat dau (mm)
           x1,y1      - diem ket thuc (mm)
           i_off      - offset X tu start den tam (mm); 0 neu dung R
           j_off      - offset Y tu start den tam (mm); 0 neu dung R
           r          - ban kinh (mm); 0 neu dung I,J
           clockwise  - 1=G2 (thuan chieu kim ho), 0=G3 (nguoc chieu)
           feedrate   - toc do (mm/min)
           t_start_ms - thoi gian bat dau (ms)
           csv        - FILE* ghi trajectory.csv (NULL = bo qua)
   =================================================== */
void interpolate_arc(float x0, float y0,
                     float x1, float y1,
                     float i_off, float j_off,
                     float r,
                     int clockwise,
                     float feedrate,
                     float t_start_ms,
                     FILE *csv) {

    /* neu dung R: tinh i_off, j_off tu R */
    if (r != 0.0f) {
        float dx = x1 - x0;
        float dy = y1 - y0;
        float d  = sqrt(dx*dx + dy*dy);
        if (d < 0.001f || d > 2.0f * fabs(r) + 0.01f) {
            printf("  [WARN] arc geometry invalid: d=%.2f, R=%.2f\n", d, fabs(r));
            return;
        }

        /* clamp de xu ly truong hop d xap xi 2R (ban nguyet) */
        float h_sq = r*r - (d/2.0f)*(d/2.0f);
        float h  = (h_sq > 0.0f) ? sqrt(h_sq) : 0.0f;
        float mx = (x0 + x1) / 2.0f;
        float my = (y0 + y1) / 2.0f;

        float px = -dy / d;
        float py =  dx / d;

        float sign = (r > 0) ? 1.0f : -1.0f;
        float center_x, center_y;
        if (clockwise) {
            center_x = mx + sign * h * px;
            center_y = my + sign * h * py;
        } else {
            center_x = mx - sign * h * px;
            center_y = my - sign * h * py;
        }

        i_off = center_x - x0;
        j_off = center_y - y0;
    }

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
        if (csv) fprintf(csv, "%.1f,%.4f,%.4f\n", t_start_ms + t, x, y);
    }

    printf("  t=%7.1f ms   X=%7.2f   Y=%7.2f  <- end\n",
           t_start_ms + total_ms, x1, y1);
    if (csv) fprintf(csv, "%.1f,%.4f,%.4f\n", t_start_ms + total_ms, x1, y1);
}

/* ===================================================
   7. MAIN
   =================================================== */
int main(int argc, char *argv[]) {
    const char *filepath = (argc > 1) ? argv[1] : "test.gcode";

    printf("Reading file: %s\n\n", filepath);

    struct GCommand commands[100];
    int cmd_count = read_gcode_file(filepath, commands, 100);
    printf("Found %d commands:\n\n", cmd_count);

    /* Mo file CSV de Python doc va ve */
    FILE *csv = fopen("trajectory.csv", "w");
    if (csv) fprintf(csv, "t_ms,x,y\n");

    float cx = 0.0f, cy = 0.0f;
    float t_ms     = 0.0f;
    float feedrate = 3000.0f;
    int   mode     = MODE_ABSOLUTE;

    int i;
    for (i = 0; i < cmd_count; i++) {
        struct GCommand *cmd = &commands[i];
        int is_arc = (cmd->code[1] == '2' || cmd->code[1] == '3');

        /* cap nhat mode */
        if (cmd->mode_change == 90) {
            mode = MODE_ABSOLUTE;
            printf("    [G90] -> absolute mode\n");
        } else if (cmd->mode_change == 91) {
            mode = MODE_INCREMENTAL;
            printf("    [G91] -> incremental mode\n");
        }

        if (cmd->code[0] == '\0') continue;

        /* cap nhat feedrate */
        if (is_arc) {
            if (cmd->data.arc.has_f) feedrate = cmd->data.arc.f;
        } else {
            if (cmd->data.linear.has_f) feedrate = cmd->data.linear.f;
        }

        /* tinh toa do dich */
        float target_x, target_y;
        if (is_arc) {
            if (mode == MODE_INCREMENTAL) {
                target_x = cx + (cmd->data.arc.has_x ? cmd->data.arc.x : 0.0f);
                target_y = cy + (cmd->data.arc.has_y ? cmd->data.arc.y : 0.0f);
            } else {
                target_x = cmd->data.arc.has_x ? cmd->data.arc.x : cx;
                target_y = cmd->data.arc.has_y ? cmd->data.arc.y : cy;
            }
        } else {
            if (mode == MODE_INCREMENTAL) {
                target_x = cx + (cmd->data.linear.has_x ? cmd->data.linear.x : 0.0f);
                target_y = cy + (cmd->data.linear.has_y ? cmd->data.linear.y : 0.0f);
            } else {
                target_x = cmd->data.linear.has_x ? cmd->data.linear.x : cx;
                target_y = cmd->data.linear.has_y ? cmd->data.linear.y : cy;
            }
        }

        printf("--- Command %d: ", i+1);
        print_command(cmd);

        /* noi suy quỹ đao */
        if (cmd->code[1] == '2') {
            interpolate_arc(cx, cy, target_x, target_y,
                            cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f,
                            cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f,
                            cmd->data.arc.has_r ? cmd->data.arc.r : 0.0f,
                            1, feedrate, t_ms, csv);
        } else if (cmd->code[1] == '3') {
            interpolate_arc(cx, cy, target_x, target_y,
                            cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f,
                            cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f,
                            cmd->data.arc.has_r ? cmd->data.arc.r : 0.0f,
                            0, feedrate, t_ms, csv);
        } else {
            interpolate_linear(cx, cy, target_x, target_y, feedrate, t_ms, csv);
        }

        /* cap nhat dong ho */
        float speed = feedrate / 60.0f / 1000.0f;
        if (is_arc) {
            float i_off, j_off;
            if (cmd->data.arc.has_r) {
                float r  = cmd->data.arc.r;
                float dx = target_x - cx;
                float dy = target_y - cy;
                float d  = sqrt(dx*dx + dy*dy);
                if (d < 0.001f || d > 2.0f * fabs(r) + 0.01f) { cx = target_x; cy = target_y; continue; }
                float h_sq2 = r*r - (d/2.0f)*(d/2.0f);
                float h  = (h_sq2 > 0.0f) ? sqrt(h_sq2) : 0.0f;
                float mx = (cx + target_x) / 2.0f;
                float my = (cy + target_y) / 2.0f;
                float px = -dy / d;
                float py =  dx / d;
                float sign = (r > 0) ? 1.0f : -1.0f;
                float center_x, center_y;
                if (cmd->code[1] == '2') {
                    center_x = mx + sign * h * px;
                    center_y = my + sign * h * py;
                } else {
                    center_x = mx - sign * h * px;
                    center_y = my - sign * h * py;
                }
                i_off = center_x - cx;
                j_off = center_y - cy;
            } else {
                i_off = cmd->data.arc.has_i ? cmd->data.arc.i : 0.0f;
                j_off = cmd->data.arc.has_j ? cmd->data.arc.j : 0.0f;
            }
            float R           = sqrt(i_off*i_off + j_off*j_off);
            float center_x    = cx + i_off;
            float center_y    = cy + j_off;
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

    printf("Total time: %.1f ms (%.2f sec)\n", t_ms, t_ms/1000.0f);

    if (csv) {
        fclose(csv);
   
        printf("Saved: trajectory.csv  -> chay 'python visualize.py' de ve hinh\n");
    }

    return 0;
}
