#include <stdio.h>
#include <string.h>
#include "gcode.h"


/*
 * parse_line: doc 1 dong G-code thanh GCommand.
 *
 * Cac dac diem cua G-code phai xu ly:
 *   1. N number: so thu tu dong, tuy chon ("N40 G01 X10") -> bo qua.
 *   2. Comment trong ngoac don: "G01 X10 (di den diem A)" -> xoa truoc khi parse.
 *   3. Modal: dong khong co G0-G3 thi dung lenh G cuoi cung con hieu luc.
 *      Vi du "N70 Y10" sau mot dong G1 -> hieu la G1 Y10.
 *   4. Nhieu G tren 1 dong: "G90 G01 X10" -> xu ly tat ca.
 *
 * Tham so:
 *   line       - chuoi 1 dong G-code
 *   modal_code - G code dang hieu luc tu dong truoc
 */
struct GCommand parse_line(char *line, const char *modal_code) {
    struct GCommand cmd;
    char clean[256];
    char *p, *w;
    int depth, g_num, k;

    memset(&cmd, 0, sizeof(cmd));

    /* buoc 1: bo N number dau dong */
    p = line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == 'N') {
        p++;
        while (*p >= '0' && *p <= '9') p++;
    }

    /* buoc 2: copy sang buffer 'clean', bo ki tu nam trong ngoac don.
       depth dem so ngoac mo chua dong (ho tro ngoac long nhau). */
    w = clean;
    depth = 0;
    for (; *p; p++) {
        if (*p == '(') { depth++; continue; }
        if (*p == ')') { depth--; continue; }
        if (depth == 0) *w++ = *p;
    }
    *w = '\0';

    /* buoc 3: tim tat ca G code tren dong.
       strchr nhay den tung chu 'G', sscanf doc so nguyen ngay sau no. */
    for (p = clean; (p = strchr(p, 'G')) != NULL; p++) {
        sscanf(p, "G%d", &g_num);
        if (g_num == 90 || g_num == 91) cmd.mode_change = g_num;
        else if (g_num >= 0 && g_num <= 3) snprintf(cmd.code, 4, "G%d", g_num);
    }

    /* buoc 4: dong khong co G0-G3 -> ke thua lenh modal cua dong truoc */
    if (cmd.code[0] == '\0' && modal_code && modal_code[0] != '\0')
        memcpy(cmd.code, modal_code, 4);
    if (cmd.code[0] == '\0') return cmd;

    /* buoc 5: doc cac tham so. Voi moi chu cai trong "XYIJRF":
       strchr tim vi tri chu cai do, sscanf doc so thap phan ngay sau.
       vals[k]/flags[k] tro den field tuong ung trong cmd -- them tham so
       moi (vd Z) chi can them 1 chu cai va 1 cap con tro. */
    {
        const char letters[]  = "XYIJRF";
        float *vals[]  = { &cmd.x, &cmd.y, &cmd.i, &cmd.j, &cmd.r, &cmd.f };
        int   *flags[] = { &cmd.has_x, &cmd.has_y, &cmd.has_i,
                           &cmd.has_j, &cmd.has_r, &cmd.has_f };
        for (k = 0; k < 6; k++) {
            p = strchr(clean, letters[k]);
            if (p && sscanf(p + 1, "%f", vals[k]) == 1) *flags[k] = 1;
        }
    }
    return cmd;
}

/* read_gcode_file: doc toan bo file, tra ve so lenh doc duoc.
   Bo qua dong comment (bat dau bang ';') va dong trong.
   modal_code luu lenh G cuoi cung de dong sau ke thua (dac tinh modal). */
int read_gcode_file(const char *filepath, struct GCommand *commands, int max_cmd) {
    FILE *file = fopen(filepath, "r");
    if (!file) { printf("Error: Cannot open %s\n", filepath); return 0; }

    char line[256];
    int  cmd_count = 0;
    char modal_code[4] = "";

    while (fgets(line, 256, file)) {
        if (line[0] == ';' || line[0] == '\n') continue;
        struct GCommand cmd = parse_line(line, modal_code);
        if (cmd.code[0] != '\0') memcpy(modal_code, cmd.code, 4);
        if (cmd.code[0] == '\0' && cmd.mode_change == 0) continue;  /* dong rong */
        commands[cmd_count++] = cmd;
        if (cmd_count >= max_cmd) break;
    }
    fclose(file);
    return cmd_count;
}

