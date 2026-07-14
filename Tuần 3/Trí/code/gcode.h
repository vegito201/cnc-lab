#ifndef GCODE_H
#define GCODE_H

/* =============================================================
   G-CODE PARSING
   ============================================================= */

/* Mot lenh G-code sau khi parse.
   Lenh thang (G0/G1) chi dung x,y,f; lenh cung (G2/G3) dung them i,j,r.
   ponytail: 1 struct phang thay cho union linear/arc nhu ban goc --
   vai float khong dung (i,j,r nam im khi lenh thang) re hon viec
   moi cho doc du lieu phai viet code 2 nhanh arc/linear. */
struct GCommand {
    char  code[4];      /* "G0".."G3", "" neu dong khong co lenh di chuyen */
    int   mode_change;  /* 0 = khong doi, 90 = G90 absolute, 91 = G91 incremental */
    float x, y;         /* toa do dich */
    float i, j;         /* offset tu diem dau den tam cung: tam = (x0+I, y0+J) */
    float r;            /* ban kinh cung (cach viet thay cho I,J) */
    float f;            /* feedrate (mm/min) */
    int   has_x, has_y, has_i, has_j, has_r, has_f;  /* 1 neu dong co tham so do */
};

struct GCommand parse_line(char *line, const char *modal_code);
int read_gcode_file(const char *filepath, struct GCommand *commands, int max_cmd);

#endif /* GCODE_H */
