//
// Created by Martin Wickham on 3/12/16.
//

#include "svg.h"

static inline int writeGridLine(FILE *f, int x, int y, int x2, int y2, int usage, int cap) {
    int overflow = 0;
    int r, g, b, width;
    if (usage == 0) {
        return 0;
    } else if (usage <= cap) {
        // non-overflow case... grey. darker == more usage
        r = 255 - 225 * usage / (cap + 1);
        g = r;
        b = r;
        width = 7 * usage / cap;
    } else {
        // overflow... red.
        r = 255;
        g = 0;
        b = 0;
        width = 6;
        overflow = usage - cap;
    }
    fprintf(f,
            "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:rgb(%d,%d,%d);stroke-width:%d\" />\n",
            10*x + 5, 10*y + 5, 10*x2 + 5, 10*y2 + 5, r, g, b, width);
    return overflow;
}

int writeCongestionSvg(const RoutingInst &inst, const char *filename) {
    int overflow = 0;
    FILE* f = fopen(filename, "w");
    fprintf(f, "<!DOCTYPE html><html><body>\n");
    fprintf(f, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" style='width:%ipx;height:%ipx'>\n", inst.gx * 10, inst.gy * 10);

    fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\" />\n",          0,          0,          0, 10*inst.gy);
    fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\" />\n",          0, 10*inst.gy, 10*inst.gx, 10*inst.gy);
    fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\" />\n", 10*inst.gx,          0, 10*inst.gx, 10*inst.gy);
    fprintf(f, "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" style=\"stroke:black;stroke-width:1\" />\n",          0,          0, 10*inst.gx,          0);

    for (int y = 0; y < inst.gy; y++) {
        for (int x = 0; x < inst.gx; x++) {
            const Cell &cell = inst.cell(x, y);
            if (x < inst.gx - 1) {
                overflow += writeGridLine(f, x, y, x+1, y, cell.right.utilization, inst.cap);
            }
            if (y < inst.gy - 1) {
                overflow += writeGridLine(f, x, y, x, y+1, cell.down.utilization, inst.cap);
            }
        }
    }

    fprintf(f, "</svg></body></html>");
    fclose(f);
    return overflow;
}
