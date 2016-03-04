// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <string>
#include <sstream>
#include <assert.h>

#include "ece556.h"

using std::string;
using std::getline;
using std::istream;
using std::ostream;
using std::stringstream;
using std::cout;
using std::cerr;
using std::endl;

void setup_routing_inst(RoutingInst &inst, int gx, int gy, int cap, int nets) {
    // set up fields
    inst.gx = gx;
    inst.gy = gy;
    inst.cap = cap;
    inst.numNets = nets;
    inst.numCells = gx*gy;

    // allocate nets and cells from the same memory block, for cache efficiency.
    size_t net_size = nets * sizeof(Net);
    size_t num_bytes = net_size + (gx*gy) * sizeof(Cell);

    char *memory = new char[num_bytes];
    inst.nets = reinterpret_cast<Net *>(memory);
    inst.cells = reinterpret_cast<Cell *>(memory + net_size);

    // initialize grid cells with cap.
    for (int c = 0; c < inst.numCells; c++) {
        inst.cells[c].right.capacity = cap;
        inst.cells[c].right.utilization = 0;
        inst.cells[c].down.capacity = cap;
        inst.cells[c].down.utilization = 0;
    }
}

inline void apply_blockage(RoutingInst &inst, int x, int y, int ex, int ey, int new_cap) {
    if (x == ex) { // vertical
        for (; y < ey; y++) {
            inst.cell(x, y).down.capacity = new_cap;
        }
    } else {
        assert(y == ey);
        for (; x < ex; x++) {
            inst.cell(x, y).right.capacity = new_cap;
        }
    }
}

int fail(const char *msg) {
    cerr << "ERROR: " << msg << endl;
    return 0;
}

int readBenchmark(istream &in, RoutingInst &rst) {
    if (!in) return fail("Bad input stream");

    int linenum = -1;
    string line;
    string token;

    // read the grid size
    int gx = 0, gy = 0;
    if (!getline(in, line))  return fail("Couldn't read grid size");
    stringstream grid(line);
    grid >> token;
    if (token != "grid")     return fail("Benchmark doesn't start with 'grid'");
    if (!(grid >> gx >> gy)) return fail("Benchmark doesn't have grid size");
    if (gx < 0 || gy < 0)    return fail("Invalid grid size!");

    // read the capacity
    int capacity = 0;
    if (!getline(in, line))     return fail("Couldn't read capacity");
    stringstream capline(line);
    capline >> token;
    if (token != "capacity")    return fail("Benchmark line 2 doesn't start with capacity");
    if (!(capline >> capacity)) return fail("Benchmark doesn't have capacity");
    if (capacity <= 0)          return fail("Invalid capacity");

    // read the number of nets
    int numnets = 0;
    if (!getline(in, line)) return fail("Couldn't read num nets");
    stringstream netline(line);
    netline >> token;
    if (token != "num")     return fail("Line 3 does not start with 'num'");
    netline >> token;
    if (token != "net")     return fail("Line 3 does not start with 'num net'");
    netline >> numnets;
    if (numnets <= 0)       return fail("Bad number of nets");

    // allocate buffers
    setup_routing_inst(rst, gx, gy, capacity, numnets);

    // read the nets
    for (int n = 0; n < numnets; n++) {
        // set up the net
        if (!getline(in, line)) return fail("Couldn't read all the nets");
        stringstream netdef(line);
        int num_pins = -1;
        netdef >> token >> num_pins;
        if (num_pins <= 0)      return fail("Net had 0 or fewer pins");
        rst.nets[n].id = n; // TODO: do we actually ever need this?
        rst.nets[n].numPins = num_pins;
        rst.nets[n].pins = new Point[num_pins]; // TODO: Littering the heap, allocate from pool instead
        rst.nets[n].nroute.numSegs = -1; // TODO: may not need to label this...

        // read the points
        for (int c = 0; c < num_pins; c++) {
            if (!getline(in, line)) return fail("Couldn't read all net points");
            stringstream pindef(line);
            pindef >> rst.nets[n].pins[c].x >> rst.nets[n].pins[c].y;
        }
    }

    // read the blockage metadata
    if (!getline(in, line)) return fail("Couldn't read blockage count");
    stringstream blockhead(line);
    int num_blockages = -1;
    blockhead >> num_blockages;
    if (num_blockages < 0) return fail("Invalid number of blockages");

    // read the blockages
    for (int c = 0; c < num_blockages; c++) {
        int x1, y1, x2, y2, new_cap;
        if (!getline(in, line)) return fail("Couldn't read all blockages");
        stringstream blockage(line);
        if (!(blockage >> x1 >> y1 >> x2 >> y2 >> new_cap)) return fail("Invalid blockage");
        apply_blockage(rst, x1, y1, x2, y2, new_cap);
    }

    return 1;
}

int solveRouting(RoutingInst &rst){
    /*********** TO BE FILLED BY YOU **********/

    return 1;
}

int writeOutput(ostream &out, RoutingInst &rst){
    /*********** TO BE FILLED BY YOU **********/

    return 1;
}


int release(RoutingInst &rst){
    /*********** TO BE FILLED BY YOU **********/

    return 1;
}


