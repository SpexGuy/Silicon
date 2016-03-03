// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include "ece556.h"

using std::istream;
using std::ostream;

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

    // mark nets as uninitialized (TODO: do we really need to do this?)
    for (int c = 0; c < inst.numNets; c++) {
        inst.nets[c].id = -1;
    }

    // initialize grid cells with cap.
    for (int c = 0; c < inst.numCells; c++) {
        inst.cells[c].right.capacity = cap;
        inst.cells[c].right.utilization = 0;
        inst.cells[c].down.capacity = cap;
        inst.cells[c].down.utilization = 0;
    }
}

int readBenchmark(istream &in, RoutingInst &rst) {
    /*********** TO BE FILLED BY YOU **********/

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


