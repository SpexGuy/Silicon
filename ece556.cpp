// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <string>
#include <sstream>
#include <assert.h>
#include <vector>
#include <algorithm>

#include "ece556.h"
#include "astar.h"

extern "C" {
    #include <flute/flute.h>
}


using std::string;
using std::vector;
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
        inst.cells[c].right.utilization = 0;
        inst.cells[c].down.utilization = 0;
    }
}

inline void apply_blockage(RoutingInst &inst, int x, int y, int ex, int ey, int new_cap) {
    if (x == ex) { // vertical
        for (; y < ey; y++) {
            inst.cell(x, y).down.utilization = inst.cap - new_cap;
        }
    } else {
        assert(y == ey);
        for (; x < ex; x++) {
            inst.cell(x, y).right.utilization = inst.cap - new_cap;
        }
    }
}

int fail(const char *msg) {
    cerr << "ERROR: " << msg << endl;
    return 0;
}

int readBenchmark(istream &in, RoutingInst &rst) {
    if (!in) return fail("Bad input stream");

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

    readLUT(); // setup FLUTE

    return 1;
}

void L_route(RoutingInst &rst, Segment &seg) {
    int minX = min(seg.p1.x, seg.p2.x);
    int maxX = max(seg.p1.x, seg.p2.x);
    int minY = min(seg.p1.y, seg.p2.y);
    int maxY = max(seg.p1.y, seg.p2.y);

    // calculate cost of each L
    int bxpy_cost = 0;
    int bypx_cost = 0;
    for (int x = minX; x < maxX; x++) {
        bxpy_cost += rst.cell(x, seg.p2.y).right.utilization;
        bxpy_cost += rst.cell(x, seg.p1.y).right.utilization;
    }
    for (int y = minY; y < maxY; y++) {
        bxpy_cost += rst.cell(seg.p1.x, y).down.utilization;
        bxpy_cost += rst.cell(seg.p2.x, y).down.utilization;
    }

    // allocate edge indices
    // TODO: Better allocator.
    int numEdges = seg.numEdges = abs(seg.p1.x-seg.p2.x)+abs(seg.p1.y-seg.p2.y);
    int *edge = seg.edges = new int[numEdges];

    // mark more expensive L
    if (bxpy_cost > bypx_cost) {
        for (int x = minX; x < maxX; x++) {
            rst.cell(x, seg.p2.y).right.utilization++;
            *edge = rst.edge_index(x, seg.p2.y, true);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            rst.cell(seg.p1.x, y).down.utilization++;
            *edge = rst.edge_index(seg.p1.x, y, false);
            edge++;
        }
    } else {
        for (int x = minX; x < maxX; x++) {
            rst.cell(x, seg.p1.y).right.utilization++;
            *edge = rst.edge_index(x, seg.p1.y, true);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            rst.cell(seg.p2.x, y).down.utilization++;
            *edge = rst.edge_index(seg.p2.x, y, false);
            edge++;
        }
    }
    assert(edge == (seg.edges + numEdges));
}

struct SegmentInfo {
    int overflow;
    Segment *seg;

    SegmentInfo(Segment *seg) noexcept : seg(seg) {}
    SegmentInfo(const SegmentInfo &other) noexcept
            : overflow(other.overflow), seg(other.seg) {}
    SegmentInfo &operator=(const SegmentInfo &other) noexcept {
        overflow = other.overflow;
        seg = other.seg;
        return *this;
    }
};

inline int calculate_overflow(const RoutingInst &rst, const Segment &seg) {
    int of = 0;
    for (int c = 0; c < seg.numEdges; c++) {
        of += rst.overflow(seg.edges[c]);
    }
    return of;
}

void init_overflow(const RoutingInst &rst, vector<SegmentInfo> &seg_info) {
    // TODO: iteration like this shits all over the L2 cache
    // (because seg.seg is a ptr and they may be all out of order)
    for (auto &seg : seg_info) {
        seg.overflow = calculate_overflow(rst, *seg.seg);
    }
}

int solveRouting(RoutingInst &rst) {
    int xs[MAXD*2];
    int *ys = xs + MAXD;

    vector<SegmentInfo> seg_info;
    for (int n = 0; n < rst.numNets; n++) {

        for (int c = 0; c < rst.nets[n].numPins; c++) {
            xs[c] = rst.nets[n].pins[c].x;
            ys[c] = rst.nets[n].pins[c].y;
        }

        Tree tree = flute(rst.nets[n].numPins, xs, ys, ACCURACY);

        int maxNumCons = tree.deg*2 - 3;
        int numSegs = 0;
        rst.nets[n].nroute.segments = new Segment[maxNumCons];

        for (int c = 0; c < tree.deg*2 - 2; c++) {
            Branch &base = tree.branch[c];
            if (base.n == c) continue;
            Branch &parent = tree.branch[base.n];
            if (base.x == parent.x && base.y == parent.y) continue;

            rst.nets[n].nroute.segments[numSegs].p1 = Point{base.x, base.y};
            rst.nets[n].nroute.segments[numSegs].p2 = Point{parent.x, parent.y};
            seg_info.emplace_back(&rst.nets[n].nroute.segments[numSegs]);
            numSegs++;
        }

        assert(numSegs <= maxNumCons);

        rst.nets[n].nroute.numSegs = numSegs;

        for (int s = 0; s < numSegs; s++) {
            L_route(rst, rst.nets[n].nroute.segments[s]);
        }

        free(tree.branch);
    }

    cout << "Starting overflow" << endl;
    init_overflow(rst, seg_info);
    std::sort(seg_info.begin(), seg_info.end(),
              [](const SegmentInfo &a, const SegmentInfo &b) -> bool
              { return a.overflow > b.overflow; }
    );
    int over_count = 0;
    for (auto &info : seg_info) {
        if (info.overflow > 0) over_count++;
        else break;
    }
    cout << over_count << " of " << seg_info.size() << " nets are overflowed (" << float(over_count*100)/seg_info.size() << "%)" << endl;
    return 1;
}

inline bool is_straight(int p1ex, int p2ex) {
    return (p1ex & 1) == (p2ex & 1);
}

inline void write_line(ostream &out, RoutingInst &inst, int edge1, int edge2) {
    assert(edge1 <= edge2); // Temporary, maybe.
    out << inst.point_from_edge(edge1) << '-' << inst.point(inst.end(edge2)) << endl;
}

inline void write_segment(ostream &out, RoutingInst &rst, Segment &segment) {
    int edge = segment.edges[0];
    for (int c = 1; c < segment.numEdges; c++) {
        if (!is_straight(edge, segment.edges[c])) {
            write_line(out, rst, edge, segment.edges[c - 1]);
            edge = segment.edges[c];
        }
    }
    write_line(out, rst, edge, segment.edges[segment.numEdges-1]);
}

int writeOutput(ostream &out, RoutingInst &rst){
    for (int n = 0; n < rst.numNets; n++) {
        out << 'n' << n << endl;
        for (int c = 0; c < rst.nets[n].nroute.numSegs; c++) {
            if (rst.nets[n].nroute.segments[c].empty()) continue;
            write_segment(out, rst, rst.nets[n].nroute.segments[c]);
        }
        out << '!' << endl;
    }
    return 1;
}

