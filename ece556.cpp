// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.

#include <assert.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>

#include "ece556.h"
#include "astar.h"
#include "svg.h"

extern "C" {
    #include <flute/flute.h>
   #undef min
   #undef max
   #undef abs
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
using std::min;
using std::max;
using std::abs;



// ------------------------- readBenchmark --------------------------------

void setup_routing_inst(RoutingInst &inst, int gx, int gy, int cap, int nets) {
    // set up fields
    inst.gx = gx;
    inst.gy = gy;
    inst.cap = cap;
    inst.numNets = nets;
    inst.numCells = gx*gy;

    inst.nets = new Net[nets];
    inst.utilization = new Cell[gx*gy];
    inst.virtual_cap = new Cell[gx*gy];

    for (int c = 0; c < gx*gy; c++) {
        inst.virtual_cap[c].right = cap;
        inst.virtual_cap[c].down = cap;
    }
}

inline void apply_blockage(RoutingInst &inst, int x, int y, int ex, int ey, int new_cap) {
    if (x == ex) { // vertical
        for (; y < ey; y++) {
            inst.util(x, y).down = inst.cap - new_cap;
        }
    } else {
        assert(y == ey);
        for (; x < ex; x++) {
            inst.util(x, y).right = inst.cap - new_cap;
        }
    }
}

void fail(const char *msg) {
    cerr << "ERROR: " << msg << endl;
    exit(1);
}

void readBenchmark(istream &in, RoutingInst &rst) {
    if (!in) return fail("Bad input file");

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
}



// ----------------------- Initial Solution ----------------------------

void use_edge(RoutingInst &inst, Net &net, int edge) {
    auto result = net.routed_edges.emplace(edge, 1);
    if (result.second) {
        inst.util(edge)++;
    } else {
        result.first->second++;
    }
}

void ripup_edge(RoutingInst &inst, Net &net, int edge) {
    auto result = net.routed_edges.find(edge);
    assert(result != net.routed_edges.end());

    result->second--;
    if (result->second == 0) {
        net.routed_edges.erase(result);
        inst.util(edge)--;
    }
}

void L_route(RoutingInst &rst, Net &net, Segment &seg) {
    int minX = min(seg.p1.x, seg.p2.x);
    int maxX = max(seg.p1.x, seg.p2.x);
    int minY = min(seg.p1.y, seg.p2.y);
    int maxY = max(seg.p1.y, seg.p2.y);

    // calculate cost of each L
    int bxpy_cost = 0;
    int bypx_cost = 0;
    for (int x = minX; x < maxX; x++) {
        bxpy_cost += rst.util(x, seg.p2.y).right;
        bxpy_cost += rst.util(x, seg.p1.y).right;
    }
    for (int y = minY; y < maxY; y++) {
        bxpy_cost += rst.util(seg.p1.x, y).down;
        bxpy_cost += rst.util(seg.p2.x, y).down;
    }

    // allocate edge indices
    // TODO: Better allocator.
    int numEdges = seg.numEdges = abs(seg.p1.x-seg.p2.x)+abs(seg.p1.y-seg.p2.y);
    int *edge = seg.edges = new int[numEdges];

    // mark less expensive L
    if (bxpy_cost < bypx_cost) {
        for (int x = minX; x < maxX; x++) {
            *edge = rst.edge_index(x, seg.p2.y, true);
            use_edge(rst, net, *edge);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            *edge = rst.edge_index(seg.p1.x, y, false);
            use_edge(rst, net, *edge);
            edge++;
        }
    } else {
        for (int x = minX; x < maxX; x++) {
            *edge = rst.edge_index(x, seg.p1.y, true);
            use_edge(rst, net, *edge);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            *edge = rst.edge_index(seg.p2.x, y, false);
            use_edge(rst, net, *edge);
            edge++;
        }
    }
    assert(edge == (seg.edges + numEdges));
}

void routeInitialSolutionShitty(RoutingInst &rst) {
    // just L-route all of the segments
    for (int n = 0; n < rst.numNets; n++) {
        int numSegs = rst.nets[n].numPins - 1;
        Segment *segments = new Segment[numSegs];
        rst.nets[n].nroute.numSegs = numSegs;
        rst.nets[n].nroute.segments = segments;

        for (int s = 0; s < numSegs; s++) {
            segments[s].p1 = rst.nets[n].pins[0];
            segments[s].p2 = rst.nets[n].pins[s+1];
            L_route(rst, rst.nets[n], segments[s]);
        }
    }
}

void routeInitialSolution(RoutingInst &rst) {
    // Use FLUTE to plot steiner trees for the initial solution
    // Then L-route the steiner trees
    int xs[MAXD*2];
    int *ys = xs + MAXD;

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
            numSegs++;
        }

        assert(numSegs <= maxNumCons);

        rst.nets[n].nroute.numSegs = numSegs;

        for (int s = 0; s < numSegs; s++) {
            L_route(rst, rst.nets[n], rst.nets[n].nroute.segments[s]);
        }

        free(tree.branch);
    }
}

// -------------------------- solveRouting ------------------------------

struct SegmentInfo {
    int overflow;
    Net *net;
    Segment *seg;

    SegmentInfo(Net *net, Segment *seg) noexcept : net(net), seg(seg) {}
    SegmentInfo(const SegmentInfo &other) noexcept
            : overflow(other.overflow), net(other.net), seg(other.seg) {}
    SegmentInfo &operator=(const SegmentInfo &other) noexcept {
        overflow = other.overflow;
        net = other.net;
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

inline int calculate_virtual_overflow(const RoutingInst &rst, const Segment &seg) {
  int of = 0;
  for (int c = 0; c < seg.numEdges; c++) {
    of += max(0, rst.util(seg.edges[c]) - rst.vcap(seg.edges[c]));
  }
  return of;
}

inline int calculate_total_overflow(const RoutingInst &rst) {
    int of = 0;
    for (int c = 0; c < rst.numCells; c++) {
        of += max(0, rst.utilization[c].right - rst.cap);
        of += max(0, rst.utilization[c].down - rst.cap);
    }
    return of;
}

void init_overflow(const RoutingInst &rst, vector<SegmentInfo> &seg_info) {
    // TODO: iteration like this shits all over the L2 cache
    // (because seg.seg is a ptr and they may be all out of order)
    for (auto &seg : seg_info) {
        seg.overflow = calculate_virtual_overflow(rst, *seg.seg);
    }
}

void ripup(RoutingInst &rst, SegmentInfo &seg) {
    for (int c = 0; c < seg.seg->numEdges; c++) {
        ripup_edge(rst, *seg.net, seg.seg->edges[c]);
    }
    delete [] seg.seg->edges;
    seg.seg->edges = nullptr;
}

void maze_route(RoutingInst &inst, Net *net, Segment *pSegment) {
    assert(pSegment->edges == nullptr);

    Point tl, br;
    const int bb_size = 20;
    tl.x = max(0, min(pSegment->p1.x, pSegment->p2.x) - bb_size);
    tl.y = max(0, min(pSegment->p1.y, pSegment->p2.y) - bb_size);
    br.x = min(inst.gx, max(pSegment->p1.x, pSegment->p2.x) + 1 + bb_size);
    br.y = min(inst.gy, max(pSegment->p1.y, pSegment->p2.y) + 1 + bb_size);
    vector<Point> path;
    maze_route_p2p(inst, *net, pSegment->p1, pSegment->p2, tl, br, path);

    pSegment->edges = new int[path.size()-1];
    pSegment->numEdges = int(path.size() - 1); // path better not be longer than 2^31
    for (int c = 0; c < path.size()-1; c++) {
        Point &prev = path[c];
        Point &curr = path[c+1];
        if (prev.x == curr.x) {
            assert(abs(prev.y - curr.y) == 1);
            if (prev.y < curr.y)
                pSegment->edges[c] = inst.edge_index(prev.x, prev.y, false);
            else
                pSegment->edges[c] = inst.edge_index(curr.x, curr.y, false);
        } else {
            assert(prev.y == curr.y);
            assert(abs(prev.x - curr.x) == 1);

            if (prev.x < curr.x)
                pSegment->edges[c] = inst.edge_index(prev.x, prev.y, true);
            else
                pSegment->edges[c] = inst.edge_index(curr.x, curr.y, true);
        }
        use_edge(inst, *net, pSegment->edges[c]);
    }
    assert(path.front() == pSegment->p1);
    assert(path.back()  == pSegment->p2);
}

void ripupAndReroute(RoutingInst &rst, vector<SegmentInfo> &seg_info, time_t time_limit) {
    cout << "Calculate overflow" << endl;
    init_overflow(rst, seg_info);
    std::sort(seg_info.begin(), seg_info.end(),
              [](const SegmentInfo &a, const SegmentInfo &b) -> bool
              { return a.overflow > b.overflow; }
    );

    cout << "Ripup" << endl;
    int over_count = 0;
    for (auto &info : seg_info) {
        if (info.overflow > 0) {
            ripup(rst, info);
            over_count++;
        }
        else break;
    }
    cout << over_count << " of " << seg_info.size() << " nets were overflowed (" << float(over_count*100)/seg_info.size() << "%)" << endl;

    cout << "Reroute" << endl;
    time_t start_time = time(nullptr);
    time_t lastElapsed = -1;
    int routed_count = 0;
    bool panicked = false;
    for (auto &info : seg_info) {
        if (info.overflow > 0) {
            if (!panicked) { // hurray for branch prediction!
                routed_count++;

                maze_route(rst, info.net, info.seg);

                // check time remaining
                time_t now = time(nullptr);
                time_t elapsed = now - start_time;
                if (elapsed - lastElapsed >= 1) {
                    time_t estimated = elapsed * over_count / routed_count;
                    cout << "\rRouted " << routed_count << " of " << over_count << " (" << elapsed << " elapsed, " <<
                    estimated << " total)." << std::flush;
                    lastElapsed = elapsed;

                    // panic if we have one minute left, and just L-route everything.
                    if (time_limit - now < 60) {
                        cout << "ohcrapohcrapohcrapohcrap runningrunningRUNNING!!!!";
                        panicked = true;
                    }
                }
            } else {
                // we are OUT OF TIME! L-route EVERYTHING!!!
                // TODO: A shittier, faster L-route that doesn't pick the optimal L.
                L_route(rst, *info.net, *info.seg);
            }
        }
        else break;
    }
    time_t elapsed = time(nullptr) - start_time;
    cout << "\r" << over_count << " nets routed in " << elapsed << " seconds." << endl;
}

void solveRouting(RoutingInst &rst, time_t time_limit, bool shitty_initial) {

    // find initial solution
    if (shitty_initial)
        routeInitialSolutionShitty(rst);
    else
        routeInitialSolution(rst);

    // build array of all segments
    vector<SegmentInfo> seg_info;
    for (int n = 0; n < rst.numNets; n++) {
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {
            seg_info.emplace_back(&rst.nets[n], &rst.nets[n].nroute.segments[s]);
        }
    }

    // iterate RUaRR until time limit is exceeded
    RoutingSolution currentBest;
    int currentBestOverflow;

    time_t startTime = time(nullptr);
    time_t lastTime = startTime; 
    int currentOverflow = 0;
    time_t currentTime = startTime;
    int overflow = calculate_total_overflow(rst);
    int lastOverflow = overflow;
    double timeChange = 0;
    double overflowChange = 0;
    double expectedQ = 0;
    double currentQ = 0;
    int secsIn15Min = 15*60;
    int secsIn5Min = 5*60;

    int ruarr_iter = 0;
    while(time_limit - time(nullptr) > 60) {
#ifndef NDEBUG
        stringstream filename;
        filename << "intermediate-" << ruarr_iter << ".html";
        string str = filename.str();
        writeCongestionSvg(rst, str.c_str());
#endif

        ruarr_iter++;

        if (ruarr_iter > 1) {
	  // update virtual capacity
	  for(int c = 0; c < rst.gx*rst.gy; c++) {
	    int capacity = rst.cap;
	    int utilRight = rst.utilization[c].right;
	    int utilDown = rst.utilization[c].down;
	    int overflowRight = utilRight - capacity;
	    int overflowDown = utilDown - capacity;
	    rst.virtual_cap[c].right = min(rst.virtual_cap[c].right - overflowRight, capacity);
	    rst.virtual_cap[c].down = min(rst.virtual_cap[c].down - overflowDown, capacity);
	  }
        }

        cout << "Overflow: " << overflow << endl;
        currentBestOverflow = overflow;
        currentBest.clone(rst);

        cout << "\nBeginning RipupAndReroute iteration " << ruarr_iter << endl;
        ripupAndReroute(rst, seg_info, time_limit);

        overflow = calculate_total_overflow(rst);
        currentTime = time(nullptr);
        currentOverflow = overflow;
        timeChange = (double)(currentTime - lastTime);
        overflowChange = (double)(currentOverflow - lastOverflow);
        currentQ = currentOverflow*(1 + ((currentTime - startTime)/secsIn15Min));
        expectedQ = (currentOverflow + overflowChange)*(1 + ((currentTime - startTime + timeChange)/secsIn15Min));
        if (overflow >= currentBestOverflow) {
            cout << "Overflow no longer decreasing!" << endl;
            std::move(currentBest).restore(rst);
            break;
        }else if((expectedQ >= currentQ) && ((currentTime - startTime) >= secsIn5Min)){
            cout << "Overflow not decreasing fast enough!" << endl;
            break;
        }
    }
}


// --------------------------- writeOutput -------------------------------

inline bool is_straight(int p1ex, int p2ex) {
    return (p1ex & 1) == (p2ex & 1);
}

inline void write_line(ostream &out, RoutingInst &inst, int edge1, int edge2) {
    if (edge1 > edge2)
        std::swap(edge1, edge2);
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

void writeOutput(ostream &out, RoutingInst &rst){
    if (!out) {
        cerr << "Bad output file" << endl;
        exit(1);
    }

    for (int n = 0; n < rst.numNets; n++) {
        out << 'n' << n << endl;
        for (int c = 0; c < rst.nets[n].nroute.numSegs; c++) {
            write_segment(out, rst, rst.nets[n].nroute.segments[c]);
        }
        out << '!' << endl;
    }
}

