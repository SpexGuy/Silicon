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

const int FLUTE_SCALE = 10000;

const bool useCongestionAwareInitial = false;
const bool useCongestionAwareTreeGen = true;


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

inline void use_edge(RoutingInst &inst, Net &net, int edge) {
    auto result = net.routed_edges.emplace(edge, 1);
    if (result.second) {
        inst.util(edge)++;
    } else {
        result.first->second++;
    }
}

inline void ripup_edge(RoutingInst &inst, Net &net, int edge) {
    auto result = net.routed_edges.find(edge);
    assert(result != net.routed_edges.end());

    result->second--;
    if (result->second == 0) {
        net.routed_edges.erase(result);
        inst.util(edge)--;
    }
}

template<typename Util>
void L_route(RoutingInst &rst, Net &net, Segment &seg, Util u) {
    int minX = min(seg.p1.x, seg.p2.x);
    int maxX = max(seg.p1.x, seg.p2.x);
    int minY = min(seg.p1.y, seg.p2.y);
    int maxY = max(seg.p1.y, seg.p2.y);

    // calculate cost of each L
    int bxpy_cost = 0;
    int bypx_cost = 0;
    for (int x = minX; x < maxX; x++) {
        bxpy_cost += u(rst.edge_index(x, seg.p2.y, true));
        bypx_cost += u(rst.edge_index(x, seg.p1.y, true));
    }
    for (int y = minY; y < maxY; y++) {
        bxpy_cost += u(rst.edge_index(seg.p1.x, y, false));
        bypx_cost += u(rst.edge_index(seg.p2.x, y, false));
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
void L_route(RoutingInst &rst, Net &net, Segment &seg) {
    L_route(rst, net, seg, [&rst](int idx) -> int {return rst.util(idx);});
}
void L_route_all(RoutingInst &rst) {
    for (int n = 0; n < rst.numNets; n++) {
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {
            L_route(rst, rst.nets[n], rst.nets[n].nroute.segments[s]);
        }
    }
}

void three_bend_route(RoutingInst &rst, Net &net, Segment &seg) {
    
    const int margin = 10;
    int minX = min(seg.p1.x, seg.p2.x) - margin;
    int maxX = max(seg.p1.x, seg.p2.x) + margin;
    int minY = min(seg.p1.y, seg.p2.y) - margin;
    int maxY = max(seg.p1.y, seg.p2.y) + margin;
    int numX = maxX - minX + 1;
    int numY = maxY - minY + 1;

    Cell *cells = new Cell[numX * numY];
    
    // Calculate Costs 
    for (int y = 0; y < numY; y++) {
        for (int x = 0; x < numX; x++) {
            // Horizonal Cost
            if(x != 0) {
                cells[x + (numX*y)].right = rst.util(x+minX-1,y+minY).right + cells[x-1 +(numX*y)].right;
            }

            // Vertical
            if (y != 0) {
                cells[x + (numX*y)].down = rst.util(x+minX,y+minY-1).down + cells[x + (numX*(y-1))].down;
            }
        }
    }

    // Calculate cost of four possible 3 bend paths
    int bestX, bestY;
    int bestCost = std::numeric_limits<int>::max();
    bool bestFirst, bestThird;
    for (int y = 0; y < numY; y++) {
        for (int x = 0; x < numX; x++) {
           int costFirst = abs(cells[seg.p1.x + (numX * seg.p1.y)].right - cells[x + (numX * seg.p1.y)].right) 
                       + abs(cells[x + (numX *seg.p1.y)].down - cells[x + (numX * y)].down);

           int costSecond = abs(cells[seg.p1.x + (numX * seg.p1.y)].right - cells[seg.p1.x + (numX * y)].right)
                       + abs(cells[seg.p1.x + (numX * y)].down - cells[x + (numX * y)].down); 
            
           int costThird = abs(cells[seg.p2.x + (numX * seg.p2.y)].right - cells[x + (numX * seg.p2.y)].right)
                       + abs(cells[x + (numX * seg.p2.y)].down - cells[x + (numX * y)].down); 

           int costFourth = abs(cells[seg.p2.x + (numX * seg.p2.y)].right - cells[seg.p2.x + (numX * y)].right)
                       + abs(cells[seg.p2.x + (numX * y)].down - cells[x + (numX * y)].down); 
           
           bool first = costFirst < costSecond;
           bool third = costThird < costFourth;
           int totalCost = min(costFirst, costSecond) + min(costThird, costFourth);
           if (totalCost < bestCost) {
               bestX = x;
               bestY = y;
               bestCost = totalCost;
               bestFirst = first;
               bestThird = third;
           }

        }
    }

    // route through bestX, bestY using first/second and third/fourth for bestCost cost
    
    int numEdges = seg.numEdges = abs(seg.p1.x-bestX)+abs(seg.p1.y-bestY) +
                                  abs(seg.p2.x-bestX)+abs(seg.p2.y-bestY);
    int *edge = seg.edges = new int[numEdges];

    minX = min(seg.p1.x, bestX);
    maxX = max(seg.p1.x, bestX);
    minY = min(seg.p1.y, bestY);
    maxY = max(seg.p1.y, bestY);
    if (bestFirst) {
        for (int x = minX; x < maxX; x++) {
            *edge = rst.edge_index(x, seg.p1.y, true);
            use_edge(rst, net, *edge);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            *edge = rst.edge_index(bestX, y, false);
            use_edge(rst, net, *edge);
            edge++;
        }
    } else {
        for (int x = minX; x < maxX; x++) {
            *edge = rst.edge_index(x, bestY, true);
            use_edge(rst, net, *edge);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            *edge = rst.edge_index(seg.p1.x, y, false);
            use_edge(rst, net, *edge);
            edge++;
        }
    }

    minX = min(seg.p2.x, bestX);
    maxX = max(seg.p2.x, bestX);
    minY = min(seg.p2.y, bestY);
    maxY = max(seg.p2.y, bestY);
    if (bestThird) {
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
            *edge = rst.edge_index(x, seg.p2.y, true);
            use_edge(rst, net, *edge);
            edge++;
        }
        for (int y = minY; y < maxY; y++) {
            *edge = rst.edge_index(bestX, y, false);
            use_edge(rst, net, *edge);
            edge++;
        }
    }

    assert(edge == (seg.edges + numEdges));
    delete [] cells;
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

void make_segments(Net &net, Tree &tree) {
    int maxNumCons = tree.deg*2 - 3;
    int numSegs = 0;
    net.nroute.segments = new Segment[maxNumCons];

    for (int c = 0; c < tree.deg*2 - 2; c++) {
        Branch &base = tree.branch[c];
        if (base.n == c) continue;
        Branch &parent = tree.branch[base.n];
        if (base.x == parent.x && base.y == parent.y) continue;

        net.nroute.segments[numSegs].p1 = Point{base.x, base.y};
        net.nroute.segments[numSegs].p2 = Point{parent.x, parent.y};
        numSegs++;
    }

    assert(numSegs <= maxNumCons);
    net.nroute.numSegs = numSegs;
}

void flute_calculate_segments(RoutingInst &rst) {
    int xs[MAXD*2];
    int *ys = xs + MAXD;

    for (int n = 0; n < rst.numNets; n++) {

        for (int c = 0; c < rst.nets[n].numPins; c++) {
            xs[c] = rst.nets[n].pins[c].x;
            ys[c] = rst.nets[n].pins[c].y;
        }

        Tree tree = flute(rst.nets[n].numPins, xs, ys, ACCURACY);

        make_segments(rst.nets[n], tree);

        free(tree.branch);
    }
}

void ripup_congestion(vector<int> &congestion, const RoutingInst &rst, const Segment &seg) {
    int minx = min(seg.p1.x, seg.p2.x);
    int maxx = max(seg.p1.x, seg.p2.x);
    int miny = min(seg.p1.y, seg.p2.y);
    int maxy = max(seg.p1.y, seg.p2.y);

    if (miny == maxy) {
        for (int x = minx; x < maxx; x++) {
            congestion[rst.edge_index(x, miny, true)] -= 2; // count as both top and bottom
        }
    } else if (minx == maxx) {
        for (int y = miny; y < maxy; y++) {
            congestion[rst.edge_index(minx, y, false)] -= 2;
        }
    } else {
        for (int x = minx; x < maxx; x++) {
            congestion[rst.edge_index(x, miny, true)]--;
            congestion[rst.edge_index(x, maxy, true)]--;
        }
        for (int y = miny; y < maxy; y++) {
            congestion[rst.edge_index(minx, y, false)]--;
            congestion[rst.edge_index(maxx, y, false)]--;
        }
    }
}

void setup_congestion(vector<int> &congestion, const RoutingInst &rst) {
    for (int n = 0; n < rst.numNets; n++) {
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {

            auto &seg = rst.nets[n].nroute.segments[s];

            int minx = min(seg.p1.x, seg.p2.x);
            int maxx = max(seg.p1.x, seg.p2.x);
            int miny = min(seg.p1.y, seg.p2.y);
            int maxy = max(seg.p1.y, seg.p2.y);

            if (miny == maxy) {
                for (int x = minx; x < maxx; x++) {
                    congestion[rst.edge_index(x, miny, true)] += 2; // count as both top and bottom
                }
            } else if (minx == maxx) {
                for (int y = miny; y < maxy; y++) {
                    congestion[rst.edge_index(minx, y, false)] += 2;
                }
            } else {
                for (int x = minx; x < maxx; x++) {
                    congestion[rst.edge_index(x, miny, true)]++;
                    congestion[rst.edge_index(x, maxy, true)]++;
                }
                for (int y = miny; y < maxy; y++) {
                    congestion[rst.edge_index(minx, y, false)]++;
                    congestion[rst.edge_index(maxx, y, false)]++;
                }
            }

        }
    }
}

void routeInitialSolutionCongestion(RoutingInst &rst) {
    cout << "Using congestion estimation for initial solution" << endl;
    flute_calculate_segments(rst);

    vector<int> congestion; // ok, sometimes RAII is nice. But defer would be better!
    congestion.resize(2UL * rst.gx * rst.gy, 0); // fill with 0

    setup_congestion(congestion, rst);

    // Do one round of 1-by-1 R&R using the congestion map
    // Hopefully with the congestion map it's order invariant...
    for (int n = 0; n < rst.numNets; n++) {
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {
            ripup_congestion(congestion, rst, rst.nets[n].nroute.segments[s]);

            L_route(rst, rst.nets[n], rst.nets[n].nroute.segments[s],
                    [&rst, &congestion](int idx) -> int {return congestion[idx] + 2*rst.util(idx);});
        }
    }
}
void routeInitialSolution(RoutingInst &rst) {
    flute_calculate_segments(rst);
    L_route_all(rst);
}


// -------------- rerouteCongestionAwareInitialSolution -----------------

template<bool horz>
int calculate_average_congestion(const RoutingInst &rst, int minx, int miny, int maxx, int maxy) {
    if (maxx == minx || maxy == miny) return 0;

    int total_congestion = 0;
    for (int y = miny; y < maxy; y++) {
        for (int x = minx; x < maxx; x++) {
            total_congestion += rst.util(rst.edge_index(x, y, horz));
        }
    }

    if (horz)
        return (FLUTE_SCALE * total_congestion) / (rst.cap * (maxy - miny)) + 1;
    else
        return (FLUTE_SCALE * total_congestion) / (rst.cap * (maxx - minx)) + 1;
}

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

void ripup(RoutingInst &rst, const SegmentInfo &seg) {
    for (int c = 0; c < seg.seg->numEdges; c++) {
        ripup_edge(rst, *seg.net, seg.seg->edges[c]);
    }
    delete [] seg.seg->edges;
    seg.seg->edges = nullptr;
}

void rerouteCongestionAwareInitialSolution(RoutingInst &rst) {
    // TODO: This redoes a lot of the work that FLUTE does.
    // If we need to make this faster, we can save off the sorted array and pass it into FLUTE.
    // beware: this is a lot of memory (like 8 pages).  Hopefully it doesn't segfault or stack overflow.

    // note MAXD = 800
    int xs[MAXD];
    int ys[MAXD];
    int xp[MAXD];
    int yp[MAXD];
    int xscales[MAXD-1];
    int yscales[MAXD-1];
    int adjusted_xs[MAXD];
    int adjusted_ys[MAXD];

    for (int n = 0; n < rst.numNets; n++) {
        if (rst.nets[n].numPins <= 2) continue;

        int p;
        for (p = 0; p < rst.nets[n].numPins; p++) {
            xs[p] = rst.nets[n].pins[p].x;
            ys[p] = rst.nets[n].pins[p].y;
            xp[p] = p;
            yp[p] = p;
        }

        // setup xp and yp as sorted permutations for x and y
        std::sort(xp, xp+p, [xs](const int a, const int b) -> bool {return xs[a] < xs[b];});
        std::sort(yp, yp+p, [ys](const int a, const int b) -> bool {return ys[a] < ys[b];});

        // no sense warping if only one axis
        if (xs[xp[p-1]] == xs[xp[0]]) continue;
        if (ys[yp[p-1]] == ys[yp[0]]) continue;

        // calculate the average congestion values to be used as scales
        for (int c = 0; c < p-1; c++) {
            xscales[c] = calculate_average_congestion<true>(rst, xs[xp[c]], ys[yp[0]], xs[xp[c+1]], ys[yp[p-1]]);
            yscales[c] = calculate_average_congestion<false>(rst, xs[xp[0]], ys[yp[c]], xs[xp[p-1]], ys[yp[c+1]]);
        }

        // setup adjusted_xs and adjusted_ys as the warped points
        int xPos = 0, yPos = 0;
        for (int c = 0; c < p; c++) {
            adjusted_xs[xp[c]] = xPos;
            adjusted_ys[yp[c]] = yPos;
            xPos += xscales[c];
            yPos += yscales[c];
        }

        // calculate the steiner tree for the warped points
        Tree t = flute(p, adjusted_xs, adjusted_ys, ACCURACY);

        // now map that tree back onto the indexes.
        // yeah, this is O(n^2). I can't think of a better way to do it.
        xPos = 0;
        yPos = 0;
        for (int c = 0; c < p; c++) {
            for (int d = 0; d < t.deg*2-2; d++) {
                if (t.branch[d].x == xPos)
                    t.branch[d].x = -c-1;
                if (t.branch[d].y == yPos)
                    t.branch[d].y = -c-1;
            }

            xPos += xscales[c];
            yPos += yscales[c];
        }



        // then map those indexes onto the points
        for (int d = 0; d < t.deg*2-2; d++) {
            assert(t.branch[d].x < 0); // make sure we actually mapped all the points
            assert(t.branch[d].y < 0);

            t.branch[d].x = xs[xp[-t.branch[d].x-1]];
            t.branch[d].y = ys[yp[-t.branch[d].y-1]];
        }

        // clear up the segments in the existing solution
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {
            ripup(rst, SegmentInfo(&rst.nets[n], &rst.nets[n].nroute.segments[s]));
        }
        delete [] rst.nets[n].nroute.segments;

        // finally, map the tree onto segments.
        make_segments(rst.nets[n], t);

        // and route it
        for (int s = 0; s < rst.nets[n].nroute.numSegs; s++) {
            L_route(rst, rst.nets[n], rst.nets[n].nroute.segments[s]);
        }

        // and then clean up memory
        free(t.branch);
    }
}

// -------------------------- solveRouting ------------------------------

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
    else if (useCongestionAwareInitial)
        routeInitialSolutionCongestion(rst);
    else
        routeInitialSolution(rst);

    if (useCongestionAwareTreeGen)
        rerouteCongestionAwareInitialSolution(rst);

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

