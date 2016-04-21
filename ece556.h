// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.


#ifndef ECE556_H
#define ECE556_H

#include <assert.h>
#include <iostream>
#include <unordered_set>
#include <unordered_map>

/**
 * A structure to represent a 2D Point.
 */
struct Point {
    int x; /* x coordinate ( >=0 in the routing grid)*/
    int y; /* y coordinate ( >=0 in the routing grid)*/

    inline bool operator==(const Point &other) const {
        return x == other.x && y == other.y;
    }
    inline bool operator!=(const Point &other) const {
        return !(*this == other);
    }
};

inline std::ostream &operator<<(std::ostream &out, const Point &p) {
    return out << '(' << p.x << ',' << p.y << ')';
}


/**
 * A structure to represent a segment
 */
struct Segment {
    Point p1 ; 	/* start point of a segment */
    Point p2 ; 	/* end point of a segment */

    int numEdges ; 	/* number of edges in the segment*/
    int *edges ;  	/* array of edges representing the segment*/

    inline bool empty() {
        return p1 == p2;
    }

    inline Segment &operator=(const Segment &other) {
        numEdges = other.numEdges;
        edges = new int[numEdges];
        for (int e = 0; e < numEdges; e++) {
            edges[e] = other.edges[e];
        }

        return *this;
    }
};

inline std::ostream &operator<<(std::ostream &out, const Segment &s) {
    return out << s.p1 << '-' << s.p2;
}


/**
 * A structure to represent a route
 */
struct Route {
    int numSegs ;  	/* number of segments in a route*/
    Segment *segments ;  /* an array of segments (note, a segment may be flat, L-shaped or any other shape, based on your preference */

    Route &operator=(const Route &other) {
        numSegs = other.numSegs;
        segments = new Segment[numSegs];
        for (int s = 0; s < numSegs; s++) {
            segments[s] = other.segments[s];
        }

        return *this;
    }
};


/**
 * A structure to represent nets
 */
struct Net {
    int id ; 		    /* ID of the net */ // TODO: is this really necessary?
    int numPins ; 		/* number of pins (or terminals) of the net */
    Point *pins ; 		/* array of pins (or terminals) of the net. */
    Route nroute ;		/* stored route for the net. */

    std::unordered_map<int, int> routed_edges; // reference counted xD
};

struct Edge {
    int utilization = 0;
};

// THE CODE DEPENDS ON THIS STRUCTURE FOR A CELL!!!
// DO NOT MODIFY.
struct Cell {
    Edge right;
    Edge down;
};

/**
 * A structure to represent the routing instance
 */
struct RoutingInst {
    int gx = -1;		/* x dimension of the global routing grid */
    int gy = -1;		/* y dimension of the global routing grid */

    int cap = -1;

    int numNets = -1;	/* number of nets */
    Net *nets = nullptr;		/* array of nets */

    int numCells = -1; 	/* number of cells in the grid */
    Cell *cells = nullptr;

    inline int index(const int x, const int y) const {
        // naive row-major scheme for now...
        // TODO: morton curve or something better for traversing
        //assert(valid(x, y)); // this breaks A* (but it's safe, I promise!)
        return y * gx + x;
    }
    inline int edge_index(const int x, const int y, bool horz) const {
        return (index(x, y) << 1) | (horz ? 0 : 1);
    }

    inline Point point(const int idx) const {
        return Point{idx % gx, idx / gx};
    }
    inline Point point_from_edge(const int edge) const {
        return point(edge >> 1);
    }
    // converts edge idx -> point idx
    inline int end(int edge) {
        return (edge >> 1) + ((edge & 1) ? gx : 1);
    }

    inline Cell &cell(const int x, const int y) {
        return cells[index(x, y)];
    }
    inline const Cell &cell(const int x, const int y) const {
        return cells[index(x, y)];
    }
    inline Cell &cell(const Point &p) {
        return cell(p.x, p.y);
    }
    inline const Cell &cell(const Point &p) const {
        return cell(p.x, p.y);
    }

    inline Edge &edge(const int index) {
        return reinterpret_cast<Edge *>(cells)[index];
    }
    inline const Edge &edge(const int index) const {
        return reinterpret_cast<const Edge *>(cells)[index];
    }

    inline bool valid(int x, int y) const {
        return x >= 0 && x < gx &&
               y >= 0 && y < gy;
    }
    inline bool valid(const Point &p) const {
        return valid(p.x, p.y);
    }

    inline int overflow(const int edge_index) const {
        assert(valid(point_from_edge(edge_index)));
        return std::max(edge(edge_index).utilization - cap, 0);
    }
};

/**
 * The information needed to restore a RoutingInst
 */
struct RoutingSolution {
    int numNets = -1;	/* number of nets */
    Net *nets = nullptr;		/* array of nets */

#ifndef NDEBUG
    int numCells = -1; 	/* number of cells in the grid */
    Cell *cells = nullptr;
#endif

    inline void clone(const RoutingInst &other) {
        numNets = other.numNets;
        nets = new Net[numNets];
        for (int n = 0; n < numNets; n++) {
            nets[n].nroute = other.nets[n].nroute;
        }

#ifndef NDEBUG
        numCells = other.numCells;
        cells = new Cell[numCells];
        for (int c = 0; c < numCells; c++) {
            cells[c] = other.cells[c];
        }
#endif
    }

    inline void destroy() {
        if (nets != nullptr) {
            for (int n = 0; n < numNets; n++) {
                for (int s = 0; s < nets[n].nroute.numSegs; s++) {
                    delete [] nets[n].nroute.segments[s].edges;
                }
                delete [] nets[n].nroute.segments;
            }
            delete [] nets;
        }

#ifndef NDEBUG
        if (cells != nullptr) {
            delete[] cells;
        }
#endif
    }

    void restore(RoutingInst &other) && {
        std::swap(numNets, other.numNets);
        std::swap(nets, other.nets);
#ifndef NDEBUG
        std::swap(numCells, other.numCells);
        std::swap(cells, other.cells);
#endif
    }

    ~RoutingSolution() {
        destroy();
    }
};

/* int readBenchmark(const char *fileName, routingInst *rst)
   Read in the benchmark file and initialize the routing instance.
   This function needs to populate all fields of the routingInst structure.
   input1: fileName: Name of the benchmark input file
   input2: pointer to the routing instance
*/
void readBenchmark(std::istream &in, RoutingInst &rst);


/* int solveRouting(routingInst *rst)
   This function creates a routing solution
   input1: pointer to the routing instance
   input2: time at which the routing must be finished
   input3: whether to do shitty net decomposition
*/
void solveRouting(RoutingInst &rst, time_t time_limit, bool shitty_initial);

/* int writeOutput(const char *outRouteFile, routingInst *rst)
   Write the routing solution obtained from solveRouting(). 
   Refer to the project link for the required output format.

   Finally, make sure your generated output file passes the evaluation script to make sure
   it is in the correct format and the nets have been correctly routed. The script also reports
   the total wirelength and overflow of your routing solution.

   input1: the output file
   input2: pointer to the routing instance
   output: 1 if successful, 0 otherwise 
  */
void writeOutput(std::ostream &out, RoutingInst &rst);


#endif // ECE556_H
