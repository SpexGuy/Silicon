// ECE556 - Copyright 2014 University of Wisconsin-Madison.  All Rights Reserved.


#ifndef ECE556_H
#define ECE556_H

#include <assert.h>
#include <iostream>

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
};


/**
 * A structure to represent nets
 */
struct Net {
    int id ; 		    /* ID of the net */ // TODO: is this really necessary?
    int numPins ; 		/* number of pins (or terminals) of the net */
    Point *pins ; 		/* array of pins (or terminals) of the net. */
    Route nroute ;		/* stored route for the net. */
};

struct Edge {
    int utilization;
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
    int gx ;		/* x dimension of the global routing grid */
    int gy ;		/* y dimension of the global routing grid */

    int cap ;

    int numNets;	/* number of nets */
    Net *nets;		/* array of nets */

    int numCells; 	/* number of cells in the grid */
    Cell *cells;

    inline int index(const int x, const int y) const {
        // naive row-major scheme for now...
        // TODO: morton curve or something better for traversing
        assert(valid(x, y));
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

inline float usage(const RoutingInst &inst, const Edge &edge) {
    return float(edge.utilization) / float(inst.cap);
}

/* int readBenchmark(const char *fileName, routingInst *rst)
   Read in the benchmark file and initialize the routing instance.
   This function needs to populate all fields of the routingInst structure.
   input1: fileName: Name of the benchmark input file
   input2: pointer to the routing instance
   output: 1 if successful
*/
int readBenchmark(std::istream &in, RoutingInst &rst);


/* int solveRouting(routingInst *rst)
   This function creates a routing solution
   input: pointer to the routing instance
   output: 1 if successful, 0 otherwise (e.g. the data structures are not populated) 
*/
int solveRouting(RoutingInst &rst, int time_limit);

/* int writeOutput(const char *outRouteFile, routingInst *rst)
   Write the routing solution obtained from solveRouting(). 
   Refer to the project link for the required output format.

   Finally, make sure your generated output file passes the evaluation script to make sure
   it is in the correct format and the nets have been correctly routed. The script also reports
   the total wirelength and overflow of your routing solution.

   input1: name of the output file
   input2: pointer to the routing instance
   output: 1 if successful, 0 otherwise 
  */
int writeOutput(std::ostream &out, RoutingInst &rst);


#endif // ECE556_H
