//
// Created by Martin Wickham on 3/12/16.
//

#ifndef SILICON_ASTAR_H
#define SILICON_ASTAR_H

#include <algorithm>
#include <unordered_map>
#include <queue>
#include <assert.h>
#include "ece556.h"

#define OVERFLOW_EXPENSE 10;

struct AStarFrontierRecord {
    Point parent;
    Point self;
    int cost;
    int hcost;

    AStarFrontierRecord() {}
    AStarFrontierRecord(const Point &parent, const Point &self, int cost, int h)
            : parent(parent), self(self), cost(cost), hcost(h + cost)
    {
        assert(cost >= 0);
        assert(h >= 0);
    }

    // DO NOT USE! This is for std::priority_queue
    inline bool operator<(const AStarFrontierRecord &other) const {
        return hcost > other.hcost;
    }
};

struct AStarDomainRecord {
    Point parent;
#ifndef NDEBUG
    int cost;
#endif

    AStarDomainRecord() {}

    AStarDomainRecord(const AStarFrontierRecord &record)
            : parent(record.parent)
#ifndef NDEBUG
            , cost(record.cost)
#endif
    {}
};

// For std::unordered_map<Point, ...>
namespace std {
    template<>
    struct hash<Point> {
        inline size_t operator()(const Point &p) const {
            return hash<int>()(p.x) ^ hash<int>()(p.y);
        }
    };
}

typedef std::unordered_map<Point, AStarDomainRecord> Domain;
typedef std::priority_queue<AStarFrontierRecord> Frontier;

inline int default_cost(const RoutingInst &inst, int edge) {
    int newcost = inst.util(edge) + 1;
    if (newcost > inst.vcap(edge))
      newcost += (newcost - inst.vcap(edge)) * OVERFLOW_EXPENSE;
    return newcost;
}

// H(Point) is a lambda which returns the heuristic value h(x) for a point x.
// G(Point) is a lambda which returns true iff the given point is a goal point.
// Returns the cost of the solution
template<typename H, typename G, typename V, typename C>
inline int AStar(const RoutingInst &inst, Frontier &frontier, std::vector<Point> &path, H heuristic, G is_goal, V valid, C cost, int estimate) {
    assert(path.empty());
    long worstNodes = estimate * estimate;

    Domain explored;
    explored.reserve(worstNodes);

    while(!frontier.empty()) {

        const AStarFrontierRecord current = frontier.top();

        if (is_goal(current.self)) {
            // backtrace to beginning
            path.push_back(current.self);
            Point p = current.parent;
            while(p.x >= 0) {
                path.push_back(p);
                assert(explored.find(p) != explored.end());
                p = explored[p].parent;
            }
            std::reverse(path.begin(), path.end());
            return current.cost;
        }

        frontier.pop();

        auto check_repeat = explored.find(current.self);
        if (check_repeat != explored.end()) {
            assert(check_repeat->second.cost <= current.cost);
            continue;
        }
        explored.emplace(current.self, current);

        Point right_pt{current.self.x+1, current.self.y  };
        Point down_pt {current.self.x  , current.self.y+1};
        Point left_pt {current.self.x-1, current.self.y  };
        Point up_pt   {current.self.x  , current.self.y-1};
        int right = inst.edge_index(current.self.x, current.self.y, true );
        int down  = inst.edge_index(current.self.x, current.self.y, false);
        int left  = inst.edge_index(left_pt.x     , left_pt.y     , true );
        int up    = inst.edge_index(up_pt.x       , up_pt.y       , false);

        auto astar_add_child = [&](Point child, int edge) {
            if (child == current.parent) return;
            if (!valid(child)) return;

            int newcost = cost(edge);

            int total_cost = current.cost + newcost;
            auto it = explored.find(child);
            if (it != explored.end()) {
                assert(it->second.cost <= total_cost);
                return;
            }
            frontier.emplace(current.self, child, total_cost, heuristic(child));
        };

        astar_add_child(right_pt, right);
        astar_add_child(down_pt , down );
        astar_add_child(left_pt , left );
        astar_add_child(up_pt   , up   );
    }
    assert(false);
    return -1;
}

inline int maze_route_p2p(const RoutingInst &inst, const Net &net, const Point &start, const Point &end, const Point &tl, const Point &br, std::vector<Point> &path) {
    assert(inst.valid(start));
    assert(inst.valid(end));

    Frontier frontier;
    frontier.emplace(Point{-1,-1}, start, 0, 0); // heuristic cost doesn't matter here because we pop immediately
    return AStar(inst, frontier, path,
                 [end](const Point p) -> int  {return abs(p.x-end.x) + abs(p.y-end.y);},
                 [end](const Point p) -> bool {return p == end;},
                 [tl, br](const Point p) -> bool {return p.x >= tl.x && p.y >= tl.y && p.x < br.x && p.y < br.y;},
                 [&inst, &net](const int e) -> int {
                     if (net.routed_edges.find(e) != net.routed_edges.end()) {
                         return 1; // just wirelength, no overflow cost
                     }
                     return default_cost(inst, e);
                 },
                 abs(start.x - end.x) + abs(start.y - end.y));
}

#endif //SILICON_ASTAR_H
