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

// H(Point) is a lambda which returns the heuristic value h(x) for a point x.
// G(Point) is a lambda which returns true iff the given point is a goal point.
// Returns the cost of the solution
template<typename H, typename G, typename V>
inline int AStar(const RoutingInst &inst, Frontier &frontier, std::vector<Point> &path, H heuristic, G is_goal, V valid, int estimate) {
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

        Point right{current.self.x+1, current.self.y  };
        Point down {current.self.x  , current.self.y+1};
        Point left {current.self.x-1, current.self.y  };
        Point up   {current.self.x  , current.self.y-1};

        #define astar_add_child(child, util_pt, util_dir) do { \
            if (child == current.parent) continue; \
            if (!valid(child)) continue; \
            \
            int newcost = inst.cell(util_pt).util_dir.utilization + 1; \
            if (newcost > inst.cap) \
                newcost += (newcost-inst.cap) * OVERFLOW_EXPENSE; \
            \
            int cost = current.cost + newcost; \
            auto it = explored.find(child); \
            if (it != explored.end()) { \
                assert(it->second.cost <= cost); \
                continue; \
            } \
            frontier.emplace(current.self, child, cost, heuristic(child)); \
        } while(0)

        astar_add_child(right, current.self, right);
        astar_add_child(down , current.self, down );
        astar_add_child(left , left        , right);
        astar_add_child(up   , up          , down );

        #undef astar_add_child
    }
    assert(false);
    return -1;
}

inline int maze_route_p2p(const RoutingInst &inst, const Point &start, const Point &end, const Point &tl, const Point &br, std::vector<Point> &path) {
    assert(inst.valid(start));
    assert(inst.valid(end));

    Frontier frontier;
    frontier.emplace(Point{-1,-1}, start, 0, 0); // heuristic cost doesn't matter here because we pop immediately
    return AStar(inst, frontier, path,
                 [end](const Point p) -> int  {return abs(p.x-end.x) + abs(p.y-end.y);},
                 [end](const Point p) -> bool {return p == end;},
                 [tl, br](const Point p) -> bool {return p.x >= tl.x && p.y >= tl.y && p.x < br.x && p.y < br.y;},
                 abs(start.x - end.x) + abs(start.y - end.y));
}

#endif //SILICON_ASTAR_H
