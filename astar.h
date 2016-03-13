//
// Created by Martin Wickham on 3/12/16.
//

#ifndef SILICON_ASTAR_H
#define SILICON_ASTAR_H

#include <unordered_map>
#include <queue>
#include <assert.h>
#include "ece556.h"

struct AStarFrontierRecord {
    Point parent;
    Point self;
    int cost;
    int h;

    AStarFrontierRecord() {}
    AStarFrontierRecord(const Point &parent, const Point &self, int cost, int h)
            : parent(parent), self(self), cost(cost), h(h)
    {
        assert(cost >= 0);
        assert(h >= 0);
    }

    // DO NOT USE! This is for std::priority_queue
    inline bool operator<(const AStarFrontierRecord &other) const {
        return (h + cost) > (other.h + other.cost);
    }
};

struct AStarDomainRecord {
    Point parent;
    int cost;

    AStarDomainRecord() {}
    AStarDomainRecord(const AStarFrontierRecord &record)
            : parent(record.parent), cost(record.cost) {}
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
template<typename H, typename G>
inline void AStar(const RoutingInst &inst, Frontier &frontier, H heuristic, G is_goal) {
    Domain explored;
    while(!frontier.empty()) {
        const AStarFrontierRecord current = frontier.top();
        if (is_goal(current.self)) {
            // TODO: backtrace
            return;
        }
        frontier.pop();

        auto check_repeat = explored.find(current.self);
        if (check_repeat != explored.end()) {
            assert(check_repeat->second.cost <= current.cost);
            continue;
        }
        explored[current.self] = current;

        Point right{current.self.x+1, current.self.y  };
        Point down {current.self.x  , current.self.y+1};
        Point left {current.self.x-1, current.self.y  };
        Point up   {current.self.x  , current.self.y-1};


#define astar_add_child(child, util_pt, util_dir) do { \
            if (child == current.parent) continue; \
            if (child.x < 0 || child.x >= inst.gx) continue; \
            if (child.y < 0 || child.y >= inst.gy) continue; \
            \
            int cost = current.cost + inst.cell(util_pt).util_dir.utilization + 1; \
            auto it = explored.find(child); \
            if (it != explored.end()) { \
                assert(it->second.cost <= cost); \
                continue; \
            } \
            frontier.push(AStarFrontierRecord(current.self, child, cost, heuristic(child))); \
        } while(0)

        astar_add_child(right, current.self, right);
        astar_add_child(down , current.self, down );
        astar_add_child(left , left        , right);
        astar_add_child(up   , up          , down );

#undef astar_add_child
    }
}

inline void maze_route_p2p(const RoutingInst &inst, const Point &start, const Point &end) {
    Frontier frontier;
    frontier.emplace(Point{-1,-1}, start, 0, 0); // heuristic cost doesn't matter here because we pop immediately
    return AStar(inst, frontier,
                 [&end](const Point &p) -> int  {return abs(p.x-end.x) + abs(p.y-end.y);},
                 [&end](const Point &p) -> bool {return p == end;});
}

inline void maze_route_p2p_dijkstra(const RoutingInst &inst, const Point &start, const Point &end) {
    Frontier frontier;
    frontier.emplace(Point{-1,-1}, start, 0, 0); // heuristic cost doesn't matter here because we pop immediately
    return AStar(inst, frontier,
                 [&end](const Point &p) -> int  {return 0;},
                 [&end](const Point &p) -> bool {return p == end;});
}


#endif //SILICON_ASTAR_H
