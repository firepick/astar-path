var mathjs = require("mathjs");
var GraphNode = require("./GraphNode");
var PriorityQ = require("./PriorityQ");

(function(exports) { 
    class AStarGraph {
        constructor(options = {}) {
            this.maxIterations = options.maxIterations || 10000;
            this.fastOpenSize = options.fastOpenSize || 8;
            this.stats = {};
        }
        neighborsOf(node, goal) {
            // NOTE: neighbors can be generated dynamically, but they must be unique
            throw new Error("neighborsOf(node) must be overridden by subclass");
        }
        cost(node1, node2) {
            // actual cost 
            throw new Error("cost(node1, node2) must be overridden by subclass");
        }
        estimateCost(node1, goal) {
            // estimatedCost must be admissible (i.e., less than or equal to actual cost)
            throw new Error("estimateCost(node1, goal) must be overridden by subclass");
        }
        pathTo(node) {
            var totalPath = [node];
            while ((node = node.cameFrom)) {
                totalPath.push(node);
            }
            return totalPath.reverse();
        }
        findPath(start, goal, options) { 
            // Implements A* algorithm
            var msStart = new Date();
            var pq = new PriorityQ({
                compare: (a,b) => a.fscore - b.fscore,
            });
            var onNeighbor = this.onNeighbor = options.onNeighbor || ((node,outcome) => node);
            var onCurrent = options.onCurrent || ((node)=>true);
            start.fscore = this.estimateCost(start, goal);
            start.gscore = 0;
            start.isOpen = true;
            pq.insert(start);
            var stats = this.stats.findPath = {
                iter: 0,
                nodes: 0,
                inOpen: 0,
            };
            var path = [];
            while (stats.iter++ < this.maxIterations) {
                var current = pq.extractMin();
                if (current == null) {
                    console.log("no solution. open set is empty");
                    break;
                }
                if (!onCurrent(current)) {
                    break;
                }
                if (current === goal) {
                    path = this.pathTo(current);
                    break;
                }
                current.isOpen = false;
                current.isClosed = true;
                for (var neighbor of this.neighborsOf(current, goal)) {
                    stats.nodes++;
                    if (neighbor.isClosed) {
                        onNeighbor(neighbor,"-cl");
                        continue;
                    }
                    var tentative_gScore = current.gscore + this.cost(current, neighbor);
                    if (tentative_gScore >= neighbor.gscore) {
                        onNeighbor(neighbor, (tentative_gScore > neighbor.gscore ? "-g>" : "-g=")); 
                        continue;
                    }
                    neighbor.cameFrom = current;
                    neighbor.gscore = tentative_gScore;
                    neighbor.fscore = neighbor.gscore + this.estimateCost(neighbor, goal);
                    if (neighbor.isOpen) {
                        onNeighbor(neighbor," -o");
                        stats.inOpen++;
                    } else {
                        neighbor.isOpen = true;
                        pq.insert(neighbor);
                        onNeighbor(neighbor,"+++");
                    }
                }
            }
            stats.ms = new Date() - msStart;
            stats.pqm = pq.bmax.map((b) => b && mathjs.round(b.fscore, 1));
            stats.pqb = pq.b.map((b) => b.length);
            stats.pqfill = pq.stats.fill;
            stats.pqslice = pq.stats.slice;
            stats.path = path.length;
            return {
                path: path,
                stats: stats,
            }
        }
    }

    module.exports = exports.AStarGraph = AStarGraph;
})(typeof exports === "object" ? exports : (exports = {}));

