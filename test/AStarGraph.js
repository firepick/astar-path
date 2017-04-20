var mathjs = require("mathjs");
var GraphNode = require("../src/GraphNode");
var PriorityQ = require("../src/PriorityQ");
var AStarGraph = require("../src/AStarGraph");

(typeof describe === 'function') && describe("AStarGraph", function() {
    var should = require("should");

    it("AStarGraph subclass finds shortest path", function() {
        var verbose = 0;
        // define a simple graph with weighted transitions between named nodes
        var nodeCosts = {
            START: {
                A: 1, // huge hills
                B: 3, // optimal
                C: 2, // dead end
            },
            A: { A1: 1, A2: 2, },
            A1: { END: 100, },
            A2: { END: 50, },
            B: { B1: 3, },
            B1: { END: 3, },
            C: { },
            END: { },
        }
        var nodes = {};
        Object.keys(nodeCosts).forEach((name) => nodes[name] = new GraphNode({name:name}));

        // extends AStarGrap with appropriate
        class TestGraph extends AStarGraph {
            constructor(costs, options) {
                super(options);
                this.costs = costs;
            }
            neighborsOf(node, goal) { 
                return Object.keys(this.costs[node.name]).map((name) => nodes[name]);
            }
            cost(n1, n2) {
                var neighborCost = n2 && this.costs[n1.name][n2.name]; // n2 is a neighbor of n1
                if (!neighborCost) {
                    throw new Error("cannot compute cost to non-neighbor");
                }
                return neighborCost;
            }
            estimateCost(n1, goal) {
                if (n1 === goal) {
                    return 0;
                }
                var neighborCost = goal && this.costs[n1.name][goal.name]; // goal is a neighbor of n1
                var minNeighborCost = () => { // if goal is not a neighbor of n1
                    // compute cost as minimum cost of n1 to its neighbors
                    var neighborCosts = this.costs[n1.name];
                    return Object.keys(neighborCosts).reduce(
                        (acc, name) => mathjs.min(acc, neighborCosts[name]),
                        Number.MAX_SAFE_INTEGER);
                }
                return neighborCost || minNeighborCost();
            }
        }

        var graph = new TestGraph(nodeCosts);
        var START = nodes.START;
        var A = nodes.A;
        var A1 = nodes.A1;
        var A2 = nodes.A2;
        var B = nodes.B;
        var B1 = nodes.B1;
        var END = nodes.END;
        graph.estimateCost(START, END).should.equal(1);
        graph.estimateCost(START, B).should.equal(3);
        graph.cost(A2, END).should.equal(50);
        graph.estimateCost(A2, END).should.equal(50);

        var options = {
            onCurrent: (current) => { // called whenever current node changes
                if (verbose) {
                    console.log(JSON.stringify(current),
                        "f:"+current.fscore,
                        "g:"+current.gscore,
                        ""); 
                }
                return true;
            },
            onNeighbor: (node, outcome) => { // called whenever a node is rejected
                if (verbose) {
                    console.log(outcome, JSON.stringify(node), gscore_new, node.gscore);
                }
                return null;
            },
        };
        var path = graph.findPath(START, END, options).path; 
        should.deepEqual(path.map((n) => n.name), ["START","B","B1","END"]);
    })
    it("push/pop are faster than unshift/shift", function() {
        var start = {color: "purple"};
        var msStart = new Date();
        for (var i=0; i<100; i++) {
            var a = [];
            for (var j=0; j<1000; j++) {
                a.push(start);
            }
            for (var j=0; j<1000; j++) {
                a.pop();
            }
        }
        var msElapsedPush = new Date() - msStart;

        var a = [];
        var msStart = new Date();
        for (var i=0; i<200; i++) {
            var a = [];
            for (var j=0; j<1000; j++) {
                a.unshift(start);
            }
            for (var j=0; j<1000; j++) {
                a.shift();
            }
        }
        var msElapsedUnshift = new Date() - msStart;
        msElapsedPush.should.below(msElapsedUnshift/2);
    })
})
