var AStarGraph = require("./AStarGraph");
var GraphNode = require("../src/GraphNode");

(typeof describe === 'function') && describe("GraphNode", function() {
    const should = require("should");

    it("GraphNode(props) creates an AStarGraph node with given properties", function() {
        var node = new GraphNode({
            color: "purple",
        });
        should.deepEqual(JSON.parse(JSON.stringify(node)), {
            color: "purple",
            id: 0,
        });
    })
})
