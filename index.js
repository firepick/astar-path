(function(exports) {
    ////////////////// constructor
    function AStarPath() {
        var that = this;
        return that;
    }

    ///////////////// class ////////////////////
    AStarPath.PathFactory = require("./src/PathFactory");
    AStarPath.AStarGraph = require("./src/AStarGraph");
    AStarPath.GraphNode = require("./src/GraphNode");
    AStarPath.PathNode = require("./src/PathNode");
    AStarPath.PriorityQ = require("./src/PriorityQ");

    module.exports = exports.AStarPath = AStarPath;
})(typeof exports === "object" ? exports : (exports = {}));

// mocha -R min --inline-diffs *.js
(typeof describe === 'function') && describe("AStarPath", function() {
    var AStarPath = exports.AStarPath; // require("./AStarPath");

})
