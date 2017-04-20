var mathjs = require("mathjs");
var AStarGraph = require("./AStarGraph");
var PathNode = require("./PathNode");
var PriorityQ = require("./PriorityQ");

(function(exports) { 
    function* iNeighborsCruise(pf, node, goal, anewbasis) {
        // generate restricted set of neighbors (8 for 3D)
        var neighbor;
        var asteady = [];
        for (var i = 0; i < anewbasis.length; i++) { 
            var basis = anewbasis[i];
            var a0 = basis[0];
            if (a0 == null) {
                return; // no values
            }
            asteady.push(a0); // asteady is no change in acceleration neighbor

            // The adown neighbors decreases acceleration for all but one of the axes
            var adown = [];
            if (anewbasis.length === 1) {
                adown.push(basis[basis.length-1]);
            } else {
                for (var j = 0; j < anewbasis.length; j++) {
                    adown.push(i===j ? basis[0] : basis[basis.length-1]);
                }
            }
            (neighbor = pf.createNeighbor(adown,node,goal)) && (yield neighbor);
        }
        (neighbor = pf.createNeighbor(asteady,node,goal)) && (yield neighbor); // da==0
        for (var i = 0; i < anewbasis.length; i++) { // da>0
            var basis = anewbasis[i];
            if (basis.length < 3) { // already generated
                continue;
            }
            var aup = [];
            for (var j = 0; j < anewbasis.length; j++) {
                if (i==j) {
                    aup && aup.push(basis[1]);
                } else {
                    aup && aup.push(basis[0]);
                }
            }
            aup && (neighbor = pf.createNeighbor(aup,node,goal)) && (yield neighbor);
        }
    }
    function* iNeighborsAccel(pf, node, goal, anewbasis) {
        // generate all possible neighbors (27 for 3D)
        var neighbor;
        var permutations = PathFactory.permutations(anewbasis);
        permutations = Array.from(permutations);
        //console.log("permutations", permutations, "anewbasis", anewbasis, "node", JSON.stringify(node));
        for (var anew of permutations) { 
            (neighbor = pf.createNeighbor(anew,node,goal)) && (yield neighbor);
        }
    }
    
    class PathFactory extends AStarGraph {
        constructor(options={}) {
            super(options);
            this.dimensions = options.dimensions || 1;
            this.estimateFree = options.estimateFree || this.cost_tsvva;
            this.estimateConstrained = options.estimateConstrained || this.cost_explore;
            this.cost = options.cost || this.cost_constant(0.2);
            this.vMax = options.maxVelocity || Array(this.dimensions).fill(10);
            this.aMax = options.maxAcceleration || Array(this.dimensions).fill(2);
            this.jMin = options.minJerk || Array(this.dimensions).fill(0.1);
            if (options.constrain) {
                this.constrain = options.constrain;
                this.isConstrained = options.isConstrained || ((node) => true);
            } else {
                this.constrain = ((neighbor) => neighbor);
                this.isConstrained = options.isConstrained || ((node) => false);
            }
            this.onNoPath = options.onNoPath || ((start, goal, onNoPath) => onNoPath(start, goal));
            this.jerkScale = 1; // fraction of path length
            this.jMax = this.aMax; // initial value
            this.round = 2; // force discrete graph space
            this.nodeId = 0;
            this.stats = {
                lookupTotal: 0,
                lookupHit: 0,
                tsva: 0,
                neighborsOf: 0,
            };
            this.clear();
        }
        clear() {
            this.nodeMap = {};
        }
        createNeighbor(avariation,node,goal) {
            var vvariation = mathjs.add(node.v,avariation); // instantaneous acceleration
            var svariation = mathjs.add(node.s,vvariation); // instantaneous acceleration
            var neighbor = this.svaToNode(svariation, vvariation, avariation);
            var n = this.constrain(neighbor);
            if (n) {
                if (this.estimateCost(neighbor, goal) < Number.MAX_SAFE_INTEGER) {
                    return n;
                }
            } else {
                this.onNeighbor && this.onNeighbor(neighbor,"-cn");
            }
            return null;
        }
        isCruiseNode(node) {
            return node.a.length === node.a.reduce((acc,a) => !a ? (1+acc) : acc, 0) && // not accel
                node.v.reduce((acc,v) => v ? (1+acc) : acc, 0) > 0; // moving
        }
        svaToNode(position, velocity, acceleration) {
            var node = new PathNode(position, velocity, acceleration);
            return this.lookupNode(node);
        }
        toJSON() {
            var obj = super.toJSON();
            console.log("ha");
            obj.f && (obj.f = ((obj.f*100+0.5)|0)/100);
            return obj;
        }
        iAxisAccelerations(node, i, dsgoal) {
            var pf = this;
            var cullOvershoot = !node.c;
            function * iAxisAccelerations(node, i, dsgoal) {
                var ai = node.a[i];
                var aMax = pf.aMax[i];
                var aMin = -aMax;
                var jMax = pf.jMax[i];
                var aplus = ai + jMax;
                var aminus = ai - jMax;
                if (node.c) { // client must constrain neighbors
                    yield(ai); // maintain acceleration
                    aplus = aMax < aplus ? aMax : aplus;
                    aminus = aminus < aMin ? aMin : aminus;
                    if (dsgoal >= 0) { // [a, aplus, aminus]
                        ai < aplus && (yield aplus); // faster
                        aminus < ai && (yield aminus); // slower
                    } else { // [a, aminus, aplus]
                        aminus < ai && (yield aminus); // slower
                        ai < aplus && (yield aplus); // faster
                    }
                    return;
                }

                var vi = node.v[i];
                var vplus = vi + aplus;
                var vminus = vi + aminus;
                var vzero = vi + ai;
                var vMax = pf.vMax[i];
                var vMin = -vMax;
                if (dsgoal >= 0) {
                    var ds = dsgoal;
                    var dsvMax = ds < vMax ? ds : vMax; // prevent overshoot

                    (vzero <= 0 || vzero <= ds) &&
                    (vMin <= vzero && vMin <= vzero && vzero <= dsvMax) &&
                    (yield ai); // maintain acceleration

                    (vplus <= 0 || vplus <= ds) &&
                    (ai < aMax && vMin <= vplus && vplus <= dsvMax) &&
                    (yield (aMax < aplus ? aMax : aplus));
                    
                    //(vminus <= 0 || vminus <= ds) && // allow overshoot for deceleration
                    (aMin < ai && vMin <= vminus && vminus <= vMax) &&
                    (yield (aMin > aminus ? aMin : aminus));
                } else {
                    var dsvMin = vMin < dsgoal ? dsgoal : vMin; // prevent overshoot

                    (0 <= vzero || dsgoal <= vzero) &&
                    (vMin <= vzero && dsvMin <= vzero && vzero <= vMax) &&
                    (yield ai); // maintain acceleration

                    (0 <= vminus || dsgoal <= vminus) &&
                    (aMin < ai && dsvMin <= vminus && vminus <= vMax) &&
                    (yield (aMin > aminus ? aMin : aminus));

                    //(0<vplus || dsgoal <= vplus) && // allow overshoot for deceleration
                    (ai < aMax && vMin <= vplus && vplus <= vMax) &&
                    (yield (aMax < aplus ? aMax : aplus));
                }
            }
            return iAxisAccelerations(node, i, dsgoal);
        }
        lookupNode(node) {
            var key = node.key; 
            var result = this.nodeMap[key];
            this.stats.lookupTotal++;
            if (result) {
                this.stats.lookupHit++;
            } else {
                result = this.nodeMap[key] = node;
                node.c = this.isConstrained(node) ? 1 : 0;
                node.id = ++this.nodeId;
            }
            return result;
        }
        axisAccelerations(node, i, dsgoal) {
            return Array.from(this.iAxisAccelerations(node, i, dsgoal));
        }
        static validateNode(node) {
            if (!(node instanceof PathNode)) {
                throw new Error("Expected neigbhorsOf(?PathNode?,...)");
            }
            return node;
        }
        predecessors(node) {
            node.h = node.h || 0;
            var preds = [];
            // Simplifying assumption: predecessors are slow moving cruising nodes that can stop in one step
            var moving = node.v.reduce((acc,v) => v ? true : acc, false);
            var pushPredecessor = (s,v) => {
                var pred = this.constrain(this.svaToNode(s, v));
                if (pred) {
                    pred.h = node.h + this.cost(pred, node);
                    preds.push(pred);
                }
            }
            if (moving) {
                var s = node.s.map((s,i) => s - node.v[i]);
                pushPredecessor(s, node.v);
            } else {
                for (var i=0; i < this.dimensions; i++) {
                    var vplus = this.aMax.map((a,j) => i===j ? a : 0);
                    var splus = node.s.map((s,j) => i===j ? s-vplus[i] : s);
                    pushPredecessor(splus, vplus);
                    var vminus = this.aMax.map((a,j) => i===j ? -a : 0);
                    var sminus = node.s.map((s,j) => i===j ? s-vminus[i] : s);
                    pushPredecessor(sminus, vminus);
                }
            }
            return preds;
        }
        findFreeGoal(goal) { // return path to goal from unconstrained predecessor
            this.nodeMap = {}; // discard prior results
            var pq = new PriorityQ({
                compare: (a,b) => a.h - b.h,
            });
            var stats = this.stats.findFree = {
                iter: 0,
                nodes: 0,
            };
            pq.insert(goal);
            var path = [];
            while (stats.iter++ < this.maxIterations) {
                var current = pq.extractMin();
                if (current == null) {
                    console.log("no solution. open set is empty");
                    break;
                }
                if (!current.c) {
                    path = this.pathTo(current).reverse();
                    break;
                }
                this.predecessors(current).forEach((pred) => {
                    pred.cameFrom = current;
                    pq.insert(pred);
                });
            }
            return path;
        }
        findPath(start, goal, options) { 
            start = this.svaToNode(
                mathjs.round(start.s, this.round),
                mathjs.round(start.v, this.round),
                mathjs.round(start.a, this.round)
            );
            goal = this.svaToNode(
                mathjs.round(goal.s, this.round),
                mathjs.round(goal.v, this.round),
                mathjs.round(goal.a, this.round)
            );
            var goalPath = this.findFreeGoal(goal);
            var goalFree = goalPath[0];
            var ds = mathjs.abs(mathjs.subtract(goal.s, start.s));
            var jerk = mathjs.round(mathjs.divide(ds, this.jerkScale), this.round);
            this.jMax = jerk.map((j,i) => mathjs.min(this.aMax[i], mathjs.max(j, this.jMin[i])));
            var result = super.findPath(start, goalFree, options);
            result.stats.start = start.s;
            result.stats.goal = goal.s;
            var onNoPath = (start,goal,onNoPath) => {
                throw new Error("No path found\n" +
                    "  start:"+JSON.stringify(start)+"\n" +
                    "   goal:"+JSON.stringify(goal)+"\n" +
                    "  nodes:"+this.nodeId+"\n"+
                    "   iter:"+this.maxIterations
                    );
            };
            if (result.path.length) {
                result.path = result.path.concat(goalPath.slice(1));
            } else {
                this.onNoPath(start, goal, onNoPath);
            }

            return result;
        }
        isGoalNeighbor(node, goal) { // a goal is zero-velocity
            var verbose = false;

            for (var i = 0; i < this.dimensions; i++) {
                var jMax = this.jMax[i];
                var ds = goal.s[i] - node.s[i];

                if (ds < -jMax || jMax < ds) {
                    verbose && console.log("too far", ds,  JSON.stringify(node));
                    return false; // too far
                }

                var vi = node.v[i];
                if (vi < -jMax || jMax < vi) {
                    verbose && console.log("too fast", JSON.stringify(node));
                    return false; // too fast
                }
                if (vi < 0 && ds > 0 || vi > 0 && ds < 0) {
                    verbose && console.log("wrong way velocity", JSON.stringify(node));
                    return false; // wrong way velocity
                }

                var ai = node.a[i];
                if (ai < -jMax || jMax < ai) {
                    verbose && console.log("too jerky", JSON.stringify(node));
                    return false; // too jerky
                }

                var dv = 0 - vi;
                if (dv > 0 && ai < -jMax || dv < 0 && ai > jMax) { // allow 2 * jerk acceleration when stopping
                    verbose && console.log("wrong way acceleration", JSON.stringify(node));
                    return false; // wrong way acceleration
                }
            }
            return true;
        }
        neighborsOf(node, goal) {
            this.stats.neighborsOf++;
            if (goal && this.isGoalNeighbor(node, goal)) {
                return [goal];
            }
            var createNeighbor = (avariation) => {
                var vvariation = mathjs.add(node.v,avariation); // instantaneous acceleration
                var svariation = mathjs.add(node.s,vvariation); // instantaneous acceleration
                var neighbor = this.svaToNode(svariation, vvariation, avariation);
                var n = this.constrain(neighbor);
                if (n) {
                    if (this.estimateCost(neighbor, goal) < Number.MAX_SAFE_INTEGER) {
                        return n;
                    }
                } else {
                    this.onNeighbor && this.onNeighbor(neighbor,"-cn");
                }
                return null;
            }
            var anewbasis = node.a.map((a,i) => this.axisAccelerations(node, i, goal.s[i]-node.s[i]));
            return !node.c && this.isCruiseNode(node)
                ? iNeighborsCruise(this, node, goal, anewbasis)
                : iNeighborsAccel(this, node, goal, anewbasis);
        }
        tsvva(s, v1, v2, amax) {
            this.stats.tsva++;
            if (s < 0) {
                return this.tsvva(-s, -v1, -v2, amax);
            }
            var d = amax * s + 0.5*(v1*v1 + v2*v2);
            var pm = 2*mathjs.sqrt(d);
            var n1 = pm - v1 - v2;
            //var n2 = -pm - v1 - v2; // never positive
            //return (n2 < 0 ? n1 : n2)/amax;
            return n1/amax;
        }
        cost_constant(cost) {
            return (n1,n2) => n1 == n2 ? 0 : cost;
        }
        cost_distance(n1, goal) {
            var sumsquares = n1.s.reduce((acc,s,i) => {
                var diff = goal.s[i] - s;
                return acc + diff * diff;
            },0);
            return mathjs.sqrt(sumsquares);
        }
        cost_explore(n1, goal) {
            var constrained_cost = n1.v.reduce((acc,v,i) => acc + (this.vMax[i]-v)/this.vMax[i], 0); 
            return this.estimateFree(n1,goal) + constrained_cost;
        }
        cost_tsvva(n1, goal) {
            var ds = mathjs.subtract(goal.s, n1.s);
            var t = ds.map((s,i) => this.tsvva(s, n1.v[i], goal.v[i], this.aMax[i]));
            return mathjs.sum(t);
        }
        cost_tsvva_explore(n1,goal) { // explore aggressively with maximum speed
            return n1.v.reduce((acc,v,i) => 
                acc + 
                +(this.vMax[i]-v)/this.vMax[i]
                +this.tsvva(goal.s[i]-n1.s[i], n1.v[i], goal.v[i], this.aMax[i]),
                0); 
        }
        estimateCost(n1, goal) {
            if (n1.h) {
                return n1.h; // cached estimate
            }
            if (n1.c) { // constrained node
                return this.estimateConstrained(n1, goal);
            }
            return this.estimateFree(n1, goal);
        }
        static get PathNode() {
            return PathNode;
        }
        static permutations(vv) {
            var vvelts = vv.reduce((acc, e) => e == null || !e.length ? 0:(acc+1), 0);
            if (vv.length !== vvelts) {
                return [];
            }
            function * variations() {
                var vi = Array(vv.length).fill(0);
                for(;;) {
                    yield vv.map((v,i) => v[vi[i]]);
                    for (var i = 0; i < vv.length; i++) {
                        if (++vi[i] < vv[i].length) {
                            break;
                        }
                        vi[i] = 0;
                        if (i+1 >= vv.length) {
                            return; 
                        }
                    }
                } 
            }
            return Array.from(variations());
        }
    }

    module.exports = exports.PathFactory = PathFactory;
})(typeof exports === "object" ? exports : (exports = {}));

