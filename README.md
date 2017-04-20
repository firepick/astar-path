## Whatizit?
**astar-path** is a Javascript library that computes a ballistic flight path
between a start and goal nodes. The trajectory computed is subject to
position, velocity, and accelerations constraints as well as optional
user defined node constraints. 

### PathNode
Trajectories are defined by a PathNode array that is computed by the `findPath` method
of a PathFactory instance when given a start node and a goal node. Here we create
a three dimensional PathFactory that forbids x or y axis movement below a zcruise height
of 15 and also forbids z-movement below zero:

<code>
var zcruise = 15;
var pf = new PathFactory({
    dimensions: 3,
    maxVelocity: [25,25,4], // x,y,z velocity
    maxAcceleration: [5,5,1], // x,y,z acceleration
    maxIterations: 5000,
    isConstrained: (node) => node.s[2] < zcruise,
    constrain: (n) => {
        if (n.s[2] < 0) {
            return null; // only paths above bed
        }
        if (n.s[2] < zcruise) {
            if (n.v[0] || n.v[1] || n.a[0] || n.a[1]) {
                return null; // no xy movement below zcruise;
            }
        }
        return n;
    },
});
</code>

<a href="https://github.com/firepick/astar-path/wiki/images/constrainzy.png">
    <img src="https://github.com/firepick/astar-path/wiki/images/constrainzy.png" height=400px></a>

### Installation
`npm install astar-path`
