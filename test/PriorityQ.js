var mathjs = require("mathjs");
var PriorityQ = require("../src/PriorityQ");

(typeof describe === 'function') && describe("PriorityQ", function() {
    var should = require("should");

    it("PriorityQ(options) can have a custom comparator", function() {
        var pq = new PriorityQ({
            compare: (a,b) => a.value - b.value,
        });
        pq.insert({value:10});
        pq.insert({value:5});
        pq.insert({value:1});
        pq.insert({value:5});
        pq.extractMin().value.should.equal(1);
        pq.extractMin().value.should.equal(5);
        pq.extractMin().value.should.equal(5);
        pq.extractMin().value.should.equal(10);
    })
    it("insert() should increase length", function() {
        var pq = new PriorityQ();
        pq.length.should.equal(0);
        pq.insert(10);
        pq.insert(1);
        pq.insert(5);
    })
    it("extractMin() removes and returns smallest element", function() {
        var pq = new PriorityQ();
        pq.insert(10);
        pq.insert(5);
        pq.insert(1);
        pq.insert(5);
        pq.validate();
        pq.extractMin().should.equal(1);
        pq.validate();
        pq.extractMin().should.equal(5);
        pq.extractMin().should.equal(5);
        pq.extractMin().should.equal(10);
        should(pq.extractMin()).equal(null);
        pq.validate();
    })
    it("PriorityQ(options) can use multiple buckets", function() {
        var verbose = 0;
        var pq = new PriorityQ({
            sizes: [3, 3, Number.MAX_SAFE_INTEGER],
        });
        pq.validate();
        pq.insert(3);
        pq.insert(1);
        pq.insert(7);
        pq.insert(6);
        pq.insert(4);
        pq.insert(0);
        pq.insert(10);
        pq.insert(2);
        pq.insert(9);
        pq.insert(8);
        pq.insert(5);
        should.deepEqual(pq.b, [
            [3, 1, 0, 2],
            [7, 6, 4,5],
            [10, 9, 8],
        ]);
        pq.validate();
        pq.insert(11);
        should.deepEqual(pq.b, [
            [3, 1, 0, 2],
            [7, 6, 4,5],
            [10, 9, 8, 11],
        ]);
        pq.length.should.equal(12);
        var n = pq.length;
        for (var i = 0; i < n; i++) {
            var v = pq.extractMin();
            verbose && console.log("buckets", v, JSON.stringify(pq.b));
            should(v).equal(i);
            pq.length.should.equal(n-i-1);
        }
        should.deepEqual(pq.b, [
            [],
            [],
            [],
        ]);
        should(pq.extractMin()).equal(null);
    });
    it("take(n,src) return n values from src", function() {
        var pq = new PriorityQ();
        var src = [1,3,2,3,7,5];
        var dst = pq.take(0, src);
        should.deepEqual(dst, []);
        should.deepEqual(src, [7,5,3,3,2,1]);
        var dst = pq.take(3, src);
        should.deepEqual(dst, [1,2,3]); // destination is sorted
        should.deepEqual(src, [7,5,3]);
        var dst = pq.take(4, src);
        should.deepEqual(dst, [3,5,7]); // destination is sorted
        should.deepEqual(src, []);
        var dst = pq.take(3, null);
        should.deepEqual(dst, []);
    });
    it("summarize() generates summary", function() {
        var pq = new PriorityQ();
        var log = "";
        pq.summarize({
            log: (...x) => (log += JSON.stringify(x)),
        });
        log.length.should.above(50);
    });
    it("PriorityQ() can handle many items", function() {
        this.timeout(60*1000);
        var verbose = 0;
        var n = 10000;
        function test() {
            var pq = new PriorityQ();

            var values = [];
            for (var i=0; i < n; i++) {
                v = mathjs.random(-10,10)+i/100; // random number with increasing mean
                v = mathjs.round(v, 2);
                values.push(v);
            }

            var msStart = new Date();
            values.forEach((v) => pq.insert(v));
            var msElapsed = new Date() - msStart;
            verbose>1 && console.log("insert", msElapsed);

            values.sort(pq.compare);

            verbose && console.log("pq", "max:"+pq.bmax, 
                "buckets:"+pq.b.map((b) => b.length),
                "stats:"+JSON.stringify(pq.stats),
                "");
            pq.validate();

            var msStart = new Date();
            for (var i=0; i<values.length; i++) {
                var v = pq.extractMin();
                v.should.equal(values[i]);
            }
            var msElapsed = new Date() - msStart;
            verbose>0 && console.log("extractMin", msElapsed);
        }
        for (var itest = 0; itest < 1; itest++) {
            test();
        }
    });
})
