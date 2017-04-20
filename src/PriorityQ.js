var mathjs = require("mathjs");

(function(exports) { 
    class PriorityQ {
        constructor(options = {}) {
            this.sizes = options.sizes || [5, 50, 500, Number.MAX_SAFE_INTEGER];
            this.split = options.split || [16,64,4096]; // split bucket when it exceeds this size
            this.b = this.sizes.map(() => []);
            this.bmax = this.sizes.map(() => null);
            this.length = 0;
            this.stats = {
                fill: 0,
                split: 0,
            };
            this.compare = options.compare || ((a,b) => (a-b));
        }

        take(n, src) { // return array of n lowest values from src
            var dst = [];
            if (src) {
                src.sort((a,b) => this.compare(b,a));
                while (dst.length < n && src.length) {
                    let v = src.pop();
                    (v != null) && dst.push(v);
                }
            }
            return dst;
        }

        fillBucket(i) {
            this.stats.fill++;
            var n = 0;
            if (0 <= i && i+1 < this.b.length) {
                if (this.b[i+1].length || this.fillBucket(i+1)) {
                    this.b[i] = this.take(this.sizes[i] || 1, this.b[i+1]);
                    if ((n = this.b[i].length)) {
                        this.bmax[i] = this.b[i][n-1];
                    }
                }
            }
            return n;
        }

        summarize(options={}) {
            var log = options.log || console.log;
            this.b.forEach((b,i) => {
                log("b["+i+"]", 
                    this.b[i].length, JSON.stringify(this.b[i].slice(0,10)), 
                    "bmax:"+this.bmax[i], 
                    "sizes:"+this.sizes[i]);
            });
        }

        validate(onErr = (()=>null)) {
            try {
                var len = this.b.reduce((acc,b) => acc + b.length,0);
                if (len !== this.length) {
                    throw new Error("length expected:"+this.length+" actual:", len);
                }
                this.b.forEach((b,i) => {
                    var max = b.reduce((acc,v) => (acc == null || this.compare(v, acc) > 0 ? v : acc), null);
                    if (max != null && max !== this.bmax[i]) {
                        throw new Error("bmax["+i+"] expected:"+max+" actual:"+this.bmax[i]);
                    }
                });
                this.bmax.forEach((m,i) => {
                    if (i > 0 && this.bmax[i] != null && this.bmax[i-1] > m) {
                        throw new Error("bmax not monotonic");
                    }
                });
            } catch(err) {
                console.log("=====", err.message, "=====");
                this.summarize();
                onErr(err);
                throw new Error(err);
            }
        }

        splitBucket(i) {
            if (this.b.length-1 <= i) {
                return null; // no more buckets
            }
            this.stats.split++;
            this.b[i].sort(this.compare);
            var bilen = this.b[i].length;
            var cut = mathjs.round(mathjs.min(this.b[i].length,this.split[i])/2);

            this.b[i+1] = this.b[i+1].concat(this.b[i].slice(cut));
            this.b[i] = this.b[i].slice(0, cut);
            this.bmax[i] = this.b[i][cut-1];
            this.bmax[i+1] == null && (this.bmax[i+1] = this.b[i+1][this.b[i+1].length-1]);

            if (this.split[i+1] && this.b[i+1].length > this.split[i+1]) {
                this.splitBucket(i+1);
            }
        }

        insert(value) {
            var nb = this.b.length;
            this.length++;
            for (var iBucket = 0; iBucket < nb; iBucket++) {
                var vmax = this.bmax[iBucket];
                if (vmax == null) {
                    this.b[iBucket].push(value);
                    this.bmax[iBucket] = value;
                    break;
                }
                if (this.compare(value,vmax) <= 0) {
                    this.b[iBucket].push(value);
                    break;
                }
            }
            if (iBucket === nb) { 
                this.bmax[nb-1] = value;
                this.b[nb-1].push(value);
            }
            if (this.split[0] && this.b[0].length > this.split[0]) {
                this.splitBucket(0);
            }
            var len = this.b.reduce((acc,b) => acc+b.length,0);
            if (len !== this.length) {
                throw new Error();
            }
            return this;
        }

        extractMin() {
            var min = null;
            if (this.length) {
                var b0new = [];
                for (var i=this.b[0].length; i-- > 0; ) {
                    var v = this.b[0][i];
                    if (min == null) {
                        min = v; // v can be null;
                    } else if (v != null) {
                        if (this.compare(v, min) < 0) {
                            b0new.push(min);
                            min = v;
                        } else {
                            b0new.push(v);
                        }
                    }
                }
                this.b[0] = b0new;
                if (min == null) {
                    return this.fillBucket(0) && this.extractMin();
                } 
                this.length--;
            }
            return min;
        }
    }

    module.exports = exports.PriorityQ = PriorityQ;
})(typeof exports === "object" ? exports : (exports = {}));

