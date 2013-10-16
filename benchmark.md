Using the aggregation framework through global variables. The distributed graph does not
have any edges created explicitly. However, due to the use of the global variables, we
can think about it as if it were a complete graph.

Building data using ```parfil::tests::Case2``` with 1000 particles.

| # iterations| strong node time (s) | weak node time (s) | distributed system (s) |
|:-----------:|:--------------------:|:------------------:|:----------------------:|
| 100         | 0.05                 | 0.18               | 13-21                  |
| 1k          | 0.4                  | 1.5                | 156-170                |
