Using the aggregation framework through global variables. The distributed graph does not
have any edges created explicitly.

Building data using ```parfil::tests::Case2``` with 1000 particles.

| # iterations| strong node time (s) | weak node time (s) | distributed system (s) |
|:-----------:|:--------------------:|:------------------:|:----------------------:|
| 1           | -                    | -                  | 0.15-0.5 (mean ~0.17)  |
| 100         | 0.05                 | 0.18               | 13-21                  |
| 1k          | 0.4                  | 1.5                | 156-170                |
