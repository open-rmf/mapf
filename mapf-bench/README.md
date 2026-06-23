# `mapf-bench`

The sole purpose of this crate is to run benchmarks using the `mapf` library. Currently the benchmarks we
run are run against [moving AI](https://www.movingai.com/benchmarks/mapf/)'s benchmarks. This benchmark is often
seen as the defacto "mapf" banchmark. This crate contains abinary to load the benchmark files into memory and run
a single benchmark.

It is advised to use the python benchmark script located in the `scripts/` folder tot automate the downloading and
running of the benchmakrs.
