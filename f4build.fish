#!/usr/bin/fish
source ../../scripts/activate.fish

mkdir -p logs

set -x PYTHONUNBUFFERED 1
perf record -g --call-graph dwarf -o perf_route.data -- f4pga -v build --nocache --flow flow_basys3.json | tee logs/f4_run.log

#openFPGALoader -b basys3 ./build/basys3/*.bit
