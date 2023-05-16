#!/bin/bash
set -o verbose
set -o errexit
export CARGO_PROFILE_RELEASE_LTO=true
export CARGO_PROFILE_RELEASE_OPT_LEVEL=z
cargo build --example grid --target wasm32-unknown-unknown --release
RUST_BACKTRACE=full wasm-bindgen --target web --out-dir web target/wasm32-unknown-unknown/release/examples/grid.wasm
cd web
wasm-opt -Oz -o libgrid_bg_optimized.wasm libgrid_bg.wasm
