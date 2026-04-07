[![style](https://github.com/open-rmf/mapf/actions/workflows/style.yaml/badge.svg?branch=main)](https://github.com/open-rmf/mapf/actions/workflows/style.yaml)
[![ci](https://github.com/open-rmf/mapf/actions/workflows/ci.yaml/badge.svg?branch=main)](https://github.com/open-rmf/mapf/actions/workflows/ci.yaml)

# multi-agent (path finding) planning framework

Mapf is a (currently experimental) Rust library for multi-agent planning, with
a focus on cooperative path finding.

This repo is just getting started, so please bear with us for a while as we work
towards making useful tools.

This is being developed as part of the [Open-RMF](https://github.com/open-rmf)
project which provides an open source framework for enabling interoperability
between heterogeneous fleets of mobile robots, including cooperation across
different platforms and vendors.

# Helpful Links

* [Rust Book](https://doc.rust-lang.org/stable/book/)

# Install dependencies

## Ubuntu / MacOS / Linux / Unix

We tend to always target the latest versions of Rust, so we recommend using the `rustup` tool: https://www.rust-lang.org/tools/install

```bash
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

If you already have `rustup` then you can use this to bring your installation up to date:
```bash
$ rustup update
```

## Windows

Follow instructions for installing rustup-init.exe [here](https://forge.rust-lang.org/infra/other-installation-methods.html#other-ways-to-install-rustup).

# Run an example

From the root directory:

```bash
$ cargo run --release --example grid
```

This example will allow you to experiment with multiple agents cooperatively
planning from their starts to their goals in a grid environment. Use left/right
click to set the start/goal cells of each agent. Use shift+left/right click to
add/remove occupancy from cells. The velocities and sizes of each agent can be
customized independently, and you can save/load scenarios into yaml files.

Some premade scenarios can be found in the `mapf-viz/scenarios` folder. Load a
scenario on startup by passing the scenario name as an executable argument, e.g.:

```bash
$ cargo run --release --example grid -- mapf-viz/scenarios/sizes.yaml
```

Note that the scenario filename to load at startup must come after a `--` with a space on each side.
