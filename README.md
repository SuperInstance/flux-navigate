# flux-navigate

> BFS pathfinding engine with waypoints, replanning, and obstacle avoidance for FLUX agents.

## What This Is

`flux-navigate` is a Rust crate implementing a **grid-based navigator** — it finds shortest paths on a 32×32 boolean grid using BFS, supports multi-waypoint routes, and dynamically replans when obstacles change.

## Role in the FLUX Ecosystem

Spatial reasoning is essential for agents operating in shared environments. `flux-navigate` provides the pathfinding brain:

- **`flux-compass`** provides heading vectors that navigate uses for orientation
- **`flux-perception`** fuses sensor data to update obstacle maps
- **`flux-simulator`** uses navigate for fleet vessel movement in simulations
- **`flux-dream-cycle`** can schedule navigation tasks as part of agent planning

## Key Features

| Feature | Description |
|---------|-------------|
| **BFS Pathfinding** | Shortest path on 32×32 grid with obstacle avoidance |
| **Dynamic Replanning** | `replan()` recalculates when obstacles block the current path |
| **Waypoint Routing** | Chain multiple waypoints for complex multi-stop routes |
| **Progress Tracking** | `progress()` returns 0.0–1.0 completion estimate |
| **Blocked Detection** | `blocked()` flag when no path exists to destination |
| **Step-by-Step** | `step()` advances one cell per call for integration with simulators |

## Quick Start

```rust
use flux_navigate::Navigator;

let mut nav = Navigator::new();

// Set up obstacles
let mut grid = [[false; 32]; 32];
grid[5][5] = true; // wall at (5,5)
nav.set_grid(&grid);

// Navigate with waypoints
nav.add_waypoint(10, 10);
nav.add_waypoint(20, 5);
nav.set_destination(0, 0).unwrap(); // final destination

// Step through the path
while !nav.at_destination() {
    match nav.step() {
        0 => break,      // arrived
        -1 => { /* blocked, no path */ break; }
        _ => {}           // still moving
    }
}

println!("Position: {:?}", nav.current());
println!("Progress: {:.0}%", nav.progress() * 100.0);
```

## Building & Testing

```bash
cargo build
cargo test
```

## Related Fleet Repos

- [`flux-compass`](https://github.com/SuperInstance/flux-compass) — Heading/orientation for movement direction
- [`flux-perception`](https://github.com/SuperInstance/flux-perception) — Sensor fusion for obstacle detection
- [`flux-simulator`](https://github.com/SuperInstance/flux-simulator) — Fleet simulation with vessel movement
- [`flux-dream-cycle`](https://github.com/SuperInstance/flux-dream-cycle) — Task scheduling for coordinated movement
- [`flux-evolve`](https://github.com/SuperInstance/flux-evolve) — Evolutionary optimization of navigation parameters

## License

Part of the [SuperInstance](https://github.com/SuperInstance) FLUX fleet.
