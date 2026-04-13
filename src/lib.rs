use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};
use std::cmp::{Ordering, Reverse};

// ============================================================================
// Core types
// ============================================================================

/// A 2D point with integer coordinates.
#[derive(Clone, Debug, Copy, PartialEq, Eq, Hash)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Point { x, y }
    }

    /// Euclidean distance to another point.
    pub fn distance_to(self, other: Point) -> f64 {
        let dx = (self.x - other.x) as f64;
        let dy = (self.y - other.y) as f64;
        (dx * dx + dy * dy).sqrt()
    }

    /// Manhattan distance to another point.
    pub fn manhattan_to(self, other: Point) -> i32 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }
}

// ============================================================================
// Graph data structure with weighted edges
// ============================================================================

/// An edge connecting two nodes in a weighted graph.
#[derive(Clone, Debug, Copy, PartialEq)]
pub struct Edge {
    pub from: usize,
    pub to: usize,
    pub weight: f64,
}

/// A weighted directed graph backed by an adjacency list.
#[derive(Clone, Debug)]
pub struct Graph {
    nodes: Vec<String>,
    adjacency: HashMap<usize, Vec<(usize, f64)>>,
    node_index: HashMap<String, usize>,
    edge_count: usize,
}

impl Graph {
    /// Create a new empty graph.
    pub fn new() -> Self {
        Graph {
            nodes: Vec::new(),
            adjacency: HashMap::new(),
            node_index: HashMap::new(),
            edge_count: 0,
        }
    }

    /// Add a named node. Returns the node index.
    /// If the node already exists, returns its existing index.
    pub fn add_node(&mut self, name: &str) -> usize {
        if let Some(&idx) = self.node_index.get(name) {
            return idx;
        }
        let idx = self.nodes.len();
        self.nodes.push(name.to_string());
        self.node_index.insert(name.to_string(), idx);
        self.adjacency.insert(idx, Vec::new());
        idx
    }

    /// Get the number of nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Get the number of edges.
    pub fn edge_count(&self) -> usize {
        self.edge_count
    }

    /// Get a node name by index.
    pub fn node_name(&self, idx: usize) -> Option<&str> {
        self.nodes.get(idx).map(|s| s.as_str())
    }

    /// Get a node index by name.
    pub fn node_index(&self, name: &str) -> Option<usize> {
        self.node_index.get(name).copied()
    }

    /// Add a directed weighted edge. Returns false if nodes don't exist.
    pub fn add_edge(&mut self, from: &str, to: &str, weight: f64) -> bool {
        let from_idx = match self.node_index.get(from) {
            Some(&idx) => idx,
            None => return false,
        };
        let to_idx = match self.node_index.get(to) {
            Some(&idx) => idx,
            None => return false,
        };
        self.add_edge_by_index(from_idx, to_idx, weight);
        true
    }

    /// Add a directed weighted edge by node indices.
    pub fn add_edge_by_index(&mut self, from: usize, to: usize, weight: f64) {
        self.adjacency.entry(from).or_default().push((to, weight));
        self.edge_count += 1;
    }

    /// Add a bidirectional edge (both directions with the same weight).
    pub fn add_edge_undirected(&mut self, from: &str, to: &str, weight: f64) -> bool {
        if !self.add_edge(from, to, weight) {
            return false;
        }
        self.add_edge(to, from, weight)
    }

    /// Get neighbors of a node by index.
    pub fn neighbors(&self, idx: usize) -> &[(usize, f64)] {
        self.adjacency.get(&idx).map(|v| v.as_slice()).unwrap_or(&[])
    }

    /// Get all node names.
    pub fn node_names(&self) -> &[String] {
        &self.nodes
    }

    /// Remove all edges (keeps nodes).
    pub fn clear_edges(&mut self) {
        for adj in self.adjacency.values_mut() {
            adj.clear();
        }
        self.edge_count = 0;
    }
}

impl Default for Graph {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Waypoint system
// ============================================================================

/// A named waypoint in a navigation graph.
#[derive(Clone, Debug, PartialEq)]
pub struct Waypoint {
    pub name: String,
    pub position: Point,
    pub index: usize,
}

/// A collection of named waypoints with graph connectivity.
#[derive(Clone, Debug)]
pub struct WaypointGraph {
    waypoints: Vec<Waypoint>,
    graph: Graph,
    name_map: HashMap<String, usize>,
}

impl WaypointGraph {
    pub fn new() -> Self {
        WaypointGraph {
            waypoints: Vec::new(),
            graph: Graph::new(),
            name_map: HashMap::new(),
        }
    }

    /// Add a waypoint. Returns its index.
    pub fn add_waypoint(&mut self, name: &str, x: i32, y: i32) -> usize {
        let idx = self.graph.add_node(name);
        if idx >= self.waypoints.len() {
            self.waypoints.push(Waypoint {
                name: name.to_string(),
                position: Point::new(x, y),
                index: idx,
            });
            self.name_map.insert(name.to_string(), idx);
        }
        idx
    }

    /// Connect two waypoints with a weighted edge.
    pub fn connect(&mut self, from: &str, to: &str, weight: Option<f64>) -> bool {
        let w = weight.unwrap_or_else(|| {
            let a = self.waypoints[self.name_map[from]].position;
            let b = self.waypoints[self.name_map[to]].position;
            a.distance_to(b)
        });
        self.graph.add_edge(from, to, w)
    }

    /// Connect two waypoints bidirectionally.
    pub fn connect_bidirectional(&mut self, from: &str, to: &str, weight: Option<f64>) -> bool {
        let w = weight.unwrap_or_else(|| {
            let a = self.waypoints[self.name_map[from]].position;
            let b = self.waypoints[self.name_map[to]].position;
            a.distance_to(b)
        });
        self.graph.add_edge_undirected(from, to, w)
    }

    /// Get a waypoint by name.
    pub fn get_waypoint(&self, name: &str) -> Option<&Waypoint> {
        self.name_map.get(name).map(|&idx| &self.waypoints[idx])
    }

    /// Get the underlying graph.
    pub fn graph(&self) -> &Graph {
        &self.graph
    }

    /// Find the shortest path between named waypoints using Dijkstra.
    pub fn shortest_path(&self, from: &str, to: &str) -> Option<(Vec<String>, f64)> {
        let from_idx = *self.name_map.get(from)?;
        let to_idx = *self.name_map.get(to)?;
        let (indices, cost) = dijkstra(&self.graph, from_idx, to_idx)?;
        let names: Vec<String> = indices.iter().map(|&i| self.graph.node_name(i).unwrap().to_string()).collect();
        Some((names, cost))
    }

    /// Find the shortest path using A* with a heuristic based on waypoint positions.
    pub fn shortest_path_astar(&self, from: &str, to: &str) -> Option<(Vec<String>, f64)> {
        let from_idx = *self.name_map.get(from)?;
        let to_idx = *self.name_map.get(to)?;
        let goal_pos = self.waypoints[to_idx].position;
        let heuristic = |idx: usize| -> f64 {
            self.waypoints[idx].position.distance_to(goal_pos)
        };
        let (indices, cost) = astar(&self.graph, from_idx, to_idx, heuristic)?;
        let names: Vec<String> = indices.iter().map(|&i| self.graph.node_name(i).unwrap().to_string()).collect();
        Some((names, cost))
    }

    /// Optimize a multi-stop route visiting all specified waypoints in the best order.
    pub fn optimize_route(&self, stops: &[&str]) -> Option<(Vec<String>, f64)> {
        if stops.is_empty() {
            return None;
        }
        if stops.len() == 1 {
            return Some((vec![stops[0].to_string()], 0.0));
        }
        if stops.len() == 2 {
            let (path, cost) = self.shortest_path(stops[0], stops[1])?;
            return Some((path, cost));
        }

        // For larger sets, use nearest-neighbor heuristic for TSP-like optimization
        let remaining: Vec<usize> = stops.iter()
            .filter_map(|s| self.name_map.get(*s).copied())
            .collect();
        if remaining.len() != stops.len() {
            return None; // some stop not found
        }

        let mut best_route: Vec<String> = Vec::new();
        let mut best_cost = f64::INFINITY;

        // Try each node as starting point
        for start in 0..remaining.len() {
            let mut route = Vec::new();
            let mut unvisited: HashSet<usize> = remaining.iter().copied().collect();
            let mut current = remaining[start];
            unvisited.remove(&current);
            route.push(self.graph.node_name(current).unwrap().to_string());
            let mut total_cost = 0.0;
            let mut valid = true;

            while !unvisited.is_empty() {
                // Find nearest unvisited
                let mut nearest_idx = *unvisited.iter().next().unwrap();
                let mut nearest_cost = f64::INFINITY;
                for &u in &unvisited {
                    let c = self.graph.neighbors(current)
                        .iter()
                        .filter(|(n, _)| *n == u)
                        .map(|(_, w)| *w)
                        .next();
                    if let Some(c) = c {
                        if c < nearest_cost {
                            nearest_cost = c;
                            nearest_idx = u;
                        }
                    }
                }
                if nearest_cost == f64::INFINITY {
                    valid = false;
                    break;
                }
                total_cost += nearest_cost;
                current = nearest_idx;
                unvisited.remove(&current);
                route.push(self.graph.node_name(current).unwrap().to_string());
            }

            if valid && total_cost < best_cost {
                best_cost = total_cost;
                best_route = route;
            }
        }

        if best_cost < f64::INFINITY {
            Some((best_route, best_cost))
        } else {
            None
        }
    }

    /// Get all waypoint names.
    pub fn waypoint_names(&self) -> Vec<&str> {
        self.waypoints.iter().map(|w| w.name.as_str()).collect()
    }
}

impl Default for WaypointGraph {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Pathfinding algorithms
// ============================================================================

/// Result of a pathfinding query.
#[derive(Clone, Debug, PartialEq)]
pub struct PathResult {
    pub path: Vec<usize>,
    pub cost: f64,
    pub visited_count: usize,
}

// -- Dijkstra's Algorithm --

/// Run Dijkstra's shortest path algorithm on a weighted graph.
/// Returns (path as node indices, total cost).
pub fn dijkstra(graph: &Graph, start: usize, goal: usize) -> Option<(Vec<usize>, f64)> {
    if start == goal {
        return Some((vec![start], 0.0));
    }

    let mut dist: HashMap<usize, f64> = HashMap::new();
    let mut prev: HashMap<usize, usize> = HashMap::new();
    let mut visited = HashSet::new();

    dist.insert(start, 0.0);

    // Min-heap: (cost, node)
    let mut heap = BinaryHeap::new();
    heap.push(Reverse(OrderedFloat(0.0, start)));

    let mut visited_count = 0;

    while let Some(Reverse(OrderedFloat(d, u))) = heap.pop() {
        if visited.contains(&u) {
            continue;
        }
        visited.insert(u);
        visited_count += 1;

        if u == goal {
            // Reconstruct path
            let mut path = vec![goal];
            let mut node = goal;
            while let Some(&p) = prev.get(&node) {
                path.push(p);
                node = p;
            }
            path.reverse();
            return Some((path, d));
        }

        for &(neighbor, weight) in graph.neighbors(u) {
            if visited.contains(&neighbor) {
                continue;
            }
            let new_dist = d + weight;
            let current = dist.get(&neighbor).copied().unwrap_or(f64::INFINITY);
            if new_dist < current {
                dist.insert(neighbor, new_dist);
                prev.insert(neighbor, u);
                heap.push(Reverse(OrderedFloat(new_dist, neighbor)));
            }
        }
    }

    None
}

// -- A* Algorithm --

/// Run A* shortest path algorithm on a weighted graph with a heuristic function.
/// The heuristic `h(node)` should estimate the cost from `node` to the goal.
/// Returns (path as node indices, total cost).
pub fn astar<F>(graph: &Graph, start: usize, goal: usize, heuristic: F) -> Option<(Vec<usize>, f64)>
where
    F: Fn(usize) -> f64,
{
    if start == goal {
        return Some((vec![start], 0.0));
    }

    // f(n) = g(n) + h(n)
    let mut g_score: HashMap<usize, f64> = HashMap::new();
    let mut f_score: HashMap<usize, f64> = HashMap::new();
    let mut prev: HashMap<usize, usize> = HashMap::new();
    let mut visited = HashSet::new();

    g_score.insert(start, 0.0);
    f_score.insert(start, heuristic(start));

    let mut heap = BinaryHeap::new();
    heap.push(Reverse(OrderedFloat(heuristic(start), start)));

    while let Some(Reverse(OrderedFloat(f, u))) = heap.pop() {
        if visited.contains(&u) {
            continue;
        }
        visited.insert(u);

        if u == goal {
            let mut path = vec![goal];
            let mut node = goal;
            while let Some(&p) = prev.get(&node) {
                path.push(p);
                node = p;
            }
            path.reverse();
            return Some((path, *g_score.get(&goal).unwrap()));
        }

        let current_g = g_score.get(&u).copied().unwrap_or(f64::INFINITY);

        for &(neighbor, weight) in graph.neighbors(u) {
            if visited.contains(&neighbor) {
                continue;
            }
            let tentative_g = current_g + weight;
            let current_g_n = g_score.get(&neighbor).copied().unwrap_or(f64::INFINITY);
            if tentative_g < current_g_n {
                prev.insert(neighbor, u);
                g_score.insert(neighbor, tentative_g);
                let f = tentative_g + heuristic(neighbor);
                f_score.insert(neighbor, f);
                heap.push(Reverse(OrderedFloat(f, neighbor)));
            }
        }
    }

    None
}

// -- BFS (unweighted) --

/// Breadth-first search on a weighted graph (ignores weights).
/// Returns the shortest path in terms of hop count.
pub fn bfs(graph: &Graph, start: usize, goal: usize) -> Option<Vec<usize>> {
    if start == goal {
        return Some(vec![start]);
    }

    let mut visited = HashSet::new();
    let mut prev: HashMap<usize, usize> = HashMap::new();
    let mut queue = VecDeque::new();

    visited.insert(start);
    queue.push_back(start);

    while let Some(u) = queue.pop_front() {
        if u == goal {
            let mut path = vec![goal];
            let mut node = goal;
            while let Some(&p) = prev.get(&node) {
                path.push(p);
                node = p;
            }
            path.reverse();
            return Some(path);
        }

        for &(neighbor, _) in graph.neighbors(u) {
            if visited.insert(neighbor) {
                prev.insert(neighbor, u);
                queue.push_back(neighbor);
            }
        }
    }

    None
}

// -- DFS --

/// Depth-first search on a weighted graph (ignores weights).
/// Returns a path from start to goal if one exists (not necessarily shortest).
pub fn dfs(graph: &Graph, start: usize, goal: usize) -> Option<Vec<usize>> {
    if start == goal {
        return Some(vec![start]);
    }

    let mut visited = HashSet::new();
    let mut stack: Vec<(usize, Vec<usize>)> = vec![(start, vec![start])];

    while let Some((node, path)) = stack.pop() {
        if node == goal {
            return Some(path);
        }
        if !visited.insert(node) {
            continue;
        }
        for &(neighbor, _) in graph.neighbors(node) {
            if !visited.contains(&neighbor) {
                let mut new_path = path.clone();
                new_path.push(neighbor);
                stack.push((neighbor, new_path));
            }
        }
    }

    None
}

// ============================================================================
// Route optimization
// ============================================================================

/// Route optimization result.
#[derive(Clone, Debug)]
pub struct RouteResult {
    pub path: Vec<String>,
    pub total_cost: f64,
    pub hop_count: usize,
}

/// Find the least-cost route between two named nodes in a graph.
pub fn least_cost_route(graph: &Graph, from: &str, to: &str) -> Option<RouteResult> {
    let from_idx = graph.node_index(from)?;
    let to_idx = graph.node_index(to)?;
    let (indices, cost) = dijkstra(graph, from_idx, to_idx)?;
    let path: Vec<String> = indices.iter()
        .map(|&i| graph.node_name(i).unwrap().to_string())
        .collect();
    let hop_count = path.len().saturating_sub(1);
    Some(RouteResult { path, total_cost: cost, hop_count })
}

/// Find the minimum-hop route between two named nodes (ignores edge weights).
pub fn min_hop_route(graph: &Graph, from: &str, to: &str) -> Option<RouteResult> {
    let from_idx = graph.node_index(from)?;
    let to_idx = graph.node_index(to)?;
    let indices = bfs(graph, from_idx, to_idx)?;
    let path: Vec<String> = indices.iter()
        .map(|&i| graph.node_name(i).unwrap().to_string())
        .collect();
    let hop_count = path.len().saturating_sub(1);
    Some(RouteResult { path, total_cost: hop_count as f64, hop_count })
}

// ============================================================================
// Obstacle avoidance
// ============================================================================

/// An obstacle in 2D space, represented as an axis-aligned bounding box.
#[derive(Clone, Debug, Copy, PartialEq)]
pub struct Obstacle {
    pub x: i32,
    pub y: i32,
    pub width: u32,
    pub height: u32,
}

impl Obstacle {
    pub fn new(x: i32, y: i32, width: u32, height: u32) -> Self {
        Obstacle { x, y, width, height }
    }

    /// Check if a point is inside this obstacle.
    pub fn contains(&self, p: Point) -> bool {
        p.x >= self.x && p.x < self.x + self.width as i32
            && p.y >= self.y && p.y < self.y + self.height as i32
    }

    /// Check if a line segment from `a` to `b` intersects this obstacle.
    pub fn intersects_line(&self, a: Point, b: Point) -> bool {
        // Sample points along the line segment
        let dist = a.distance_to(b);
        if dist < 1e-6 {
            return self.contains(a);
        }
        let steps = (dist.ceil() as usize).max(10).min(100);
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let px = a.x as f64 + (b.x - a.x) as f64 * t;
            let py = a.y as f64 + (b.y - a.y) as f64 * t;
            if self.contains(Point::new(px as i32, py as i32)) {
                return true;
            }
        }
        false
    }
}

/// A collection of obstacles for collision checking.
#[derive(Clone, Debug, Default)]
pub struct ObstacleMap {
    obstacles: Vec<Obstacle>,
    grid_width: i32,
    grid_height: i32,
}

impl ObstacleMap {
    pub fn new(grid_width: i32, grid_height: i32) -> Self {
        ObstacleMap {
            obstacles: Vec::new(),
            grid_width,
            grid_height,
        }
    }

    pub fn add_obstacle(&mut self, obstacle: Obstacle) {
        self.obstacles.push(obstacle);
    }

    /// Check if a point is blocked by any obstacle.
    pub fn is_blocked(&self, p: Point) -> bool {
        if p.x < 0 || p.x >= self.grid_width || p.y < 0 || p.y >= self.grid_height {
            return true;
        }
        self.obstacles.iter().any(|o| o.contains(p))
    }

    /// Check if movement from `a` to `b` is blocked by any obstacle.
    pub fn is_path_blocked(&self, a: Point, b: Point) -> bool {
        self.obstacles.iter().any(|o| o.intersects_line(a, b))
    }

    /// Generate a grid representation for the Navigator.
    pub fn to_grid<const SIZE: usize>(&self) -> [[bool; SIZE]; SIZE] {
        let mut grid = [[false; SIZE]; SIZE];
        for x in 0..SIZE {
            for y in 0..SIZE {
                if self.is_blocked(Point::new(x as i32, y as i32)) {
                    grid[x][y] = true;
                }
            }
        }
        grid
    }

    /// Find a detour point to route around obstacles between `a` and `b`.
    pub fn find_detour(&self, a: Point, b: Point, step: i32) -> Option<Point> {
        let mid_x = ((a.x + b.x) / 2) as i32;
        let mid_y = ((a.y + b.y) / 2) as i32;

        // Try offsets in expanding radius
        for radius in (1..=10).step_by(2) {
            for dx in -radius..=radius {
                for dy in -radius..=radius {
                    if (dx as i32).abs() + (dy as i32).abs() != radius as i32 {
                        continue;
                    }
                    let candidate = Point::new(mid_x + dx * step, mid_y + dy * step);
                    if !self.is_blocked(candidate) && !self.is_path_blocked(a, candidate)
                        && !self.is_path_blocked(candidate, b) {
                        return Some(candidate);
                    }
                }
            }
        }
        None
    }
}

// ============================================================================
// Navigation mesh support
// ============================================================================

/// A triangle in a navigation mesh.
#[derive(Clone, Debug, Copy, PartialEq)]
pub struct NavTriangle {
    pub vertices: [Point; 3],
    pub center: Point,
    pub id: usize,
}

impl NavTriangle {
    pub fn new(v0: Point, v1: Point, v2: Point, id: usize) -> Self {
        let cx = (v0.x + v1.x + v2.x) / 3;
        let cy = (v0.y + v1.y + v2.y) / 3;
        NavTriangle {
            vertices: [v0, v1, v2],
            center: Point::new(cx, cy),
            id,
        }
    }

    /// Check if a point is inside this triangle using barycentric coordinates.
    pub fn contains_point(&self, p: Point) -> bool {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];

        let d00 = (v1.x - v0.x) as f64 * (v1.x - v0.x) as f64 + (v1.y - v0.y) as f64 * (v1.y - v0.y) as f64;
        let d01 = (v1.x - v0.x) as f64 * (v2.x - v0.x) as f64 + (v1.y - v0.y) as f64 * (v2.y - v0.y) as f64;
        let d11 = (v2.x - v0.x) as f64 * (v2.x - v0.x) as f64 + (v2.y - v0.y) as f64 * (v2.y - v0.y) as f64;
        let d20 = (p.x - v0.x) as f64 * (v1.x - v0.x) as f64 + (p.y - v0.y) as f64 * (v1.y - v0.y) as f64;
        let d21 = (p.x - v0.x) as f64 * (v2.x - v0.x) as f64 + (p.y - v0.y) as f64 * (v2.y - v0.y) as f64;

        let denom = d00 * d11 - d01 * d01;
        if denom.abs() < 1e-10 {
            return false;
        }

        let u = (d11 * d20 - d01 * d21) / denom;
        let v = (d00 * d21 - d01 * d20) / denom;

        u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
    }

    /// Area of the triangle.
    pub fn area(&self) -> f64 {
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        ((v1.x - v0.x) as f64 * (v2.y - v0.y) as f64
            - (v2.x - v0.x) as f64 * (v1.y - v0.y) as f64)
            .abs() / 2.0
    }
}

/// A navigation mesh composed of triangles.
#[derive(Clone, Debug)]
pub struct NavMesh {
    triangles: Vec<NavTriangle>,
    adjacency: HashMap<usize, Vec<(usize, f64)>>,
}

impl NavMesh {
    pub fn new() -> Self {
        NavMesh {
            triangles: Vec::new(),
            adjacency: HashMap::new(),
        }
    }

    /// Add a triangle to the mesh.
    pub fn add_triangle(&mut self, v0: Point, v1: Point, v2: Point) -> usize {
        let id = self.triangles.len();
        let tri = NavTriangle::new(v0, v1, v2, id);
        self.triangles.push(tri);

        // Check adjacency with existing triangles (shared edge)
        let new_verts: Vec<(i32, i32)> = tri.vertices.iter().map(|v| (v.x, v.y)).collect();
        for existing in &self.triangles {
            if existing.id == id {
                continue;
            }
            let existing_verts: Vec<(i32, i32)> = existing.vertices.iter().map(|v| (v.x, v.y)).collect();
            let shared = new_verts.iter()
                .filter(|v| existing_verts.contains(v))
                .count();
            if shared >= 2 {
                let cost = tri.center.distance_to(existing.center);
                self.adjacency.entry(id).or_default().push((existing.id, cost));
                self.adjacency.entry(existing.id).or_default().push((id, cost));
            }
        }

        id
    }

    /// Find which triangle contains a point.
    pub fn find_triangle(&self, p: Point) -> Option<&NavTriangle> {
        self.triangles.iter().find(|t| t.contains_point(p))
    }

    /// Build a graph from the nav mesh for pathfinding.
    pub fn to_graph(&self) -> Graph {
        let mut graph = Graph::new();
        for tri in &self.triangles {
            graph.add_node(&format!("tri_{}", tri.id));
        }
        for (&id, neighbors) in &self.adjacency {
            for &(neighbor, cost) in neighbors {
                graph.add_edge_by_index(id, neighbor, cost);
            }
        }
        graph
    }

    /// Find the shortest path between two points in the nav mesh.
    pub fn find_path(&self, from: Point, to: Point) -> Option<(Vec<usize>, f64)> {
        let from_tri = self.find_triangle(from)?;
        let to_tri = self.find_triangle(to)?;

        if from_tri.id == to_tri.id {
            return Some((vec![from_tri.id], from.distance_to(to)));
        }

        let graph = self.to_graph();
        let (path, cost) = dijkstra(&graph, from_tri.id, to_tri.id)?;
        Some((path, cost))
    }

    /// Get the number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Get triangle by id.
    pub fn get_triangle(&self, id: usize) -> Option<&NavTriangle> {
        self.triangles.get(id)
    }
}

impl Default for NavMesh {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Helper: OrderedFloat for BinaryHeap
// ============================================================================

#[derive(Clone, Copy, PartialEq, Debug)]
struct OrderedFloat(f64, usize);

impl Eq for OrderedFloat {}

impl PartialOrd for OrderedFloat {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrderedFloat {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap_or(Ordering::Equal)
            .then(self.1.cmp(&other.1))
    }
}

// ============================================================================
// Original Navigator (preserved for backward compatibility)
// ============================================================================

pub struct Navigator {
    grid: [[bool; 32]; 32],
    waypoints: Vec<Point>,
    pos: Point,
    dest: Point,
    path: Vec<Point>,
    path_idx: usize,
    navigating: bool,
    current_wp: usize,
}

impl Navigator {
    pub fn new() -> Self {
        Self {
            grid: [[false; 32]; 32],
            waypoints: Vec::new(),
            pos: Point { x: 0, y: 0 },
            dest: Point { x: 0, y: 0 },
            path: Vec::new(),
            path_idx: 0,
            navigating: false,
            current_wp: 0,
        }
    }

    pub fn set_grid(&mut self, g: &[[bool; 32]; 32]) {
        self.grid = *g;
    }

    pub fn set_destination(&mut self, x: i32, y: i32) -> bool {
        if x < 0 || x >= 32 || y < 0 || y >= 32 {
            return false;
        }
        if self.grid[x as usize][y as usize] {
            return false;
        }
        self.dest = Point { x, y };
        self.current_wp = 0;
        if self.pos == self.dest && self.waypoints.is_empty() {
            self.navigating = false;
            self.path.clear();
            return true;
        }
        self.navigating = true;
        self.replan()
    }

    fn effective_dest(&self) -> Point {
        if self.current_wp < self.waypoints.len() {
            self.waypoints[self.current_wp]
        } else {
            self.dest
        }
    }

    fn bfs(&self, start: Point, goal: Point) -> Option<Vec<Point>> {
        if start == goal {
            return Some(vec![]);
        }
        if goal.x < 0 || goal.x >= 32 || goal.y < 0 || goal.y >= 32 {
            return None;
        }
        if self.grid[goal.x as usize][goal.y as usize] {
            return None;
        }

        let mut visited = [[false; 32]; 32];
        let mut parent: [[Option<Point>; 32]; 32] = [[None; 32]; 32];
        let mut queue = VecDeque::new();
        queue.push_back(start);
        visited[start.x as usize][start.y as usize] = true;

        let dirs: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];

        while let Some(p) = queue.pop_front() {
            if p == goal {
                let mut path = Vec::new();
                let mut cur = goal;
                while cur != start {
                    path.push(cur);
                    cur = parent[cur.x as usize][cur.y as usize].unwrap();
                }
                path.reverse();
                return Some(path);
            }
            for (dx, dy) in &dirs {
                let nx = p.x + dx;
                let ny = p.y + dy;
                if nx >= 0 && nx < 32 && ny >= 0 && ny < 32
                    && !visited[nx as usize][ny as usize]
                    && !self.grid[nx as usize][ny as usize]
                {
                    visited[nx as usize][ny as usize] = true;
                    parent[nx as usize][ny as usize] = Some(p);
                    queue.push_back(Point { x: nx, y: ny });
                }
            }
        }
        None
    }

    pub fn replan(&mut self) -> bool {
        if !self.navigating {
            return false;
        }
        let goal = self.effective_dest();
        match self.bfs(self.pos, goal) {
            Some(p) => {
                self.path = p;
                self.path_idx = 0;
                true
            }
            None => {
                self.path.clear();
                self.path_idx = 0;
                false
            }
        }
    }

    pub fn step(&mut self) -> i32 {
        if !self.navigating {
            return 0;
        }
        if self.pos == self.effective_dest() {
            if self.current_wp < self.waypoints.len() {
                self.current_wp += 1;
                if !self.replan() {
                    return -1;
                }
                if self.path.is_empty() {
                    return 0;
                }
            } else {
                self.navigating = false;
                self.path.clear();
                return 0;
            }
        }

        if self.path_idx >= self.path.len() {
            if !self.replan() {
                return -1;
            }
            if self.path.is_empty() {
                return 0;
            }
        }

        let next = self.path[self.path_idx];
        if self.grid[next.x as usize][next.y as usize] {
            if !self.replan() {
                return -1;
            }
            if self.path.is_empty() {
                return 0;
            }
            return self.step();
        }

        self.pos = next;
        self.path_idx += 1;

        if self.pos == self.effective_dest() {
            if self.current_wp >= self.waypoints.len() {
                self.navigating = false;
                self.path.clear();
            }
            0
        } else {
            1
        }
    }

    pub fn add_waypoint(&mut self, x: i32, y: i32) {
        self.waypoints.push(Point { x, y });
    }

    pub fn clear_waypoints(&mut self) {
        self.waypoints.clear();
        self.current_wp = 0;
    }

    pub fn current(&self) -> Point {
        self.pos
    }

    pub fn next_waypoint(&self) -> Point {
        if self.current_wp < self.waypoints.len() {
            self.waypoints[self.current_wp]
        } else {
            self.dest
        }
    }

    pub fn at_destination(&self) -> bool {
        self.pos == self.dest && !self.navigating
    }

    pub fn progress(&self) -> f64 {
        if self.pos == self.dest {
            return 1.0;
        }
        let total = (self.pos.x - self.dest.x).unsigned_abs() + (self.pos.y - self.dest.y).unsigned_abs();
        if total == 0 {
            return 1.0;
        }
        let init = ((self.pos.x - self.dest.x).unsigned_abs()
            + (self.pos.y - self.dest.y).unsigned_abs()) as f64;
        let curr = ((self.pos.x - self.dest.x).unsigned_abs() + (self.pos.y - self.dest.y).unsigned_abs()) as f64;
        if init == 0.0 {
            return 1.0;
        }
        (1.0 - curr / init).max(0.0)
    }

    pub fn blocked(&self) -> bool {
        self.path.is_empty() && self.navigating
    }
}

impl Default for Navigator {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- Point tests --
    #[test]
    fn test_point_distance() {
        let a = Point::new(0, 0);
        let b = Point::new(3, 4);
        assert!((a.distance_to(b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_point_manhattan() {
        let a = Point::new(1, 2);
        let b = Point::new(4, 6);
        assert_eq!(a.manhattan_to(b), 7);
    }

    // -- Graph tests --
    #[test]
    fn test_graph_add_nodes() {
        let mut g = Graph::new();
        let a = g.add_node("A");
        let b = g.add_node("B");
        assert_eq!(g.node_count(), 2);
        assert_eq!(a, 0);
        assert_eq!(b, 1);
    }

    #[test]
    fn test_graph_add_duplicate_node() {
        let mut g = Graph::new();
        let a = g.add_node("A");
        let a2 = g.add_node("A");
        assert_eq!(a, a2);
        assert_eq!(g.node_count(), 1);
    }

    #[test]
    fn test_graph_add_edges() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        assert!(g.add_edge("A", "B", 1.0));
        assert!(g.add_edge("B", "C", 2.0));
        assert_eq!(g.edge_count(), 2);
        assert_eq!(g.neighbors(0).len(), 1);
    }

    #[test]
    fn test_graph_undirected_edge() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        assert!(g.add_edge_undirected("A", "B", 5.0));
        assert_eq!(g.edge_count(), 2);
    }

    #[test]
    fn test_graph_edge_nonexistent_node() {
        let mut g = Graph::new();
        g.add_node("A");
        assert!(!g.add_edge("A", "B", 1.0));
    }

    #[test]
    fn test_graph_clear_edges() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_edge("A", "B", 1.0);
        g.clear_edges();
        assert_eq!(g.edge_count(), 0);
        assert_eq!(g.node_count(), 2);
    }

    // -- Dijkstra tests --
    #[test]
    fn test_dijkstra_simple() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "B", 1.0);
        g.add_edge("B", "C", 2.0);
        let (path, cost) = dijkstra(&g, 0, 2).unwrap();
        assert_eq!(path, vec![0, 1, 2]);
        assert!((cost - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_dijkstra_same_node() {
        let mut g = Graph::new();
        g.add_node("A");
        let (path, cost) = dijkstra(&g, 0, 0).unwrap();
        assert_eq!(path, vec![0]);
        assert!((cost - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_dijkstra_no_path() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        let result = dijkstra(&g, 0, 1);
        assert!(result.is_none());
    }

    #[test]
    fn test_dijkstra_chooses_shortest() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_node("D");
        // Direct long path A -> D
        g.add_edge("A", "D", 10.0);
        // Shorter path A -> B -> C -> D
        g.add_edge("A", "B", 1.0);
        g.add_edge("B", "C", 1.0);
        g.add_edge("C", "D", 1.0);
        let (path, cost) = dijkstra(&g, 0, 3).unwrap();
        assert_eq!(path, vec![0, 1, 2, 3]);
        assert!((cost - 3.0).abs() < 1e-10);
    }

    // -- A* tests --
    #[test]
    fn test_astar_simple() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "B", 3.0);
        g.add_edge("B", "C", 4.0);
        let heuristic = |_n: usize| -> f64 { 0.0 }; // no heuristic = Dijkstra
        let (path, cost) = astar(&g, 0, 2, heuristic).unwrap();
        assert_eq!(path, vec![0, 1, 2]);
        assert!((cost - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_astar_with_heuristic() {
        let mut g = Graph::new();
        // 5 nodes in a line: 0(0,0), 1(1,0), 2(2,0), 3(3,0), 4(4,0)
        for i in 0..5 {
            g.add_node(&i.to_string());
        }
        g.add_edge("0", "1", 1.0);
        g.add_edge("1", "2", 1.0);
        g.add_edge("2", "3", 1.0);
        g.add_edge("3", "4", 1.0);

        let goal_x = 4;
        let heuristic = move |n: usize| -> f64 { (goal_x - n) as f64 };
        let (path, cost) = astar(&g, 0, 4, heuristic).unwrap();
        assert_eq!(path, vec![0, 1, 2, 3, 4]);
        assert!((cost - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_astar_no_path() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        let heuristic = |_n: usize| -> f64 { 0.0 };
        let result = astar(&g, 0, 1, heuristic);
        assert!(result.is_none());
    }

    // -- BFS tests --
    #[test]
    fn test_bfs_graph_simple() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "B", 5.0); // weight ignored by BFS
        g.add_edge("B", "C", 10.0);
        let path = bfs(&g, 0, 2).unwrap();
        assert_eq!(path, vec![0, 1, 2]);
    }

    #[test]
    fn test_bfs_graph_no_path() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        let result = bfs(&g, 0, 1);
        assert!(result.is_none());
    }

    #[test]
    fn test_bfs_same_node() {
        let mut g = Graph::new();
        g.add_node("A");
        let path = bfs(&g, 0, 0).unwrap();
        assert_eq!(path, vec![0]);
    }

    // -- DFS tests --
    #[test]
    fn test_dfs_simple() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "B", 1.0);
        g.add_edge("B", "C", 1.0);
        let path = dfs(&g, 0, 2).unwrap();
        assert_eq!(path[0], 0);
        assert_eq!(path[path.len() - 1], 2);
    }

    #[test]
    fn test_dfs_no_path() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        let result = dfs(&g, 0, 1);
        assert!(result.is_none());
    }

    #[test]
    fn test_dfs_same_node() {
        let mut g = Graph::new();
        g.add_node("A");
        let path = dfs(&g, 0, 0).unwrap();
        assert_eq!(path, vec![0]);
    }

    // -- WaypointGraph tests --
    #[test]
    fn test_waypoint_graph_add() {
        let mut wg = WaypointGraph::new();
        wg.add_waypoint("start", 0, 0);
        wg.add_waypoint("end", 10, 10);
        assert_eq!(wg.waypoint_names().len(), 2);
    }

    #[test]
    fn test_waypoint_graph_shortest_path() {
        let mut wg = WaypointGraph::new();
        wg.add_waypoint("A", 0, 0);
        wg.add_waypoint("B", 3, 0);
        wg.add_waypoint("C", 3, 4);
        wg.connect_bidirectional("A", "B", None);
        wg.connect_bidirectional("B", "C", None);
        let (path, cost) = wg.shortest_path("A", "C").unwrap();
        assert_eq!(path, vec!["A", "B", "C"]);
        assert!((cost - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_graph_astar_path() {
        let mut wg = WaypointGraph::new();
        wg.add_waypoint("A", 0, 0);
        wg.add_waypoint("B", 3, 0);
        wg.add_waypoint("C", 3, 4);
        wg.connect_bidirectional("A", "B", None);
        wg.connect_bidirectional("B", "C", None);
        let (path, cost) = wg.shortest_path_astar("A", "C").unwrap();
        assert_eq!(path, vec!["A", "B", "C"]);
        assert!((cost - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_waypoint_graph_optimize_two_stops() {
        let mut wg = WaypointGraph::new();
        wg.add_waypoint("A", 0, 0);
        wg.add_waypoint("B", 5, 0);
        wg.connect_bidirectional("A", "B", None);
        let (path, cost) = wg.optimize_route(&["A", "B"]).unwrap();
        assert_eq!(path.len(), 2);
        assert!((cost - 5.0).abs() < 1e-10);
    }

    // -- Obstacle tests --
    #[test]
    fn test_obstacle_contains() {
        let obs = Obstacle::new(5, 5, 3, 3);
        assert!(obs.contains(Point::new(5, 5)));
        assert!(obs.contains(Point::new(7, 7)));
        assert!(!obs.contains(Point::new(4, 5)));
        assert!(!obs.contains(Point::new(8, 5)));
    }

    #[test]
    fn test_obstacle_map_blocked() {
        let mut map = ObstacleMap::new(32, 32);
        map.add_obstacle(Obstacle::new(5, 5, 2, 2));
        assert!(map.is_blocked(Point::new(5, 5)));
        assert!(map.is_blocked(Point::new(6, 6)));
        assert!(!map.is_blocked(Point::new(0, 0)));
    }

    #[test]
    fn test_obstacle_map_out_of_bounds() {
        let map = ObstacleMap::new(32, 32);
        assert!(map.is_blocked(Point::new(-1, 0)));
        assert!(map.is_blocked(Point::new(32, 0)));
    }

    #[test]
    fn test_obstacle_line_intersection() {
        let obs = Obstacle::new(3, 0, 2, 2);
        let a = Point::new(0, 1);
        let b = Point::new(10, 1);
        assert!(obs.intersects_line(a, b));
    }

    #[test]
    fn test_obstacle_no_line_intersection() {
        let obs = Obstacle::new(5, 5, 2, 2);
        let a = Point::new(0, 0);
        let b = Point::new(4, 4);
        assert!(!obs.intersects_line(a, b));
    }

    #[test]
    fn test_obstacle_map_to_grid() {
        let mut map = ObstacleMap::new(8, 8);
        map.add_obstacle(Obstacle::new(2, 2, 2, 2));
        let grid = map.to_grid::<8>();
        assert!(grid[2][2]);
        assert!(grid[3][3]);
        assert!(!grid[0][0]);
    }

    // -- NavMesh tests --
    #[test]
    fn test_nav_triangle_contains() {
        let tri = NavTriangle::new(
            Point::new(0, 0),
            Point::new(4, 0),
            Point::new(0, 4),
            0,
        );
        assert!(tri.contains_point(Point::new(1, 1)));
        assert!(!tri.contains_point(Point::new(3, 3)));
    }

    #[test]
    fn test_nav_triangle_area() {
        let tri = NavTriangle::new(
            Point::new(0, 0),
            Point::new(4, 0),
            Point::new(0, 4),
            0,
        );
        assert!((tri.area() - 8.0).abs() < 1e-10);
    }

    #[test]
    fn test_nav_mesh_add_and_find() {
        let mut mesh = NavMesh::new();
        mesh.add_triangle(Point::new(0, 0), Point::new(4, 0), Point::new(0, 4));
        assert_eq!(mesh.triangle_count(), 1);
        assert!(mesh.find_triangle(Point::new(1, 1)).is_some());
        assert!(mesh.find_triangle(Point::new(5, 5)).is_none());
    }

    #[test]
    fn test_nav_mesh_adjacency() {
        let mut mesh = NavMesh::new();
        // Two triangles sharing an edge
        mesh.add_triangle(Point::new(0, 0), Point::new(4, 0), Point::new(0, 4));
        mesh.add_triangle(Point::new(0, 4), Point::new(4, 0), Point::new(4, 4));
        let graph = mesh.to_graph();
        // Each should have an edge to the other
        assert!(graph.neighbors(0).iter().any(|(n, _)| *n == 1));
        assert!(graph.neighbors(1).iter().any(|(n, _)| *n == 0));
    }

    #[test]
    fn test_nav_mesh_pathfinding() {
        let mut mesh = NavMesh::new();
        // Create a 2x1 grid of triangles
        mesh.add_triangle(Point::new(0, 0), Point::new(4, 0), Point::new(0, 4));
        mesh.add_triangle(Point::new(0, 4), Point::new(4, 0), Point::new(4, 4));
        mesh.add_triangle(Point::new(4, 0), Point::new(8, 0), Point::new(4, 4));
        mesh.add_triangle(Point::new(4, 4), Point::new(8, 0), Point::new(8, 4));

        let from = Point::new(1, 1); // in tri 0
        let to = Point::new(7, 1);   // in tri 2 or 3
        let result = mesh.find_path(from, to);
        assert!(result.is_some());
        let (path, cost) = result.unwrap();
        assert!(!path.is_empty());
        assert!(cost > 0.0);
    }

    // -- Route optimization tests --
    #[test]
    fn test_least_cost_route() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "B", 2.0);
        g.add_edge("A", "C", 10.0);
        g.add_edge("B", "C", 1.0);
        let route = least_cost_route(&g, "A", "C").unwrap();
        assert_eq!(route.path, vec!["A", "B", "C"]);
        assert!((route.total_cost - 3.0).abs() < 1e-10);
        assert_eq!(route.hop_count, 2);
    }

    #[test]
    fn test_min_hop_route() {
        let mut g = Graph::new();
        g.add_node("A");
        g.add_node("B");
        g.add_node("C");
        g.add_edge("A", "C", 100.0); // direct but expensive
        g.add_edge("A", "B", 1.0);
        g.add_edge("B", "C", 1.0);
        let route = min_hop_route(&g, "A", "C").unwrap();
        assert_eq!(route.hop_count, 1); // direct path has fewer hops
        assert_eq!(route.path, vec!["A", "C"]);
    }

    // -- Original Navigator backward-compat tests --
    #[test]
    fn test_new_navigator() {
        let nav = Navigator::new();
        assert_eq!(nav.current(), Point { x: 0, y: 0 });
        assert!(nav.at_destination());
        assert!(!nav.blocked());
    }

    #[test]
    fn test_default_impl() {
        let nav = Navigator::default();
        assert_eq!(nav.current(), Point { x: 0, y: 0 });
    }

    #[test]
    fn test_set_grid() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        g[5][5] = true;
        nav.set_grid(&g);
        assert!(!nav.set_destination(5, 5));
    }

    #[test]
    fn test_simple_path() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(3, 0));
        assert_eq!(nav.step(), 1);
        assert_eq!(nav.step(), 1);
        assert_eq!(nav.step(), 0);
        assert!(nav.at_destination());
    }

    #[test]
    fn test_diagonal_bfs() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(2, 2));
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
        }
        assert_eq!(steps, 3);
        assert_eq!(nav.current(), Point { x: 2, y: 2 });
        assert!(nav.at_destination());
    }

    #[test]
    fn test_blocked_cell() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        g[1][0] = true;
        nav.set_grid(&g);
        assert!(nav.set_destination(2, 0));
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
        }
        assert_eq!(nav.current(), Point { x: 2, y: 0 });
    }

    #[test]
    fn test_unreachable() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        for y in 0..32 {
            g[1][y] = true;
        }
        nav.set_grid(&g);
        assert!(!nav.set_destination(2, 0));
    }

    #[test]
    fn test_out_of_bounds_dest() {
        let mut nav = Navigator::new();
        assert!(!nav.set_destination(-1, 0));
        assert!(!nav.set_destination(32, 0));
        assert!(!nav.set_destination(0, 32));
    }

    #[test]
    fn test_same_pos_dest() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(0, 0));
        assert!(nav.at_destination());
    }

    #[test]
    fn test_waypoints() {
        let mut nav = Navigator::new();
        nav.add_waypoint(3, 0);
        nav.add_waypoint(3, 3);
        assert!(nav.set_destination(0, 3));
        loop {
            let r = nav.step();
            if r == -1 { panic!("blocked"); }
            if r == 0 && nav.at_destination() { break; }
        }
        assert_eq!(nav.current(), Point { x: 0, y: 3 });
    }

    #[test]
    fn test_clear_waypoints() {
        let mut nav = Navigator::new();
        nav.add_waypoint(5, 5);
        nav.clear_waypoints();
        assert_eq!(nav.next_waypoint(), nav.dest);
    }

    #[test]
    fn test_next_waypoint() {
        let mut nav = Navigator::new();
        nav.add_waypoint(4, 0);
        assert!(nav.set_destination(0, 4));
        assert_eq!(nav.next_waypoint(), Point { x: 4, y: 0 });
    }

    #[test]
    fn test_progress() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(10, 0));
        let p0 = nav.progress();
        assert!(p0 >= 0.0 && p0 <= 1.0);
        for _ in 0..5 {
            nav.step();
        }
        let p1 = nav.progress();
        assert!(p1 >= p0);
    }

    #[test]
    fn test_replan() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(5, 0));
        nav.pos = Point { x: 2, y: 0 };
        let mut g = [[false; 32]; 32];
        g[3][0] = true;
        g[3][1] = true;
        nav.set_grid(&g);
        assert!(nav.replan());
    }

    #[test]
    fn test_step_when_not_navigating() {
        let mut nav = Navigator::new();
        assert_eq!(nav.step(), 0);
    }

    #[test]
    fn test_blocked_flag() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        for y in 0..32 { g[1][y] = true; }
        nav.set_grid(&g);
        assert!(!nav.set_destination(2, 0));
        assert!(nav.blocked());
    }

    #[test]
    fn test_large_distance() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(31, 31));
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
            assert!(steps <= 100, "too many steps");
        }
        assert_eq!(nav.current(), Point { x: 31, y: 31 });
    }

    #[test]
    fn test_point_equality() {
        let a = Point { x: 1, y: 2 };
        let b = Point { x: 1, y: 2 };
        let c = Point { x: 2, y: 1 };
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn test_point_clone_debug() {
        let p = Point { x: 3, y: 4 };
        let q = p.clone();
        assert_eq!(p, q);
        format!("{:?}", p);
    }
}
