use std::collections::VecDeque;

#[derive(Clone, Debug, Copy, PartialEq)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

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
                // reconstruct
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
            // arrived at waypoint or final dest
            if self.current_wp < self.waypoints.len() {
                self.current_wp += 1;
                if !self.replan() {
                    return -1;
                }
                if self.path.is_empty() {
                    // already at next waypoint
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
            // path blocked, replan
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
            0 // arrived
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
        let init = ((self.path.first().map_or(self.pos, |p| self.pos).x - self.dest.x).unsigned_abs()
            + (self.path.first().map_or(self.pos, |p| self.pos).y - self.dest.y).unsigned_abs()) as f64;
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_navigator() {
        let nav = Navigator::new();
        assert_eq!(nav.current(), Point { x: 0, y: 0 });
        // default pos==dest, not navigating → at_destination is true
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
        assert_eq!(nav.step(), 1); // move to (1,0)
        assert_eq!(nav.step(), 1); // move to (2,0)
        assert_eq!(nav.step(), 0); // arrived at (3,0)
        assert!(nav.at_destination());
    }

    #[test]
    fn test_diagonal_bfs() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(2, 2));
        // BFS gives Manhattan path, should be 4 steps
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
        }
        assert_eq!(steps, 3); // 3 moves, 4th step returns 0 (arrived)
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
        // should route around via (0,1) or (2,0) path
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
        // wall across column 1
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
        // navigate to wp0 (3,0), then wp1 (3,3), then dest (0,3)
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
        // block path mid-navigation
        nav.pos = Point { x: 2, y: 0 };
        let mut g = [[false; 32]; 32];
        g[3][0] = true;
        g[3][1] = true;
        nav.set_grid(&g);
        // replan should still find a path around
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
        format!("{:?}", p); // just ensure it compiles
    }

    // ── BFS Correctness & Path Integrity ──

    #[test]
    fn test_bfs_returns_empty_path_for_same_start_goal() {
        let nav = Navigator::new();
        let result = nav.bfs(Point { x: 5, y: 5 }, Point { x: 5, y: 5 });
        assert_eq!(result, Some(vec![]));
    }

    #[test]
    fn test_bfs_returns_none_for_blocked_goal() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        g[10][10] = true;
        nav.set_grid(&g);
        let result = nav.bfs(Point { x: 0, y: 0 }, Point { x: 10, y: 10 });
        assert_eq!(result, None);
    }

    #[test]
    fn test_bfs_returns_none_for_out_of_bounds_goal() {
        let nav = Navigator::new();
        assert_eq!(nav.bfs(Point { x: 0, y: 0 }, Point { x: -1, y: 0 }), None);
        assert_eq!(nav.bfs(Point { x: 0, y: 0 }, Point { x: 32, y: 0 }), None);
        assert_eq!(nav.bfs(Point { x: 0, y: 0 }, Point { x: 0, y: -1 }), None);
        assert_eq!(nav.bfs(Point { x: 0, y: 0 }, Point { x: 0, y: 32 }), None);
    }

    #[test]
    fn test_bfs_path_consists_of_adjacent_cells() {
        let nav = Navigator::new();
        let path = nav.bfs(Point { x: 0, y: 0 }, Point { x: 5, y: 5 }).unwrap();
        // every consecutive pair must be Manhattan-adjacent (distance == 1)
        let mut prev = Point { x: 0, y: 0 };
        for pt in &path {
            let dist = (pt.x - prev.x).unsigned_abs() + (pt.y - prev.y).unsigned_abs();
            assert_eq!(dist, 1, "non-adjacent step from {:?} to {:?}", prev, pt);
            prev = *pt;
        }
    }

    #[test]
    fn test_bfs_path_length_equals_manhattan_distance_in_open_grid() {
        let nav = Navigator::new();
        let path = nav.bfs(Point { x: 0, y: 0 }, Point { x: 7, y: 4 }).unwrap();
        assert_eq!(path.len(), 11); // 7 + 4 = 11
    }

    // ── Maze / Corridor Navigation ──

    #[test]
    fn test_single_cell_corridor() {
        let mut nav = Navigator::new();
        let mut g = [[true; 32]; 32]; // everything blocked
        g[0][0] = false;
        g[1][0] = false;
        g[2][0] = false;
        g[3][0] = false;
        nav.set_grid(&g);
        assert!(nav.set_destination(3, 0));
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
        }
        assert_eq!(nav.current(), Point { x: 3, y: 0 });
    }

    #[test]
    fn test_l_shaped_corridor() {
        let mut nav = Navigator::new();
        let mut g = [[true; 32]; 32];
        // carve L-shape: (0,0)->(4,0)->(4,3)
        for x in 0..5 { g[x][0] = false; }
        for y in 0..4 { g[4][y] = false; }
        nav.set_grid(&g);
        assert!(nav.set_destination(4, 3));
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
        }
        // 4 right + 3 up = 7 moves, but last step returns 0 (arrived)
        assert_eq!(steps, 6);
        assert!(nav.at_destination());
    }

    #[test]
    fn test_zigzag_maze() {
        let mut nav = Navigator::new();
        let mut g = [[true; 32]; 32];
        // carve zigzag: row 0 cols 0..8, row 2 cols 0..8, col 7 rows 0..3
        for x in 0..8 { g[x][0] = false; }
        for x in 0..8 { g[x][2] = false; }
        for y in 0..3 { g[7][y] = false; }
        nav.set_grid(&g);
        assert!(nav.set_destination(7, 2));
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
            assert!(steps < 50, "zigzag took too many steps");
        }
        assert_eq!(nav.current(), Point { x: 7, y: 2 });
    }

    // ── Dynamic Obstacle / Replan ──

    #[test]
    fn test_dynamic_obstacle_mid_navigation() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(5, 0));
        // take 2 steps to reach (2,0)
        assert_eq!(nav.step(), 1);
        assert_eq!(nav.step(), 1);
        assert_eq!(nav.current(), Point { x: 2, y: 0 });
        // now block (3,0) and (3,1) — force detour
        let mut g = [[false; 32]; 32];
        g[3][0] = true;
        g[3][1] = true;
        nav.set_grid(&g);
        // continue stepping — should replan and eventually arrive
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 && nav.at_destination() { break; }
            if r == -1 { panic!("navigation became blocked after dynamic obstacle"); }
            steps += 1;
            assert!(steps < 30, "took too long after dynamic obstacle");
        }
        assert_eq!(nav.current(), Point { x: 5, y: 0 });
    }

    #[test]
    fn test_replan_when_not_navigating_returns_false() {
        let mut nav = Navigator::new();
        assert!(!nav.replan()); // not navigating, should return false
    }

    // ── Boundary Edge Cases ──

    #[test]
    fn test_navigate_to_all_four_corners() {
        let corners = [(0, 0), (0, 31), (31, 0), (31, 31)];
        for (cx, cy) in corners {
            let mut nav = Navigator::new();
            assert!(nav.set_destination(cx, cy), "failed to set dest ({}, {})", cx, cy);
            loop {
                let r = nav.step();
                if r == 0 { break; }
                assert_eq!(r, 1);
            }
            assert_eq!(nav.current(), Point { x: cx, y: cy });
        }
    }

    #[test]
    fn test_set_destination_to_boundary_cells() {
        let mut nav = Navigator::new();
        // all four edges, should succeed for non-blocked cells
        assert!(nav.set_destination(0, 0));
        assert!(nav.set_destination(31, 0));
        assert!(nav.set_destination(0, 31));
        assert!(nav.set_destination(31, 31));
        assert!(!nav.set_destination(32, 0));  // out of bounds
        assert!(!nav.set_destination(-1, 0));  // out of bounds
    }

    // ── Waypoint Edge Cases ──

    #[test]
    fn test_waypoint_at_start_position() {
        let mut nav = Navigator::new();
        nav.add_waypoint(0, 0); // already here
        assert!(nav.set_destination(3, 0));
        loop {
            let r = nav.step();
            if r == 0 && nav.at_destination() { break; }
            if r == -1 { panic!("blocked navigating through waypoint at start"); }
        }
        assert_eq!(nav.current(), Point { x: 3, y: 0 });
    }

    #[test]
    fn test_single_waypoint_same_as_destination() {
        let mut nav = Navigator::new();
        nav.add_waypoint(5, 5);
        assert!(nav.set_destination(5, 5));
        loop {
            let r = nav.step();
            if r == 0 && nav.at_destination() { break; }
            if r == -1 { panic!("blocked"); }
        }
        assert_eq!(nav.current(), Point { x: 5, y: 5 });
    }

    #[test]
    fn test_clear_waypoints_resets_current_wp() {
        let mut nav = Navigator::new();
        nav.add_waypoint(10, 10);
        nav.add_waypoint(20, 20);
        nav.set_destination(0, 0);
        nav.clear_waypoints();
        // after clearing, next_waypoint should be dest, not the old waypoints
        assert_eq!(nav.next_waypoint(), Point { x: 0, y: 0 });
    }

    // ── Progress ──

    #[test]
    fn test_progress_at_start_and_end() {
        let mut nav = Navigator::new();
        // at destination already
        assert!(nav.set_destination(0, 0));
        assert_eq!(nav.progress(), 1.0);

        // navigate to (10, 0) — progress should be 0.0 at start
        let mut nav2 = Navigator::new();
        nav2.set_destination(10, 0);
        let p_start = nav2.progress();
        assert!(p_start < 0.1, "progress at start should be near 0, got {}", p_start);
    }

    #[test]
    fn test_progress_is_bounded() {
        let mut nav = Navigator::new();
        nav.set_destination(15, 15);
        for _ in 0..30 {
            nav.step();
        }
        let p = nav.progress();
        assert!(p >= 0.0 && p <= 1.0, "progress {} out of [0,1]", p);
    }

    // ── Data Structure Integrity ──

    #[test]
    fn test_grid_is_32x32() {
        let nav = Navigator::new();
        // Access extreme indices to ensure grid is 32x32
        // This is a compile-time guarantee, but verify behavior
        let mut g = [[false; 32]; 32];
        g[31][31] = true;
        let mut nav = Navigator::new();
        nav.set_grid(&g);
        assert!(!nav.set_destination(31, 31)); // blocked
    }

    #[test]
    fn test_multiple_set_destination_calls() {
        let mut nav = Navigator::new();
        assert!(nav.set_destination(5, 0));
        assert!(nav.set_destination(10, 0));
        assert!(nav.set_destination(3, 3));
        assert_eq!(nav.current(), Point { x: 0, y: 0 }); // position unchanged
        // navigate to last set destination
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
        }
        assert_eq!(nav.current(), Point { x: 3, y: 3 });
    }

    // ── Performance Bounds ──

    #[test]
    fn test_max_grid_distance_within_bound() {
        let mut nav = Navigator::new();
        nav.set_destination(31, 31);
        let mut steps = 0;
        loop {
            let r = nav.step();
            if r == 0 { break; }
            assert_eq!(r, 1);
            steps += 1;
        }
        // Manhattan distance 31+31=62; step returns 1 for 61 of them, 0 on arrival
        assert_eq!(steps, 61);
    }

    #[test]
    fn test_replan_performance() {
        let mut nav = Navigator::new();
        nav.set_destination(31, 31);
        // replan 100 times — should be fast
        for _ in 0..100 {
            assert!(nav.replan());
        }
    }

    #[test]
    fn test_surrounded_origin_is_isolated() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        // block all valid neighbors of origin: (1,0) and (0,1)
        g[1][0] = true;
        g[0][1] = true;
        nav.set_grid(&g);
        // origin is still accessible (already there)
        assert!(nav.set_destination(0, 0));
        // but can't reach anything else — origin is isolated
        assert!(!nav.set_destination(2, 1));
        assert!(nav.blocked());
    }

    #[test]
    fn test_completely_walled_off_destination() {
        let mut nav = Navigator::new();
        let mut g = [[false; 32]; 32];
        // wall around (5,5) on all 4 sides
        g[4][5] = true;
        g[6][5] = true;
        g[5][4] = true;
        g[5][6] = true;
        nav.set_grid(&g);
        assert!(!nav.set_destination(5, 5));
        assert!(nav.blocked());
    }
}
