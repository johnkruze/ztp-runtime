//! Mycelial Kirchhoff organ — same 8 × f64 orb as SPECTRA `MycelialState` in
//! `spectra_genesis` `physics/terran.rs`. Health > density.
//! Conductance C = health / max(L/50, 0.25). Clock 10 Hz. Do not align(128).

/// SPECTRA mycelial last-state — 8 × f64 = 64 bytes. Field order is the wire.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct C_MycelialState {
    pub health_index: f64,
    pub hyphal_density: f64,
    pub percolation_index: f64,
    pub delivered_nutrient: f64,
    pub fragmented_flag: f64,
    pub conductance_mean: f64,
    pub tilling_stress: f64,
    pub seal_f64: f64,
}

const _: () = {
    assert!(core::mem::size_of::<C_MycelialState>() == 64);
    assert!(core::mem::align_of::<C_MycelialState>() == 8);
    assert!(core::mem::offset_of!(C_MycelialState, health_index) == 0);
    assert!(core::mem::offset_of!(C_MycelialState, hyphal_density) == 8);
    assert!(core::mem::offset_of!(C_MycelialState, percolation_index) == 16);
    assert!(core::mem::offset_of!(C_MycelialState, delivered_nutrient) == 24);
    assert!(core::mem::offset_of!(C_MycelialState, fragmented_flag) == 32);
    assert!(core::mem::offset_of!(C_MycelialState, conductance_mean) == 40);
    assert!(core::mem::offset_of!(C_MycelialState, tilling_stress) == 48);
    assert!(core::mem::offset_of!(C_MycelialState, seal_f64) == 56);
};

pub const PERCOLATION_HEALTH: f64 = 0.40;
pub const DELIVERY_GATE: f64 = 0.10;
pub const HYPHAL_SCALE_M: f64 = 50.0;
pub const SPAN_FRAC: f64 = 0.80;
pub const MYCELIAL_HZ: f64 = 10.0;
pub const MYCELIAL_DT: f64 = 1.0 / MYCELIAL_HZ;
pub const T_ESTABLISH: usize = 200;
pub const T_PROPAGATE: usize = 600;
pub const T_MEASURE: usize = 700;
const N_COLS: usize = 8;
const N_ROWS: usize = 5;
const N_NODES: usize = N_COLS * N_ROWS;
const RADIUS_M: f64 = 350.0;
const PROPAGATION_RATE: f64 = 0.05;
const DECAY_RATE: f64 = 0.01;

struct HyphalEdge {
    from: usize,
    to: usize,
    length: f64,
    health: f64,
    alive: bool,
}

impl HyphalEdge {
    fn conductance(&self) -> f64 {
        if !self.alive || self.health < 0.01 {
            return 0.0;
        }
        self.health / (self.length / HYPHAL_SCALE_M).max(0.25)
    }
}

struct MycelialNode {
    position: [f64; 2],
    health_index: f64,
    nutrient_density: f64,
    is_source: bool,
    is_sink: bool,
    signal_level: f64,
}

struct MycelialMesh {
    nodes: Vec<MycelialNode>,
    edges: Vec<HyphalEdge>,
}

impl MycelialMesh {
    /// Reconstructible east–west field. Same named plot as SPECTRA terran.rs.
    fn named_field(health_mean: f64, connectivity: f64) -> Self {
        let health_mean = health_mean.clamp(0.0, 1.0);
        let connectivity = connectivity.clamp(0.0, 1.0);
        let mut nodes = Vec::with_capacity(N_NODES);
        for r in 0..N_ROWS {
            for c in 0..N_COLS {
                let x = -RADIUS_M + 2.0 * RADIUS_M * (c as f64) / (N_COLS as f64 - 1.0);
                let y = -0.5 * RADIUS_M + RADIUS_M * (r as f64) / (N_ROWS as f64 - 1.0);
                nodes.push(MycelialNode {
                    position: [x, y],
                    health_index: health_mean,
                    nutrient_density: 0.25,
                    is_source: false,
                    is_sink: false,
                    signal_level: 0.0,
                });
            }
        }
        let max_edge = RADIUS_M * SPAN_FRAC;
        let reach = 105.0 + connectivity * max_edge;
        let mut edges = Vec::new();
        for i in 0..N_NODES {
            for j in (i + 1)..N_NODES {
                let dx = nodes[i].position[0] - nodes[j].position[0];
                let dy = nodes[i].position[1] - nodes[j].position[1];
                let dist = (dx * dx + dy * dy).sqrt();
                if dist < 1e-9 || dist >= max_edge || dist > reach {
                    continue;
                }
                edges.push(HyphalEdge {
                    from: i,
                    to: j,
                    length: dist,
                    health: health_mean,
                    alive: true,
                });
            }
        }
        let mut mesh = Self { nodes, edges };
        mesh.mark_field_ports();
        mesh
    }

    fn mark_field_ports(&mut self) {
        let n = self.nodes.len();
        if n < 2 {
            return;
        }
        for node in self.nodes.iter_mut() {
            node.is_source = false;
            node.is_sink = false;
            node.signal_level = 0.0;
        }
        let mut degree = vec![0u32; n];
        for e in &self.edges {
            if e.alive {
                degree[e.from] += 1;
                degree[e.to] += 1;
            }
        }
        let mut i_src = 0usize;
        let mut i_snk = 0usize;
        let mut x_min = f64::INFINITY;
        let mut x_max = f64::NEG_INFINITY;
        for (i, node) in self.nodes.iter().enumerate() {
            if degree[i] == 0 {
                continue;
            }
            if node.position[0] < x_min {
                x_min = node.position[0];
                i_src = i;
            }
            if node.position[0] > x_max {
                x_max = node.position[0];
                i_snk = i;
            }
        }
        if i_src == i_snk {
            i_snk = (i_src + 1) % n;
        }
        self.nodes[i_src].is_source = true;
        self.nodes[i_src].nutrient_density = 1.0;
        self.nodes[i_src].signal_level = 1.0;
        self.nodes[i_snk].is_sink = true;
    }

    fn step_signal(&mut self, dt: f64) {
        let n = self.nodes.len();
        let mut flow = vec![0.0_f64; n];
        for edge in &self.edges {
            if !edge.alive {
                continue;
            }
            let g = edge.conductance();
            if g <= 0.0 {
                continue;
            }
            let delta = self.nodes[edge.from].signal_level - self.nodes[edge.to].signal_level;
            let current = g * delta * dt;
            flow[edge.from] -= current;
            flow[edge.to] += current;
        }
        for (i, node) in self.nodes.iter_mut().enumerate() {
            node.signal_level += flow[i];
            if node.is_source {
                node.signal_level = node.signal_level.max(1.0);
            }
            node.signal_level *= 1.0 - DECAY_RATE * dt;
            node.signal_level = node.signal_level.clamp(0.0, 10.0);
        }
    }

    fn step_nutrients(&mut self, dt: f64) {
        let n = self.nodes.len();
        let mut flow = vec![0.0_f64; n];
        for edge in &self.edges {
            if !edge.alive {
                continue;
            }
            let g = edge.conductance();
            if g <= 0.0 {
                continue;
            }
            let delta =
                self.nodes[edge.from].nutrient_density - self.nodes[edge.to].nutrient_density;
            let current = g * delta * PROPAGATION_RATE * dt;
            flow[edge.from] -= current;
            flow[edge.to] += current;
        }
        for (i, node) in self.nodes.iter_mut().enumerate() {
            node.nutrient_density += flow[i];
            if node.is_source {
                node.nutrient_density = node.nutrient_density.max(0.8);
            }
            node.nutrient_density = node.nutrient_density.clamp(0.0, 2.0);
        }
    }

    fn step_health(&mut self, dt: f64) {
        for edge in self.edges.iter_mut() {
            if !edge.alive {
                continue;
            }
            let avg_h =
                (self.nodes[edge.from].health_index + self.nodes[edge.to].health_index) / 2.0;
            if avg_h > 0.5 {
                edge.health = (edge.health + 0.01 * dt).min(1.0);
            } else {
                edge.health -= DECAY_RATE * dt;
                if edge.health <= 0.0 {
                    edge.alive = false;
                }
            }
        }
    }

    fn apply_tilling(&mut self, intensity: f64) {
        if intensity <= 0.0 {
            return;
        }
        for (i, edge) in self.edges.iter_mut().enumerate() {
            if !edge.alive {
                continue;
            }
            let u = splitmix_unit(0x5449_4C4C_0000_0000 ^ i as u64);
            if u < intensity * 0.5 {
                edge.alive = false;
            }
        }
    }

    fn source_sink_connected(&self) -> bool {
        let n = self.nodes.len();
        if n < 2 {
            return false;
        }
        let mut parent: Vec<usize> = (0..n).collect();
        for edge in &self.edges {
            if !edge.alive {
                continue;
            }
            let a = find(&mut parent, edge.from);
            let b = find(&mut parent, edge.to);
            if a != b {
                parent[a] = b;
            }
        }
        let source_idx = self.nodes.iter().position(|n| n.is_source);
        let sink_idx = self.nodes.iter().position(|n| n.is_sink);
        match (source_idx, sink_idx) {
            (Some(s), Some(t)) => find(&mut parent, s) == find(&mut parent, t),
            _ => false,
        }
    }

    fn delivery_ratio(&self) -> f64 {
        let source_signal = self
            .nodes
            .iter()
            .filter(|n| n.is_source)
            .map(|n| n.signal_level)
            .sum::<f64>()
            .max(0.001);
        let sink_signal = self
            .nodes
            .iter()
            .filter(|n| n.is_sink)
            .map(|n| n.signal_level)
            .sum::<f64>();
        (sink_signal / source_signal).min(1.0)
    }

    fn average_edge_health(&self) -> f64 {
        let alive: Vec<&HyphalEdge> = self.edges.iter().filter(|e| e.alive).collect();
        if alive.is_empty() {
            return 0.0;
        }
        alive.iter().map(|e| e.health).sum::<f64>() / alive.len() as f64
    }

    fn density(&self) -> f64 {
        let n = self.nodes.len();
        if n < 2 {
            return 0.0;
        }
        let max_edges = n * (n - 1) / 2;
        let alive_edges = self.edges.iter().filter(|e| e.alive).count();
        alive_edges as f64 / max_edges as f64
    }

    fn mean_conductance(&self) -> f64 {
        let alive: Vec<&HyphalEdge> = self.edges.iter().filter(|e| e.alive).collect();
        if alive.is_empty() {
            return 0.0;
        }
        alive.iter().map(|e| e.conductance()).sum::<f64>() / alive.len() as f64
    }
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn splitmix_unit(x: u64) -> f64 {
    let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z = z ^ (z >> 31);
    (z >> 11) as f64 / (1u64 << 53) as f64
}

fn seal_from_slots(slots: [f64; 7]) -> f64 {
    let mut h: u64 = 0x4B49_5243_484F_4646;
    for x in slots {
        h ^= x.to_bits();
        h = h.rotate_left(13).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    }
    f64::from_bits((0x3FF_u64 << 52) | (h & 0x000F_FFFF_FFFF_FFFF))
}

fn snapshot(mesh: &MycelialMesh, tilling_stress: f64) -> C_MycelialState {
    let health_index = mesh.average_edge_health();
    let hyphal_density = mesh.density();
    let percolation_index = health_index / PERCOLATION_HEALTH;
    let delivered_nutrient = mesh.delivery_ratio();
    let fragmented_flag = if mesh.source_sink_connected() { 0.0 } else { 1.0 };
    let conductance_mean = mesh.mean_conductance();
    let tilling_stress = tilling_stress.max(0.0);
    let seal_f64 = seal_from_slots([
        health_index,
        hyphal_density,
        percolation_index,
        delivered_nutrient,
        fragmented_flag,
        conductance_mean,
        tilling_stress,
    ]);
    C_MycelialState {
        health_index,
        hyphal_density,
        percolation_index,
        delivered_nutrient,
        fragmented_flag,
        conductance_mean,
        tilling_stress,
        seal_f64,
    }
}

/// One reconstructible Kirchhoff run up to `time_s` at 10 Hz.
/// `time_s >= 70` (or NaN / negative) is the SPECTRA minute (`T_MEASURE` = 700).
/// Tilling severs after `T_ESTABLISH` (20 s), same as `run_mycelial_simulation`.
pub fn evaluate_state(
    health_index: f64,
    hyphal_density: f64,
    tilling_stress: f64,
    time_s: f64,
) -> C_MycelialState {
    let time_s = if !time_s.is_finite() || time_s < 0.0 {
        (T_MEASURE as f64) * MYCELIAL_DT
    } else {
        time_s
    };
    let ticks = ((time_s / MYCELIAL_DT).round() as usize).min(T_MEASURE);
    let mut mesh = MycelialMesh::named_field(health_index, hyphal_density);
    let mut tilled = false;
    for k in 0..ticks {
        if !tilled && k >= T_ESTABLISH {
            mesh.apply_tilling(tilling_stress);
            tilled = true;
        }
        mesh.step_signal(MYCELIAL_DT);
        mesh.step_nutrients(MYCELIAL_DT);
        if k < T_PROPAGATE {
            mesh.step_health(MYCELIAL_DT);
        }
    }
    snapshot(&mesh, tilling_stress)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mycelial_state_is_64_bytes() {
        assert_eq!(core::mem::size_of::<C_MycelialState>(), 64);
        assert_eq!(core::mem::align_of::<C_MycelialState>(), 8);
        assert_eq!(core::mem::offset_of!(C_MycelialState, seal_f64), 56);
    }

    #[test]
    fn conductance_is_health_over_scaled_length() {
        let e = HyphalEdge {
            from: 0,
            to: 1,
            length: 50.0,
            health: 1.0,
            alive: true,
        };
        assert!((e.conductance() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn full_minute_matches_measure_cap() {
        let live = evaluate_state(0.72, 0.35, 0.0, 70.0);
        assert!(live.health_index > 0.5);
        assert!(live.fragmented_flag == 0.0 || live.fragmented_flag == 1.0);
        assert!(live.seal_f64.is_finite());
        let tilled = evaluate_state(0.72, 0.35, 0.85, 70.0);
        assert_eq!(tilled.tilling_stress, 0.85);
    }
}
