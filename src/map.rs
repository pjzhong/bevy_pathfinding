use bevy::prelude::*;
use noise::{
    utils::{NoiseMapBuilder, PlaneMapBuilder},
    Fbm, MultiFractal,
};

pub const MAP_WIDTH: i32 = 16;
pub const MAP_HEIGHT: i32 = 16;

/// === Events ===
pub struct MapUpdatedEvent {}

/// === Resources ===
#[derive(Debug)]
pub struct Map {
    pub width: i32,
    pub height: i32,
    pub costs: Vec<Option<i32>>,
    pub blocked: Vec<bool>,
    pub allow_diagonals: bool,
}

impl Map {
    pub fn new(width: i32, height: i32, allow_diagonals: bool) -> Map {
        Map {
            width,
            height,
            costs: vec![Some(1); (width * height) as usize],
            blocked: vec![false; (width * height) as usize],
            allow_diagonals,
        }
    }

    pub fn xy_idx(&self, x: i32, y: i32) -> usize {
        (y as usize * self.width as usize) + x as usize
    }

    pub fn inside(&self, x: i32, y: i32) -> bool {
        0 <= x && x < self.width && 0 <= y && y < self.height
    }

    pub fn outside(&self, x: i32, y: i32) -> bool {
        !self.inside(x, y)
    }

    pub fn is_blocked(&self, x: i32, y: i32) -> bool {
        self.outside(x, y) || self.blocked[self.xy_idx(x, y)]
    }

    pub fn is_path(&self, x: i32, y: i32) -> bool {
        self.inside(x, y) & !self.is_blocked(x, y)
    }

    pub fn walkable_position(&self, x: i32, y: i32) -> Option<Position> {
        if self.outside(x, y) {
            return None;
        }

        if self.is_blocked(x, y) {
            return None;
        }

        Some(Position(x, y))
    }

    pub fn get_successors(&self, node: &Position) -> Vec<Successor> {
        let (x, y) = (node.x(), node.y());

        let n = self.walkable_position(x, y - 1);
        let e = self.walkable_position(x + 1, y);
        let s = self.walkable_position(x, y + 1);
        let w = self.walkable_position(x - 1, y);

        let nw = if n.is_some() || w.is_some() {
            self.walkable_position(x - 1, y - 1)
        } else {
            None
        };
        let ne = if n.is_some() || e.is_some() {
            self.walkable_position(x + 1, y - 1)
        } else {
            None
        };
        let se = if s.is_some() || e.is_some() {
            self.walkable_position(x + 1, y + 1)
        } else {
            None
        };
        let sw = if s.is_some() || w.is_some() {
            self.walkable_position(x - 1, y + 1)
        } else {
            None
        };

        vec![n, e, s, w, nw, ne, se, sw]
            .into_iter()
            .flatten()
            .map(|node| {
                let neighbor_index = self.xy_idx(node.x(), node.y());
                let neighbor_cost = self.costs[neighbor_index];
                if let Some(neighbor_cost) = neighbor_cost {
                    Successor {
                        position: node,
                        cost: neighbor_cost,
                    }
                } else {
                    Successor {
                        position: node,
                        cost: 1,
                    }
                }
            })
            .collect()
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Position(pub i32, pub i32);

impl Position {
    pub fn distance(&self, other: &Position) -> i32 {
        (self.0.abs_diff(other.0) + self.1.abs_diff(other.1)) as i32
    }

    pub fn x(&self) -> i32 {
        self.0
    }

    pub fn y(&self) -> i32 {
        self.1
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, PartialOrd)]
pub struct Successor {
    pub position: Position,
    pub cost: i32,
}

/// === Systems ===
pub fn setup_map(mut commands: Commands) {
    println!("Setup Map...");
    let mut map = Map::new(MAP_WIDTH, MAP_HEIGHT, true);
    //噪音函数，自动生成阻挡物
    let fbm = Fbm::new()
        .set_octaves(16)
        .set_frequency(1.5)
        .set_lacunarity(3.0)
        .set_persistence(0.9);
    let plane = PlaneMapBuilder::new(&fbm)
        .set_size(MAP_WIDTH as usize, MAP_HEIGHT as usize)
        .build();
    //阻挡物生成阈值
    let threshold = 0.3;
    for w in 0..MAP_WIDTH {
        for h in 0..MAP_HEIGHT {
            if threshold < plane.get_value(w as usize, h as usize) {
                let idx = map.xy_idx(w, h);
                map.blocked[idx] = true;
            }
        }
    }

    commands.insert_resource(map);
}
