use bevy::utils::HashMap;
use std::{
    cmp::Ordering,
    collections::{BTreeSet, HashSet},
    f32::consts::SQRT_2,
    rc::Rc,
};

use crate::{Map, Position};

struct Heuristic;

impl Heuristic {
    pub fn octile(dx: f32, dy: f32) -> f32 {
        let squart2 = SQRT_2 - 1.0;
        if dx < dy {
            squart2 * dx + dy
        } else {
            squart2 * dy + dx
        }
    }

    pub fn manhattan(dx: f32, dy: f32) -> f32 {
        dx + dy
    }
}

pub struct Jps;

#[derive(Debug)]
struct PathNode {
    node: Position,
    parent: Option<Rc<PathNode>>,
    // distance to start + estimate to end
    f: f32,
    // distance to start (parent's g + distance from parent)
    g: f32,
    // estimate to end
    h: f32,
}

impl PathNode {
    fn new(node: Position) -> Self {
        Self {
            node,
            parent: None,
            f: 0.0,
            g: 0.0,
            h: 0.0,
        }
    }

    fn get_x(&self) -> i32 {
        self.node.0
    }

    fn get_y(&self) -> i32 {
        self.node.1
    }

    fn set_parent(&mut self, parent: Rc<PathNode>) {
        self.parent = Some(parent);
    }
}

impl PartialEq for PathNode {
    fn eq(&self, other: &Self) -> bool {
        self.f == other.f
    }
}

impl Eq for PathNode {}

impl PartialOrd for PathNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PathNode {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f.total_cmp(&other.f)
    }
}

impl Jps {
    pub fn find_path(
        graph: &Map,
        start: Position,
        end: Position,
    ) -> (Option<Vec<Position>>, Vec<Position>) {
        if graph.is_blocked(end.x(), end.y()) {
            return (None, vec![]);
        }

        let mut open: BTreeSet<Rc<PathNode>> = BTreeSet::new();

        open.insert(Rc::new(PathNode::new(start)));

        let mut closed: HashSet<Position> = HashSet::new();
        let mut node_path: HashMap<Position, Rc<PathNode>> = HashMap::new();
        let mut tested = HashSet::new();
        while let Some(path_node) = open.pop_first() {
            if path_node.node == end {
                //TODO backtrace
                return (Some(back_trace(path_node)), Vec::from_iter(tested));
            }
            closed.insert(path_node.node.clone());
            Jps::identify_successors(
                graph,
                path_node.clone(),
                end,
                &mut closed,
                &mut open,
                &mut node_path,
                &mut tested,
            );
        }

        (None, Vec::from_iter(tested))
    }

    fn identify_successors(
        graph: &Map,
        node: Rc<PathNode>,
        goal: Position,
        closed: &mut HashSet<Position>,
        open: &mut BTreeSet<Rc<PathNode>>,
        node_path: &mut HashMap<Position, Rc<PathNode>>,
        tested: &mut HashSet<Position>,
    ) {
        let (end_x, end_y) = (goal.x(), goal.y());
        let neighbors = Jps::find_neighbors(graph, node.clone());
        for neighbor in neighbors {
            let jump_point = Jps::jump(graph, neighbor, node.node, goal, tested);
            if let Some(jump_point) = jump_point {
                if closed.contains(&jump_point) {
                    continue;
                }

                let (jx, jy) = (jump_point.x(), jump_point.y());
                let d = {
                    let dx = (jx - node.get_x()).abs() as f32;
                    let dy = (jy - node.get_y()).abs() as f32;
                    Heuristic::octile(dx, dy)
                };
                let ng = node.g + d; //next 'g' value

                let (none, prev_g) = node_path
                    .get(&jump_point)
                    .map_or((true,  0.0), |pn| (false, pn.g));
                if none || ng < prev_g {
                    let mut jump_node = PathNode::new(jump_point.clone());
                    jump_node.set_parent(node.clone());
                    jump_node.g = ng;
                    jump_node.h = Heuristic::manhattan((jx - end_x).abs() as f32, (jy - end_y).abs() as f32);
                    jump_node.f = jump_node.g + jump_node.h;

                    let jump_node = Rc::new(jump_node);
                    //update the old one
                    if let Some(prev_path_node) = node_path.remove(&jump_point) {
                        node_path.insert(jump_point.clone(), jump_node.clone());
                        open.remove(&prev_path_node);
                        open.insert(jump_node);
                    } else {
                        //insert the new one
                        open.insert(jump_node.clone());
                        node_path.insert(jump_point.clone(), jump_node);
                    }
                }
            }
        }
    }

    fn jump(
        graph: &Map,
        current: Position,
        neighbor: Position,
        goal: Position,
        tested: &mut HashSet<Position>,
    ) -> Option<Position> {
        if graph.is_blocked(current.x(), current.y()) {
            return None;
        }

        if current == goal {
            return Some(current);
        }

        tested.insert(current);

        let (x, y) = (current.x(), current.y());
        let (dx, dy) = (current.x() - neighbor.x(), current.y() - neighbor.y());

        // check for forced neighbors
        // along the diagonal
        if dx != 0 && dy != 0 {
            if (graph.is_path(x - dx, y + dy) && graph.is_blocked(x - dx, y))
                || (graph.is_path(x + dx, y - dy) && graph.is_blocked(x, y - dy))
            {
                return Some(current);
            }

            for next in vec![
                graph.walkable_position(x + dx, y),
                graph.walkable_position(x, y + dy),
            ]
            .into_iter()
            .flatten()
            {
                if Jps::jump(graph, next, current, goal, tested).is_some() {
                    return graph.walkable_position(x, y);
                }
            }
        } else {
            // check horizontally/vertically

            if dx != 0 {
                if graph.is_path(x + dx, y + 1) && graph.is_blocked(x, y + 1)
                    || graph.is_path(x + dx, y - 1) && graph.is_blocked(x, y - 1)
                {
                    return graph.walkable_position(x, y);
                }
            } else {
                if graph.is_path(x + 1, y + dy) && graph.is_blocked(x + 1, y)
                    || graph.is_path(x - 1, y + dy) && graph.is_blocked(x - 1, y)
                {
                    return graph.walkable_position(x, y);
                }
            }
        }

        // moving diagonally, must make sure one of the vertical/hhorizontal
        // neighbors is open to allow the path
        if graph.is_path(x + dx, y) || graph.is_path(x, y + dy) {
            if let Some(next) = graph.walkable_position(x + dx, y + dy) {
                Jps::jump(graph, next, current, goal, tested)
            } else {
                None
            }
        } else {
            None
        }
    }

    /// https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html
    fn find_neighbors(graph: &Map, node: Rc<PathNode>) -> Vec<Position> {
        if let Some(parent) = node.parent.as_ref() {
            let (x, y) = (node.get_x(), node.get_y());

            let (dx, dy) = {
                let (px, py) = (parent.get_x(), parent.get_y());

                (
                    (x - px) / 1.max((x - px).abs()),
                    (y - py) / 1.max((y - py).abs()),
                )
            };

            if dx != 0 && dy != 0 {
                let mut vec = vec![];
                let horizonetal = graph.walkable_position(x, y + dy);
                let vertically = graph.walkable_position(x + y, y);

                // moving horizonetally and vertically first
                if let Some(node) = horizonetal.as_ref() {
                    vec.push(node.clone());
                };
                if let Some(node) = vertically.as_ref() {
                    vec.push(node.clone());
                };

                // moving  diagonally
                if horizonetal.is_some() || vertically.is_some() {
                    if let Some(node) = graph.walkable_position(x + dx, y + dy) {
                        vec.push(node)
                    }
                }

                // force neighbors
                if graph.is_blocked(x - dx, y) && graph.is_path(x, y + dy) {
                    if let Some(node) = graph.walkable_position(x - dx, y + dy) {
                        vec.push(node);
                    }
                }

                if graph.is_blocked(x, y - dy) && graph.is_path(x + dx, y) {
                    if let Some(node) = graph.walkable_position(x + dx, y - dy) {
                        vec.push(node);
                    }
                }

                vec
            } else {
                // search horizonetally
                if dx == 0 {
                    let mut vec = vec![];
                    if let Some(node) = graph.walkable_position(x, y + dy) {
                        vec.push(node);

                        //down is force neighbors
                        if graph.is_blocked(x + 1, y) {
                            if let Some(node) = graph.walkable_position(x + 1, y + dy) {
                                vec.push(node);
                            }
                        }

                        // up is force neighbors
                        if graph.is_blocked(x - 1, y) {
                            if let Some(node) = graph.walkable_position(x - 1, y + dy) {
                                vec.push(node);
                            }
                        }
                    }

                    vec
                }
                // search vertically
                else if let Some(node) = graph.walkable_position(x + dx, y) {
                    let mut vec = vec![];
                    vec.push(node);

                    // right is force neighbors
                    if graph.is_blocked(x, y + 1) {
                        if let Some(node) = graph.walkable_position(x + dx, y + 1) {
                            vec.push(node);
                        }
                    }

                    //left is force neighbors
                    if graph.is_blocked(x, y - 1) {
                        if let Some(node) = graph.walkable_position(x + dx, y - 1) {
                            vec.push(node);
                        }
                    }

                    vec
                } else {
                    vec![]
                }
            }

            // no neighbors
        } else {
            let vec = Jps::get_all_neightbors(graph, node.node);
            vec
        }
    }

    fn get_all_neightbors(graph: &Map, node: Position) -> Vec<Position> {
        let (x, y) = (node.x(), node.y());

        let n = graph.walkable_position(x, y - 1);
        let e = graph.walkable_position(x + 1, y);
        let s = graph.walkable_position(x, y + 1);
        let w = graph.walkable_position(x - 1, y);

        let nw = if n.is_some() || w.is_some() {
            graph.walkable_position(x - 1, y - 1)
        } else {
            None
        };
        let ne = if n.is_some() || e.is_some() {
            graph.walkable_position(x + 1, y - 1)
        } else {
            None
        };
        let se = if s.is_some() || e.is_some() {
            graph.walkable_position(x + 1, y + 1)
        } else {
            None
        };
        let sw = if s.is_some() || w.is_some() {
            graph.walkable_position(x - 1, y + 1)
        } else {
            None
        };

        vec![n, e, s, w, nw, ne, se, sw]
            .into_iter()
            .flatten()
            .collect()
    }
}

// fn to_path_node(vec: Vec<Position>, parent: &Rc<PathNode>, goal: Position) -> Vec<PathNode> {
//     let (end_x, end_y) = (goal.x(), goal.y());
//     let mut res = vec![];
//     for graph_node in vec {
//         let mut path_node = PathNode::new(graph_node);
//         let (jx, jy) = (graph_node.x(), graph_node.y());
//         let d = {
//             let dx = (jx - parent.get_x()).abs() as f32;
//             let dy = (jy - parent.get_y()).abs() as f32;
//             Heuristic::octile(dx, dy)
//         };
//         path_node.set_parent(parent.clone());
//         path_node.g =  parent.g + d;
//         path_node.h = Heuristic::manhattan((jx - end_x).abs() as f32, (jy - end_y).abs() as f32);
//         path_node.f = path_node.g + path_node.h;

//         path_node.set_parent(parent.clone());

//         res.push(path_node);
//     }
//     res
// }

fn sort_neightbors(positions: Vec<Position>, gold: Position) -> Vec<Position> {
    let mut result = vec![];
    let (end_x, end_y) = (gold.x(), gold.y());
    for pos in positions {
        let val = Heuristic::manhattan(
            (pos.x() - end_x).abs() as f32,
            (pos.y() - end_y).abs() as f32,
        ) as i32;
        result.push((val, pos));
    }
    result.sort_by_key(|v| v.0);

    result.into_iter().map(|v| v.1).collect()
}

fn back_trace(path_node: Rc<PathNode>) -> Vec<Position> {
    let mut result = vec![];

    let mut start = Some(path_node);
    while let Some(node) = start {
        result.push(node.node);
        start = node.parent.clone();
    }
    result
}
