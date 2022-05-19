/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

use super::{Point, Cell};
use arrayvec::ArrayVec;

pub(crate) struct SearchF64 {
    pub(crate) value: Option<f64>,
}

impl SearchF64 {
    pub(crate) fn new() -> Self {
        return Self{value: None};
    }

    pub(crate) fn check_min(&mut self, other: f64) {
        match self.value {
            None => {
                self.value = Some(other);
            },
            Some(v) => {
                self.value = Some(v.min(other));
            }
        }
    }

    pub(crate) fn check_max(&mut self, other: f64) {
        match self.value {
            None => {
                self.value = Some(other);
            },
            Some(v) => {
                self.value = Some(v.max(other));
            }
        }
    }
}

pub(crate) struct LineSegment {
    p0: Point,
    p1: Point,
}

impl LineSegment {
    pub(crate) fn new(p0: Point, p1: Point) -> Self {
        Self{p0, p1}
    }

    pub(crate) fn vertical_intersect(&self, x: f64) -> ArrayVec<f64, 2> {
        let mut intersects = ArrayVec::new();
        if self.p0.x > x && self.p1.x > x {
            return intersects;
        }

        if self.p0.x < x && self.p1.x < x {
            return intersects;
        }

        if (self.p1.x - self.p0.x).abs() < 1e-8 {
            intersects.push(self.p0.y);
            intersects.push(self.p1.y);
            return intersects;
        }

        let m = (self.p1.y - self.p0.y)/(self.p1.x - self.p0.x);
        intersects.push(m * (x - self.p0.x) + self.p0.y);
        return intersects;
    }

    pub(crate) fn horizontal_intersect(&self, y: f64) -> ArrayVec<f64, 2> {
        let mut intersects = ArrayVec::new();
        if self.p0.y > y && self.p1.y > y {
            return intersects;
        }

        if self.p0.y < y && self.p1.y < y {
            return intersects;
        }

        if (self.p1.y - self.p0.y).abs() < 1e-8 {
            intersects.push(self.p0.x);
            intersects.push(self.p1.x);
            return intersects;
        }

        let m = (self.p1.x - self.p0.x)/(self.p1.y - self.p0.y);
        intersects.push(m * (y - self.p0.y) + self.p0.x);
        return intersects;
    }

    pub(crate) fn distance_from_point(&self, p: Point) -> f64 {
        let v1 = self.p1 - self.p0;
        let length = v1.norm();
        if length < 1e-8 {
            return (p - self.p0).norm();
        }

        let v1_u = v1/length;
        let v = p - self.p0;
        let shadow = v.dot(&v1_u);
        if shadow < 0.0 {
            return (p - self.p0).norm();
        } else if shadow > length {
            return (p - self.p1).norm();
        }

        return (v - shadow * v1_u).norm();
    }

    pub(crate) fn passes_near_cell(
        &self,
        cell: &Cell,
        cell_size: f64,
        proximity: f64
    ) -> bool {
        // 1.4143 is chosen as a value slightly higher than sqrt(2) to give us
        // an upper bound on how far any point in the cell can be from the line
        // segment without completely ruling out the possibility that it passes
        // nearby.
        let bound_distance = proximity + 1.4143*cell_size;
        for m in [0, 1] {
            for n in [0, 1] {
                let p = cell.shifted(m, n).to_bottom_left_point(cell_size);
                let dist = self.distance_from_point(p);
                if dist < proximity {
                    return true;
                }

                if bound_distance <= dist {
                    return false;
                }
            }
        }

        let p_min = cell.to_bottom_left_point(cell_size);
        let p_max = cell.shifted(1, 1).to_bottom_left_point(cell_size);

        // We need to check if either line segment endpoint is inside the cell
        for p in [&self.p0, &self.p1] {
            if p_min.x < p.x && p.x < p_max.x
                && p_min.y < p.y && p.y < p_max.y
            {
                return true;
            }
        }

        // We need to check if the line segment intersects the cell
        for x in [p_min.x, p_max.x] {
            for y in self.vertical_intersect(x) {
                if p_min.y < y && y < p_max.y {
                    return true;
                }
            }
        }

        for y in [p_min.y, p_max.y] {
            for x in self.horizontal_intersect(y) {
                if p_min.x < x && x < p_max.x {
                    return true;
                }
            }
        }

        return false;
    }
}
