/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

use float_ord::FloatOrd;
use num::traits::Zero;
use std::{
    cmp::{Eq, Ord, Ordering, PartialEq, PartialOrd},
    hash::{Hash, Hasher},
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign},
};

/// Use Cost(f64) or Cost(f32) to use floating point values as planner costs,
/// giving them the traits of total ordering, full equivalence, and hashability.
#[derive(Clone, Copy, Debug)]
pub struct Cost<Num>(pub Num);

macro_rules! cost_impl {
    ($f:ident) => {
        impl Cost<$f> {
            fn convert(self) -> FloatOrd<$f> {
                FloatOrd(self.0)
            }
        }

        impl PartialEq for Cost<$f> {
            fn eq(&self, other: &Self) -> bool {
                self.convert() == other.convert()
            }
        }
        impl Eq for Cost<$f> {}
        impl PartialOrd for Cost<$f> {
            fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
                self.convert().partial_cmp(&other.convert())
            }
        }
        impl Ord for Cost<$f> {
            fn cmp(&self, other: &Self) -> Ordering {
                self.convert().cmp(&other.convert())
            }
        }
        impl Hash for Cost<$f> {
            fn hash<H: Hasher>(&self, state: &mut H) {
                self.convert().hash(state);
            }
        }
        impl Add for Cost<$f> {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Cost(self.0 + rhs.0)
            }
        }
        impl AddAssign for Cost<$f> {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }
        impl Sub for Cost<$f> {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Cost(self.0 - rhs.0)
            }
        }
        impl SubAssign for Cost<$f> {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }
        impl Mul for Cost<$f> {
            type Output = Self;
            fn mul(self, rhs: Self) -> Self {
                Cost(self.0 * rhs.0)
            }
        }
        impl MulAssign for Cost<$f> {
            fn mul_assign(&mut self, rhs: Self) {
                self.0 *= rhs.0;
            }
        }
        impl Div for Cost<$f> {
            type Output = Self;
            fn div(self, rhs: Self) -> Self {
                Cost(self.0 / rhs.0)
            }
        }
        impl DivAssign for Cost<$f> {
            fn div_assign(&mut self, rhs: Self) {
                self.0 /= rhs.0;
            }
        }
        impl Zero for Cost<$f> {
            fn zero() -> Self {
                Cost(0.0)
            }
            fn set_zero(&mut self) {
                self.0 = 0.0
            }
            fn is_zero(&self) -> bool {
                self.0 == 0.0
            }
        }
    };
}

cost_impl!(f32);
cost_impl!(f64);
