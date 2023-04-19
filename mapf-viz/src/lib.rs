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

pub mod spatial_canvas;
pub use spatial_canvas::SpatialCanvasProgram;

pub mod visibility_visual;
pub use visibility_visual::{VisibilityVisual, SparseGridVisibilityVisual};

pub mod accessibility_visual;
pub use accessibility_visual::{AccessibilityVisual, SparseGridAccessibilityVisual};

pub mod grid;
pub use grid::InfiniteGrid;

pub mod toggle;
pub use toggle::{Toggle, Toggler};

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
