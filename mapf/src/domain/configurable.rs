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

use crate::error::Anyhow;

/// Configure the parameters of a domain
pub trait Configurable {
    type Configuration;
    fn configure<F>(self, f: F) -> Result<Self, Anyhow>
    where
        F: FnOnce(Self::Configuration) -> Result<Self::Configuration, Anyhow>,
        Self: Sized;
    // TODO(@mxgrey): Consider whether it's possible to support custom strongly
    // typed error structs. The main challenge is that callbacks used inside the
    // configuration callback could have arbitrary error structs. That's why we
    // roll it all into Anyhow for now.
}
