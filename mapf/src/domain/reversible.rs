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

/// If a domain is reversible then you can obtain its ReverseDomain whose
/// choices and dynamics can be used to search this domain in reverse.
///
/// This is useful for bidirectional search algorithms.
pub trait Reversible {
    /// The type of the reverse domain
    type ReverseDomain;

    /// What kind of error can happen if the domain has the wrong values in it.
    type ReversalError;

    /// Get the reverse of this domain
    fn reverse(&self) -> Result<Self::ReverseDomain, Self::ReversalError>;
}
