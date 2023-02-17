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

pub use std::error::Error as StdError;
pub use anyhow::Error as Anyhow;
pub use thiserror::Error as ThisError;

// pub trait Error: StdError + Send + Sync + 'static {}
// impl<T: StdError + Send + Sync + 'static> Error for T {}

/// Use this enum for situations where you are required to provide an Error
/// type but there is no possibility of an error being produced. Since NoError
/// has no variants, it is impossible to instantiate this enum.
#[derive(ThisError, Debug)]
pub enum NoError {}
