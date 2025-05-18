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

pub(crate) fn triangular_for<Item>(
    iterable: impl Iterator<Item = Item> + Clone,
    mut f: impl FnMut(&Item, Item),
) {
    let mut outer_iter = iterable.into_iter();
    while let Some(outer_value) = outer_iter.next() {
        let mut inner_iter = outer_iter.clone();
        while let Some(inner_value) = inner_iter.next() {
            f(&outer_value, inner_value);
        }
    }
}

pub(crate) struct Minimum<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> {
    value: Option<T>,
    #[allow(unused)]
    f: F,
}

impl<T: Clone, F: Fn(&T, &T) -> std::cmp::Ordering> Minimum<T, F> {
    #[allow(unused)]
    pub(crate) fn new(f: F) -> Self {
        Self { value: None, f }
    }

    #[allow(unused)]
    pub(crate) fn consider(&mut self, other: &T) -> bool {
        if let Some(value) = &self.value {
            if std::cmp::Ordering::Less == (self.f)(other, value) {
                self.value = Some(other.clone());
                return true;
            }
        } else {
            self.value = Some(other.clone());
            return true;
        }

        return false;
    }

    #[allow(unused)]
    pub(crate) fn consider_take(&mut self, other: T) -> bool {
        if let Some(value) = &self.value {
            if std::cmp::Ordering::Less == (self.f)(&other, value) {
                self.value = Some(other);
                return true;
            }
        } else {
            self.value = Some(other);
            return true;
        }

        return false;
    }

    #[allow(unused)]
    pub(crate) fn result(self) -> Option<T> {
        self.value
    }

    #[allow(unused)]
    pub(crate) fn has_value(&self) -> bool {
        self.value.is_some()
    }
}

pub enum FlatResultMapIter<T, U: IntoIterator, F, E> {
    Ok(std::iter::FlatMap<std::option::IntoIter<T>, U, F>),
    Err(Option<E>),
}

impl<T, U: IntoIterator, F, E> Iterator for FlatResultMapIter<T, U, F, E>
where
    F: FnMut(T) -> U,
{
    type Item = Result<U::Item, E>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Ok(inner) => Ok(inner.next()).transpose(),
            Self::Err(inner) => inner.take().map(|e| Err(e)),
        }
    }
}

pub trait FlatResultMapTrait {
    type Type;
    type Error;
    fn flat_result_map<U, F>(self, f: F) -> FlatResultMapIter<Self::Type, U, F, Self::Error>
    where
        Self: Sized,
        U: IntoIterator,
        F: FnMut(Self::Type) -> U;
}

impl<T, E> FlatResultMapTrait for Result<T, E> {
    type Type = T;
    type Error = E;
    fn flat_result_map<U, F>(self, f: F) -> FlatResultMapIter<Self::Type, U, F, Self::Error>
    where
        Self: Sized,
        U: IntoIterator,
        F: FnMut(T) -> U,
    {
        match self {
            Ok(iter) => FlatResultMapIter::Ok(Some(iter).into_iter().flat_map(f)),
            Err(err) => FlatResultMapIter::Err(Some(err)),
        }
    }
}

pub enum ForkIter<L, R> {
    Left(L),
    Right(R),
}

impl<L: Iterator, R: Iterator<Item = L::Item>> Iterator for ForkIter<L, R> {
    type Item = L::Item;
    fn next(&mut self) -> Option<Self::Item> {
        match self {
            Self::Left(left) => left.next(),
            Self::Right(right) => right.next(),
        }
    }
}

pub fn wrap_to_pi(mut value: f64) -> f64 {
    while std::f64::consts::PI < value {
        value -= 2.0 * std::f64::consts::PI;
    }

    while value < -std::f64::consts::PI {
        value += 2.0 * std::f64::consts::PI;
    }

    value
}

pub struct IterError<T, E> {
    error: Option<E>,
    _ignore: std::marker::PhantomData<fn(T)>,
}

impl<T, E> IterError<T, E> {
    pub fn new(error: E) -> Self {
        Self {
            error: Some(error),
            _ignore: Default::default(),
        }
    }
}

impl<T, E> Iterator for IterError<T, E> {
    type Item = Result<T, E>;
    fn next(&mut self) -> Option<Self::Item> {
        self.error.take().map(Err)
    }
}
