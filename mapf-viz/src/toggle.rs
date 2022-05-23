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

use iced::{
    canvas::event::Event,
    mouse, keyboard
};
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Toggle {
    On,
    Off,
    NoChange
}

impl Toggle {
    fn change_to(&mut self, value: Toggle) -> Toggle {
        if *self == value {
            return Toggle::NoChange;
        }

        *self = value;
        return value;
    }
}

pub trait Toggler {
    fn toggle(&mut self, event: Event) -> Toggle;
    fn state(&self) -> Toggle;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DragToggler {
    buttons: HashMap<keyboard::Modifiers, mouse::Button>,
    active_modifiers: keyboard::Modifiers,
    state: Toggle,
}

impl DragToggler {

    pub fn new(
        main_button: Option<mouse::Button>,
        alt_button: Option<(keyboard::Modifiers, mouse::Button)>,
    ) -> Self {
        let mut buttons = HashMap::new();
        if let Some(main_button) = main_button {
            buttons.insert(keyboard::Modifiers::empty(), main_button);
        }

        if let Some(alt_button) = alt_button {
            buttons.insert(alt_button.0, alt_button.1);
        }

        Self{buttons, active_modifiers: keyboard::Modifiers::empty(), state: Toggle::Off}
    }

    fn button_matches(&self, button: mouse::Button) -> bool {

        if let Some(expected_button) = self.buttons.get(&self.active_modifiers) {
            if *expected_button == button {
                return true;
            }
        }

        return false;
    }
}

impl Default for DragToggler {
    fn default() -> Self {
        Self::new(
            Some(mouse::Button::Middle),
            Some((keyboard::Modifiers::CTRL, mouse::Button::Left)),
        )
    }
}

impl Toggler for DragToggler {
    fn toggle(&mut self, event: Event) -> Toggle {
        match event {
            Event::Mouse(event) => {
                if let mouse::Event::ButtonPressed(button) = event {
                    if self.button_matches(button) {
                        return self.state.change_to(Toggle::On);
                    } else {
                        return self.state.change_to(Toggle::Off);
                    }
                }

                if let mouse::Event::ButtonReleased(button) = event {
                    if self.button_matches(button) {
                        return self.state.change_to(Toggle::Off);
                    }
                }
            },
            Event::Keyboard(event) => {
                if let keyboard::Event::ModifiersChanged(modifiers) = event {
                    if modifiers != self.active_modifiers {
                        self.active_modifiers = modifiers;
                        return self.state.change_to(Toggle::Off);
                    }
                }
            }
        }

        return Toggle::NoChange;
    }

    fn state(&self) -> Toggle {
        return self.state;
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FillToggler<On: Toggler, Off: Toggler> {
    on: On,
    off: Off,
    state: Toggle,
}

impl<On: Toggler, Off: Toggler> FillToggler<On, Off> {
    pub fn new(
        on: On,
        off: Off,
    ) -> Self {
        Self{on, off, state: Toggle::NoChange}
    }
}

impl<On: Toggler, Off: Toggler> Toggler for FillToggler<On, Off> {
    fn toggle(&mut self, event: Event) -> Toggle {
        self.on.toggle(event);
        self.off.toggle(event);

        if self.on.state() == self.off.state() {
            // If on and off are equal, then we cannot choose between filling or
            // erasing, so we will say that neither should be done.
            self.state = Toggle::NoChange;
        } else if self.on.state() == Toggle::On {
            self.state = Toggle::On;
        } else if self.off.state() == Toggle::On {
            self.state = Toggle::Off;
        }

        return self.state;
    }

    fn state(&self) -> Toggle {
        return self.state;
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeyToggler {
    key: keyboard::KeyCode,
    modifiers: Option<keyboard::Modifiers>,
    state: Toggle,
}

impl KeyToggler {
    pub fn for_key(key: keyboard::KeyCode) -> Self {
        Self{
            key,
            modifiers: None,
            state: Toggle::Off,
        }
    }

    pub fn for_key_with_modifiers(
        key: keyboard::KeyCode,
        modifiers: keyboard::Modifiers
    ) -> Self {
        Self{
            key,
            modifiers: Some(modifiers),
            state: Toggle::Off,
        }
    }

    pub fn key_toggle(&mut self, event: keyboard::Event) -> Toggle {
        if let keyboard::Event::KeyPressed{key_code, modifiers} = event {
            if let Some(expected_modifiers) = self.modifiers {
                if expected_modifiers != modifiers {
                    return self.state.change_to(Toggle::Off);
                }
            }

            if key_code == self.key {
                return self.state.change_to(Toggle::On);
            }
        }

        if let keyboard::Event::KeyReleased{key_code, ..} = event {
            if key_code == self.key {
                return self.state.change_to(Toggle::Off);
            }
        }

        if let Some(expected_modifiers) = self.modifiers {
            if let keyboard::Event::ModifiersChanged(modifiers) = event {
                if expected_modifiers != modifiers {
                    return self.state.change_to(Toggle::Off);
                }
            }
        }

        return Toggle::NoChange;
    }
}

impl Toggler for KeyToggler {
    fn toggle(&mut self, event: Event) -> Toggle {
        if let Event::Keyboard(event) = event {
            return self.key_toggle(event);
        }

        return Toggle::NoChange;
    }

    fn state(&self) -> Toggle {
        self.state
    }
}
