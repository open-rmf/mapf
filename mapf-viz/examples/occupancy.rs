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
    Application, Alignment, Element, Length, Command, Column, Row, Text, Container,
    slider::{self, Slider},
    text_input::{self, TextInput},
    button::{self, Button},
    executor,
};
use mapf::occupancy::{Grid, SparseGrid};
use mapf_viz::{
    SparseGridOccupancyVisual, InfiniteGrid,
    spatial_canvas::SpatialCanvas,
};
use mapf_viz::spatial_layers;

spatial_layers!(GridLayers<Message>: InfiniteGrid, SparseGridOccupancyVisual);

struct App {
    robot_size_slider: slider::State,
    max_robot_radius: f32,
    input: text_input::State,
    input_value: String,
    reset_size_button: button::State,
    canvas: SpatialCanvas<Message, GridLayers>,
}

impl App {
    const TICK_COUNT: u32 = 1000;
}

impl Application for App {
    type Message = Message;
    type Executor = executor::Default;
    type Flags = ();

    fn new(_flags: Self::Flags) -> (Self, Command<Self::Message>) {
        let cell_size = 1.0_f32;
        let robot_radius = 0.75_f32;

        let mut canvas = SpatialCanvas::new(
            GridLayers{
                layers: (
                    InfiniteGrid::new(cell_size),
                    SparseGridOccupancyVisual::new(
                        SparseGrid::new(cell_size as f64),
                        robot_radius
                    ),
                )
            }
        );
        canvas.zoom = 20.0;

        (
            Self{
                robot_size_slider: slider::State::new(),
                max_robot_radius: 10_f32*cell_size,
                input: text_input::State::default(),
                input_value: String::new(),
                reset_size_button: button::State::new(),
                canvas
            },
            Command::none()
        )
    }

    fn title(&self) -> String {
        "Test App".to_owned()
    }

    fn update(
        &mut self,
        message: Self::Message
    ) -> Command<Self::Message> {
        match message {
            Message::TextInputChanged(value) => {
                self.input_value = value;
                if let Ok(radius) = self.input_value.parse::<f32>() {
                    if radius > 0.0 {
                        self.max_robot_radius = radius;

                        let current_robot_radius = self.canvas.program.layers.1.visibility().agent_radius() as f32;
                        if self.max_robot_radius < current_robot_radius {
                            self.canvas.program.layers.1.set_robot_radius(radius);
                            self.canvas.cache.clear();
                        }
                    }
                }
            },
            Message::RobotSizeSlide(value) => {
                let min_size = self.canvas.program.layers.1.grid().cell_size() as f32/10_f32;
                let new_robot_radius = value as f32 * (self.max_robot_radius - min_size)/(Self::TICK_COUNT as f32) + min_size;
                self.canvas.program.layers.1.set_robot_radius(new_robot_radius);
                self.canvas.cache.clear();
            },
            Message::ResetView => {
                self.canvas.fit_to_bounds();
            },
        }

        Command::none()
    }

    fn view(&mut self) -> Element<Self::Message> {
        let mut content = Column::new()
            .spacing(20)
            .align_items(Alignment::Start)
            .width(Length::Fill)
            .height(Length::Fill);

        let min_size = self.canvas.program.layers.1.grid().cell_size() as f32/10_f32;
        let robot_radius = self.canvas.program.layers.1.visibility().agent_radius() as f32;
        let robot_tick = ((robot_radius - min_size)/(self.max_robot_radius - min_size) * Self::TICK_COUNT as f32) as u32;
        let file_row = Row::new()
            .spacing(20)
            .align_items(Alignment::Center)
            .width(Length::Fill)
            .push(
                Button::new(
                    &mut self.reset_size_button,
                    iced::Text::new("Reset View")
                ).on_press(Message::ResetView)
            )
            .push(
                Slider::new(
                    &mut self.robot_size_slider,
                    0..=Self::TICK_COUNT,
                    robot_tick,
                    Message::RobotSizeSlide,
                )
            )
            .push(Text::new(format!("{:.2}", robot_radius)))
            .push(
                TextInput::new(
                    &mut self.input,
                    "Max Robot Radius",
                    &mut self.input_value,
                    Message::TextInputChanged
                )
                .padding(10)
                .width(Length::Fill)
            );

        let instruction_row = Row::<Message>::new()
            .spacing(40)
            .align_items(Alignment::Start)
            .width(Length::Shrink)
            .push(Text::new("Left click: Add occupancy"))
            .push(Text::new("Shift + Left click: Remove occupancy"))
            .push(Text::new("Middle click: Pan view"))
            .push(Text::new("Scroll: Zoom"));

        content = content
            .push(file_row)
            .push(instruction_row)
            .push(self.canvas.view());

        Container::new(content)
            .width(Length::Fill)
            .height(Length::Fill)
            .padding(5)
            .center_x()
            .center_y()
            .into()
    }
}

#[derive(Debug, Clone)]
enum Message {
    TextInputChanged(String),
    RobotSizeSlide(u32),
    ResetView,
}

fn main() -> iced::Result {
    App::run(iced::Settings::default())
}

mod style {
    use iced::container;
    pub struct Pane;
    impl container::StyleSheet for Pane {
        fn style(&self) -> container::Style {
            container::Style{
                text_color: None,
                background: None,
                border_radius: 0.0,
                border_width: 3.0,
                border_color: iced::Color{r: 0.9, g: 0.9, b: 0.9, a: 0.9}
            }
        }
    }
}
