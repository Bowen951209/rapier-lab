use std::collections::HashMap;

use eframe::egui;
use rapier2d::{na, prelude::*};

pub struct AppInfo {
    pub screen_rect: Option<egui::Rect>,
    pub show_fps: bool,
    pub show_grid: bool,
    pub last_mouse_pos: Option<egui::Pos2>,
    pub grid_space: Option<f32>,
    pub hovered_body: Option<RigidBodyHandle>,
    /// The HashMap between rigid body handles and whether the corresponding window is open.
    /// Note that this map will get larger and larger as users opened many windows even if
    /// they closed them afterwards.
    pub body_window_open_states: HashMap<RigidBodyHandle, bool>,
}

impl Default for AppInfo {
    fn default() -> Self {
        Self {
            screen_rect: None,
            show_fps: true,
            show_grid: true,
            last_mouse_pos: None,
            grid_space: None,
            hovered_body: None,
            body_window_open_states: HashMap::new(),
        }
    }
}

pub struct Camera {
    pub center: egui::Pos2,
    pub zoom: f32,
}

pub struct PhysicsApp {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub physics_pipeline: PhysicsPipeline,
    pub gravity: na::Vector2<Real>,
    pub integration_parameters: IntegrationParameters,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub physics_hooks: (),
    pub event_handler: (),
    pub camera: Camera,
    pub fps_counter: FpsCounter,
    pub app_info: AppInfo,
}

impl PhysicsApp {
    pub fn run(self, app_name: &str) -> eframe::Result {
        let native_options = eframe::NativeOptions::default();
        eframe::run_native(app_name, native_options, Box::new(|_cc| Ok(Box::new(self))))
    }

    pub fn world_to_screen_x(&self, x: f32) -> f32 {
        let screen_rect = self.app_info.screen_rect.unwrap();
        (x + self.camera.center.x) * self.camera.zoom + screen_rect.width() * 0.5
    }

    pub fn world_to_screen_y(&self, y: f32) -> f32 {
        let screen_rect = self.app_info.screen_rect.unwrap();
        (y + self.camera.center.y) * -self.camera.zoom + screen_rect.height() * 0.5
    }

    pub fn world_to_screen(&self, p: (f32, f32)) -> egui::Pos2 {
        egui::Pos2::new(self.world_to_screen_x(p.0), self.world_to_screen_y(p.1))
    }

    pub fn screen_to_world(&self, p: egui::Pos2) -> (f32, f32) {
        let (mut x, mut y) = (p.x, p.y);
        let screen_rect = self.app_info.screen_rect.unwrap();

        x -= screen_rect.width() * 0.5;
        x /= self.camera.zoom;
        x -= self.camera.center.x;

        y -= screen_rect.height() * 0.5;
        y /= -self.camera.zoom;
        y -= self.camera.center.y;

        (x, y)
    }

    pub fn shape_type_to_str(typed_shape: TypedShape) -> &'static str {
        match typed_shape {
            TypedShape::Ball(_) => "Ball",
            TypedShape::Cuboid(_) => "Cuboid",
            TypedShape::Capsule(_) => "Capsule",
            TypedShape::Segment(_) => "Segment",
            TypedShape::Triangle(_) => "Triangle",
            TypedShape::Voxels(_) => "Voxels",
            TypedShape::TriMesh(_) => "TriMesh",
            TypedShape::Polyline(_) => "Polyline",
            TypedShape::HalfSpace(_) => "HalfSpace",
            TypedShape::HeightField(_) => "HeightField",
            TypedShape::Compound(_) => "Compound",
            TypedShape::ConvexPolygon(_) => "ConvexPolygon",
            TypedShape::RoundCuboid(_) => "RoundCuboid",
            TypedShape::RoundTriangle(_) => "RoundTriangle",
            TypedShape::RoundConvexPolygon(_) => "RoundConvexPolygon",
            TypedShape::Custom(_) => "Custom",
        }
    }

    fn mouse_hover_body(&self, mouse_pos: egui::Pos2) -> Option<RigidBodyHandle> {
        let world_pos = self.screen_to_world(mouse_pos);
        let world_pos = point![world_pos.0, world_pos.1];
        let filter = QueryFilter::new();

        let mut body_handle = None;

        self.query_pipeline.intersections_with_point(
            &self.rigid_body_set,
            &self.collider_set,
            &world_pos,
            filter,
            |handle| {
                let collider = self.collider_set.get(handle).unwrap();
                body_handle = collider.parent();

                // We only find first body, return false to stop searching
                false
            },
        );

        body_handle
    }

    fn draw_grid(&mut self, painter: &egui::Painter) {
        const GRID_SPACE_FREQ: f32 = 2.0;
        let grid_spacing_world = 10f32
            .powf(-GRID_SPACE_FREQ * (self.camera.zoom.log10() - 2.0).round() / GRID_SPACE_FREQ);
        self.app_info.grid_space = Some(grid_spacing_world);
        let screen_rect = self.app_info.screen_rect.unwrap();

        // Screen corners to world space
        let (min_world_x, min_world_y) = self.screen_to_world(screen_rect.min);
        let (max_world_x, max_world_y) = self.screen_to_world(screen_rect.max);

        // Find start position (start from the nearest integer multiple)
        let start_x = (min_world_x / grid_spacing_world).floor() * grid_spacing_world;
        let end_x = (max_world_x / grid_spacing_world).ceil() * grid_spacing_world;
        let start_y = (max_world_y / grid_spacing_world).floor() * grid_spacing_world;
        let end_y = (min_world_y / grid_spacing_world).ceil() * grid_spacing_world;

        // Draw vertical lines
        let mut x = start_x;
        while x < end_x {
            let screen_x = self.world_to_screen_x(x);
            painter.line_segment(
                [
                    egui::pos2(screen_x, screen_rect.top()),
                    egui::pos2(screen_x, screen_rect.bottom()),
                ],
                (1.0, egui::Color32::LIGHT_GRAY),
            );
            x += grid_spacing_world;
        }

        // Draw horizontal lines
        let mut y = start_y;
        while y < end_y {
            let screen_y = self.world_to_screen_y(y);
            painter.line_segment(
                [
                    egui::pos2(screen_rect.left(), screen_y),
                    egui::pos2(screen_rect.right(), screen_y),
                ],
                (1.0, egui::Color32::LIGHT_GRAY),
            );
            y += grid_spacing_world;
        }
    }

    fn draw_windows(&mut self, ctx: &egui::Context) {
        // Debug Panel
        egui::Window::new("Debug Panel")
            .default_pos({
                let screen_rect = self.app_info.screen_rect.unwrap();
                screen_rect.right_top()
            })
            .collapsible(true)
            .default_size(egui::vec2(200.0, 100.0))
            .show(ctx, |ui| {
                ui.checkbox(&mut self.app_info.show_fps, "Show FPS");
                ui.checkbox(&mut self.app_info.show_grid, "Show Grid");
            });

        // Body Monitors
        for (body_handle, is_open) in &mut self.app_info.body_window_open_states {
            if !*is_open {
                continue;
            }

            let body_index = body_handle.into_raw_parts().0;
            egui::Window::new(format!("Body index {}", body_index))
                .collapsible(true)
                .open(is_open)
                .show(ctx, |ui| {
                    let body = self.rigid_body_set.get(*body_handle).unwrap();
                    ui.label(format!(
                        "Position: ({:.2}, {:.2})",
                        body.translation().x,
                        body.translation().y
                    ));
                    ui.label(format!("Angle: {}", body.rotation().angle()));
                    ui.label(format!("Mass: {}", body.mass()));
                    ui.label(format!(
                        "Linear Velocity: ({:.2}, {:.2}), Magnitude: {:.2}",
                        body.linvel().x,
                        body.linvel().y,
                        body.linvel().magnitude()
                    ));

                    let colliders = body
                        .colliders()
                        .iter()
                        .map(|handle| self.collider_set.get(*handle).unwrap())
                        .collect::<Vec<_>>();

                    egui::CollapsingHeader::new("Colliders")
                        .default_open(true)
                        .show(ui, |ui| {
                            for (i, collider) in colliders.iter().enumerate() {
                                egui::CollapsingHeader::new(format!("Collider {}", i))
                                    .default_open(true)
                                    .show(ui, |ui| {
                                        ui.label(format!(
                                            "Position: ({:.2}, {:.2})",
                                            collider.translation().x,
                                            collider.translation().y
                                        ));
                                        ui.label(format!("Angle: {}", collider.rotation().angle()));
                                        ui.label(format!("Mass: {}", collider.mass()));
                                        ui.label(format!(
                                            "Restitution: {}",
                                            collider.restitution()
                                        ));
                                        ui.label(format!("Friction: {}", collider.friction()));
                                        ui.label(format!(
                                            "Shape: {}",
                                            Self::shape_type_to_str(
                                                collider.shape().as_typed_shape()
                                            )
                                        ));
                                    });
                            }
                        });
                });
        }
    }
}

impl eframe::App for PhysicsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let (time, mouse_pos, mouse_scroll_delta) = ctx.input(|input| {
            (
                input.time,
                input.pointer.hover_pos(),
                input.smooth_scroll_delta.y,
            )
        });
        self.fps_counter.update_frame(time);
        self.app_info.screen_rect = Some(ctx.screen_rect());
        if let Some(mouse_pos) = mouse_pos {
            self.app_info.hovered_body = self.mouse_hover_body(mouse_pos);
        }

        // Zoom the camera when mouse wheel is scrolled.
        // Use a exponetial increment to make it feel more natural.
        const ZOOM_SPEED: f32 = 0.01;
        self.camera.zoom *= (mouse_scroll_delta * ZOOM_SPEED).exp();
        if self.camera.zoom < 1.0 {
            self.camera.zoom = 1.0;
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            if self.app_info.show_fps {
                ui.heading(format!(
                    "FPS: {}\n\
                    Camera Zoom: x{:.2}\n\
                    Grid Space: {:.2}",
                    self.fps_counter.fps.unwrap_or(0.0).round(),
                    self.camera.zoom,
                    self.app_info.grid_space.unwrap_or(0.0),
                ));
            }

            // Mouse response.
            // Will trigger only when mouse is in the canvas.
            // Mouse on the UI will not trigger this.
            let response = ui.allocate_response(ui.available_size(), egui::Sense::click_and_drag());
            let mut cursor_icon = egui::CursorIcon::Default;
            if response.dragged() {
                // If mouse is dragged, move the camera and set icon to Grabbing.
                if let Some(last_pos) = self.app_info.last_mouse_pos {
                    let mut delta = mouse_pos.unwrap() - last_pos;
                    delta.y = -delta.y;
                    self.camera.center += delta / self.camera.zoom;
                }

                cursor_icon = egui::CursorIcon::Grabbing;
            } else if response.hovered() {
                if self.app_info.hovered_body.is_some() {
                    // If mouse is hovering on a body, set icon to PointingHand.
                    cursor_icon = egui::CursorIcon::PointingHand;
                } else {
                    // If mouse is hovering on the canvas, set icon to Grab.
                    cursor_icon = egui::CursorIcon::Grab;
                }
            }
            if response.clicked_by(egui::PointerButton::Primary) {
                // If a body is selected, open its window.
                if let Some(body) = self.app_info.hovered_body {
                    self.app_info.body_window_open_states.insert(body, true);
                }
            }
            response.on_hover_cursor(cursor_icon);

            // Step the physics.
            self.physics_pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                &mut self.ccd_solver,
                Some(&mut self.query_pipeline),
                &self.physics_hooks,
                &self.event_handler,
            );

            // Draw the world.
            let painter = ui.painter();
            if self.app_info.show_grid {
                self.draw_grid(painter);
            }

            for (_body_handle, body) in self.rigid_body_set.iter() {
                if body.colliders().len() > 1 {
                    panic!("Multiple colliders per body not supported");
                }

                let collider = self.collider_set.get(body.colliders()[0]).unwrap();
                let shape = collider.shape().as_typed_shape();
                let position = collider.position();
                let center = self.world_to_screen((position.translation.x, position.translation.y));

                match shape {
                    TypedShape::Ball(ball) => {
                        let radius = ball.radius * self.camera.zoom;
                        painter.circle(center, radius, egui::Color32::BLUE, egui::Stroke::NONE);

                        let line_end = position * point![ball.radius, 0.0];
                        let line_end = self.world_to_screen((line_end.x, line_end.y));
                        painter.line_segment(
                            [center, line_end],
                            egui::Stroke::new(1.0, egui::Color32::WHITE),
                        );
                    }
                    TypedShape::Cuboid(cuboid) => {
                        let vertices = [
                            position * point!(-cuboid.half_extents.x, -cuboid.half_extents.y),
                            position * point![cuboid.half_extents.x, -cuboid.half_extents.y],
                            position * point![cuboid.half_extents.x, cuboid.half_extents.y],
                            position * point!(-cuboid.half_extents.x, cuboid.half_extents.y),
                        ]
                        .iter()
                        .map(|v| self.world_to_screen((v.x, v.y)))
                        .collect::<Vec<_>>();

                        let up_right = vertices[2];

                        painter.add(egui::Shape::convex_polygon(
                            vertices,
                            egui::Color32::ORANGE,
                            egui::Stroke::NONE,
                        ));

                        painter.line_segment(
                            [center, up_right],
                            egui::Stroke::new(1.0, egui::Color32::WHITE),
                        );
                    }
                    _ => {
                        panic!("Unsupported shape");
                    }
                }
            }
        });

        self.draw_windows(ctx);

        self.app_info.last_mouse_pos = mouse_pos;

        ctx.request_repaint(); // make it a loop
    }
}

#[derive(Default)]
pub struct FpsCounter {
    last_time: Option<f64>,
    frame_count: usize,
    fps: Option<f32>,
}

impl FpsCounter {
    fn update_frame(&mut self, current_time: f64) {
        self.frame_count += 1;

        match self.last_time {
            Some(last_time) => {
                let delta_time = current_time - last_time;
                if delta_time > 1.0 {
                    self.fps = Some((self.frame_count as f64 / delta_time) as f32);
                    self.frame_count = 0;
                    self.last_time = Some(current_time);
                }
            }
            None => self.last_time = Some(current_time),
        }
    }
}
