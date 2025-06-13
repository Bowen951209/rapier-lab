use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use rapier2d::{na::Vector2, prelude::*};

pub struct AppInfo {
    pub screen_rect: Option<Rect>,
    pub show_fps: bool,
    pub show_grid: bool,
}

impl Default for AppInfo {
    fn default() -> Self {
        Self {
            screen_rect: None,
            show_fps: true,
            show_grid: true,
        }
    }
}

pub struct Camera {
    pub center: Pos2,
    pub zoom: f32,
}

pub struct PhysicsApp {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub physics_pipeline: PhysicsPipeline,
    pub gravity: Vector2<Real>,
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

    pub fn transform_to_screen_pos(&self, p: (f32, f32)) -> Pos2 {
        let (mut x, mut y) = (p.0, p.1);
        let screen_rect = self.app_info.screen_rect.unwrap();

        x += self.camera.center.x;
        x *= self.camera.zoom;
        x += screen_rect.width() * 0.5;

        y += self.camera.center.y;
        y *= -self.camera.zoom;
        y += screen_rect.height() * 0.5;

        Pos2::new(x, y)
    }

    fn draw_grid(&self, painter: &egui::Painter) {
        let screen_rect = self.app_info.screen_rect.unwrap();

        let center = self.transform_to_screen_pos(self.camera.center.into());

        let mut delta_y = 0.0;
        while delta_y < screen_rect.height() * 0.5 {
            let up_y = center.y + delta_y;
            painter.line_segment(
                [Pos2::new(0.0, up_y), Pos2::new(screen_rect.width(), up_y)],
                Stroke::new(0.5, Color32::GRAY),
            );

            let up_y = center.y - delta_y;
            painter.line_segment(
                [Pos2::new(0.0, up_y), Pos2::new(screen_rect.width(), up_y)],
                Stroke::new(0.5, Color32::GRAY),
            );

            delta_y += self.camera.zoom;
        }

        let mut delta_x = 0.0;
        while delta_x < screen_rect.width() * 0.5 {
            let right_x = center.x + delta_x;
            painter.line_segment(
                [
                    Pos2::new(right_x, 0.0),
                    Pos2::new(right_x, screen_rect.height()),
                ],
                Stroke::new(0.5, Color32::GRAY),
            );

            let left_x = center.x - delta_x;
            painter.line_segment(
                [
                    Pos2::new(left_x, 0.0),
                    Pos2::new(left_x, screen_rect.height()),
                ],
                Stroke::new(0.5, Color32::GRAY),
            );

            delta_x += self.camera.zoom;
        }
    }

    fn draw_windows(&mut self, ctx: &egui::Context) {
        egui::Window::new("Debug Panel")
            .resizable(true)
            .collapsible(true)
            .show(ctx, |ui| {
                ui.checkbox(&mut self.app_info.show_fps, "Show FPS");
                ui.checkbox(&mut self.app_info.show_grid, "Show Grid");
            });
    }
}

impl eframe::App for PhysicsApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let time = ctx.input(|input| input.time);
        self.fps_counter.update_frame(time);
        self.app_info.screen_rect = Some(ctx.screen_rect());

        egui::CentralPanel::default().show(ctx, |ui| {
            if self.app_info.show_fps {
                ui.heading(format!(
                    "FPS: {}",
                    self.fps_counter.fps.unwrap_or(0.0).round()
                ));
            }

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
                let center =
                    self.transform_to_screen_pos((position.translation.x, position.translation.y));

                match shape {
                    TypedShape::Ball(ball) => {
                        let radius = ball.radius * self.camera.zoom;
                        painter.circle(center, radius, Color32::BLUE, Stroke::NONE);

                        let line_end = position * point![ball.radius, 0.0];
                        let line_end = self.transform_to_screen_pos((line_end.x, line_end.y));
                        painter.line_segment([center, line_end], Stroke::new(1.0, Color32::WHITE));
                    }
                    TypedShape::Cuboid(cuboid) => {
                        let vertices = [
                            position * point!(-cuboid.half_extents.x, -cuboid.half_extents.y),
                            position * point![cuboid.half_extents.x, -cuboid.half_extents.y],
                            position * point![cuboid.half_extents.x, cuboid.half_extents.y],
                            position * point!(-cuboid.half_extents.x, cuboid.half_extents.y),
                        ]
                        .iter()
                        .map(|v| self.transform_to_screen_pos((v.x, v.y)))
                        .collect::<Vec<_>>();

                        let up_right = vertices[2];

                        painter.add(Shape::convex_polygon(
                            vertices,
                            Color32::ORANGE,
                            Stroke::NONE,
                        ));

                        painter.line_segment([center, up_right], Stroke::new(1.0, Color32::WHITE));
                    }
                    _ => {
                        panic!("Unsupported shape");
                    }
                }
            }
        });

        self.draw_windows(ctx);

        ctx.request_repaint();
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
