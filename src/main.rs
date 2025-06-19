use rapier_lab::get_app_by_env_args;

fn main() {
    get_app_by_env_args()
        .run("Ball and Cuboid")
        .expect("Failed to run the frame");
}
