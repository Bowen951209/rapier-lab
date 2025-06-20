use rapier_lab::get_app_by_env_args;

fn main() {
    get_app_by_env_args()
        .run("Rapier Lab")
        .expect("Failed to run the frame");
}
