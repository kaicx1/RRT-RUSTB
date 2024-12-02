use kiss3d::light::Light;
use kiss3d::nalgebra as na;
use kiss3d::ncollide3d;
use kiss3d::window::Window;
use na::{Isometry3, Vector3};
use ncollide3d::query;
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid};
use std::time::Duration;
use kiss3d::scene::SceneNode;
use rand::distributions::{Distribution, Uniform};

struct CollisionProblem {
    obstacles: Vec<Cuboid<f32>>,  // Multiple obstacles
    ball: Ball<f32>,
}

impl CollisionProblem {
    fn is_feasible(&self, point: &[f64]) -> bool {
        let ball_pos = Isometry3::new(
            Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
            na::zero(),
        );
        let prediction = 0.1;

        // Check if the ball is disjoint from all obstacles
        for (i, obstacle) in self.obstacles.iter().enumerate() {
            let cuboid_pos = Isometry3::new(
                Vector3::new((i as f32) - 1.0, 0.0, 0.0), // Spacing obstacles out
                na::zero(),
            );
            let contact_state = query::proximity(
                &ball_pos,
                &self.ball,
                &cuboid_pos,
                obstacle,
                prediction,
            );
            if contact_state != Proximity::Disjoint {
                return false;
            }
        }
        true
    }

    fn random_sample(&self) -> Vec<f64> {
        let between = Uniform::new(-4.0, 4.0);
        let mut rng = rand::thread_rng();
        vec![
            between.sample(&mut rng),
            between.sample(&mut rng),
            between.sample(&mut rng),
        ]
    }
}

fn main() {
    let mut window = Window::new("rrt test");
    window.set_light(Light::StickToCamera);

    // Define multiple obstacles with different sizes
    let obstacles = vec![
        Cuboid::new(Vector3::new(0.05f32, 0.3, 0.15)),
        Cuboid::new(Vector3::new(0.1f32, 0.1, 0.1)),
        Cuboid::new(Vector3::new(0.1f32, 0.05, 0.3)),
    ];

    let p = CollisionProblem {
        obstacles,
        ball: Ball::new(0.05f32),
    };

    // Render obstacles
    for (i, obstacle) in p.obstacles.iter().enumerate() {
        let mut obstacle_node = window.add_cube(
            obstacle.half_extents[0] * 2.0,
            obstacle.half_extents[1] * 2.0,
            obstacle.half_extents[2] * 2.0,
        );
        obstacle_node.set_color(0.0, 0.0, 1.0);
        let pos = Isometry3::new(Vector3::new((i as f32) - 1.0, 0.0, 0.0), na::zero());
        obstacle_node.set_local_transformation(pos);
    }

    // Start and goal markers
    let mut cs = window.add_cube(0.05, 0.05, 0.05);
    cs.set_color(0.0, 1.0, 0.0);
    let mut cg = window.add_cube(0.05, 0.05, 0.05);
    cg.set_color(1.0, 0.0, 0.0);

    // Ball
    let mut c2 = window.add_sphere(p.ball.radius);
    c2.set_color(1.0, 1.0, 1.0);

    let start = [1.5f64, 0.0, 0.0];
    let goal = [-1.2f64, 0.0, 0.0];
    let start_pos = Isometry3::new(
        Vector3::new(start[0] as f32, start[1] as f32, start[2] as f32),
        na::zero(),
    );
    let goal_pos = Isometry3::new(
        Vector3::new(goal[0] as f32, goal[1] as f32, goal[2] as f32),
        na::zero(),
    );

    cs.set_local_transformation(start_pos);
    cg.set_local_transformation(goal_pos);

    let mut path = vec![];
    let mut index = 0;
    let mut frame_count = 0;
	let speed_factor = 20;
	let mut trail_markers: Vec<SceneNode> = vec![];

    while window.render() {
        if index == path.len() {
            path = rrt::dual_rrt_connect(
                &start,
                &goal,
                |x: &[f64]| p.is_feasible(x),
                || p.random_sample(),
                0.01,
                2000,
            )
            .unwrap();
            rrt::smooth_path(&mut path, |x: &[f64]| p.is_feasible(x), 0.01, 100);
            index = 0;

		  for marker in trail_markers.iter_mut() {
			window.remove_node(marker);
		 }
		 trail_markers.clear();
        }
        
        if frame_count % speed_factor == 0 {
		let point = &path[index % path.len()];
		let pos = Isometry3::new(
		    Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
		    na::zero(),
		);
		c2.set_local_transformation(pos);

		let mut trail_marker = window.add_sphere(0.01); // Small sphere for trail
		trail_marker.set_local_transformation(pos);
		trail_marker.set_color(0.0, 0.5, 1.0); // Color for the trail
		trail_markers.push(trail_marker); // Store the marker

		index += 1;
		std::thread::sleep(Duration::from_millis(50));
	 }
    frame_count += 1;
    }
}
