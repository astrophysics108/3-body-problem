// dependencies
use macroquad::prelude::*;
use ode_solvers::{System, rk4::Rk4};
use nalgebra::{OVector, Const};

// constants
const WIDTH:f32 = 1000.0;
const HEIGHT:f32 = 800.0;
type State = OVector<f32, Const<9>>;

// window configuration substance
fn conf() -> Conf {
    Conf {  
        window_title: "three-body-problem".to_owned(),
        window_width: (WIDTH * 2.0) as i32,
        window_height: (HEIGHT * 2.0) as i32,
        high_dpi: true,
        fullscreen: false,
        window_resizable: false,
        icon: None,
        platform: Default::default(),
        sample_count: 1,
    }
}

// the 3 body structure
struct ThreeBody {
    m1:f32,
    m2:f32,
    m3:f32,
}

// define the system
impl System<f32, State> for ThreeBody {
    fn system(&self, _t:f32, _y:&State, _dy:&mut State) {
        
        // key variables
        let m1 = self.m1;
        let m2 = self.m2;
        let m3 = self.m3;
    }
}

#[macroquad::main(conf())]
async fn main() {

    // state variables
    let mut x1:f32 = 0.0;
    let mut x2:f32 = 0.0;
    let mut x3:f32 = 0.0;
    let mut y1:f32 = 0.0;
    let mut y2:f32 = 0.0;
    let mut y3:f32 = 0.0;
    let mut v1:f32 = 0.0;
    let mut v2:f32 = 0.0;
    let mut v3:f32 = 0.0;


    loop {

        clear_background(WHITE);

        let the_whole_system = ThreeBody{m1:10.0, m2:10.0, m3:10.0};
        let dt: f32= 0.01; 
        let t_start: f32 = 0.0;
        let t_end: f32 = 0.005;
        // initial y vector which we use the system to differentiate and update
        let y0 = State::from([v1, v2, v3, x1, x2, x3, y1, y2, y3]);

        let mut delta = Rk4::new(the_whole_system, t_start, y0, t_end, dt);

        delta.integrate().unwrap();
        let new_state = delta.y_out().last().unwrap();
        v1 = new_state[0];
        v2 = new_state[1];
        v3 = new_state[2];
        x1 = new_state[3];
        x2 = new_state[4];
        x3 = new_state[5];
        y1 = new_state[6];
        y2 = new_state[7];
        y3 = new_state[8];
        next_frame().await;
    }
}

