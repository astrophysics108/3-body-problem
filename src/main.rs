// dependencies
use macroquad::prelude::*;
use ode_solvers::{System, rk4::Rk4};
use nalgebra::{OVector, Const};

// constants
const WIDTH:f32 = 1000.0;
const HEIGHT:f32 = 800.0;
type State = OVector<f32, Const<3>>;

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
        println!("{} {} {}", m1, m2, m3);
    }
}

#[macroquad::main(conf())]
async fn main() {
    loop {
        clear_background(WHITE);

        let the_whole_system = ThreeBody{m1:10.0, m2:10.0, m3:10.0};
        next_frame().await;
    }
}

