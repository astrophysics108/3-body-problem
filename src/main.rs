// dependencies
use macroquad::prelude::*;
use ode_solvers::{System, rk4::Rk4};
use nalgebra::{OVector, Const};

// constants
const WIDTH:f32 = 1000.0;
const HEIGHT:f32 = 800.0;
type State = OVector<f32, Const<12>>;

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
#[derive(Clone, Copy)]
struct ThreeBody {
    m1:f32,
    m2:f32,
    m3:f32,
    g: f32,
    epsilon: f32,
}

impl ThreeBody {
    #[inline]
    fn accel_pair(dx: f32, dy: f32, mj: f32, g: f32, eps2: f32) -> (f32, f32) {
        let r2 = dx*dx + dy*dy + eps2;
        let inv_r = r2.sqrt().recip();
        let inv_r3 = inv_r * inv_r * inv_r;
        let s = g * mj * inv_r3;
        (s * dx, s * dy)
    }
}

// define the system
impl System<f32, State> for ThreeBody {
    fn system(&self, _t:f32, y:&State, mut dy:&mut State) {

        let epsilon2 = self.epsilon * self.epsilon; 

        let x1 = y[0];
        let x2 = y[1];
        let x3 = y[2];
        let y1 = y[3];
        let y2 = y[4];
        let y3 = y[5];

        let vx1 = y[6];
        let vx2 = y[7];
        let vx3 = y[8];
        let vy1 = y[9];
        let vy2 = y[10];
        let vy3 = y[11];

        dy[0] = vx1;
        dy[1] = vx2;
        dy[2] = vx3;
        dy[3] = vy1;
        dy[4] = vy2;
        dy[5] = vy3;

        // accelerations

        let (dx12, dy12) = (x2-x1, y2-y1);
        let (ax12, ay12) = ThreeBody::accel_pair(dx12, dy12, self.m2, self.g, epsilon2);
        let (dx13, dy13) = (x3-x1, y3-y1);
        let (ax13, ay13) = ThreeBody::accel_pair(dx13, dy13, self.m3, self.g, epsilon2);
        let (ax1, ay1) = (ax12 + ax13, ay12 + ay13);

        let (dx21, dy21) = (x1-x2, y1-y2);
        let (ax21, ay21) = ThreeBody::accel_pair(dx21, dy21, self.m1, self.g, epsilon2);
        let (dx23, dy23) = (x3-x2, y3-y2);
        let (ax23, ay23) = ThreeBody::accel_pair(dx23, dy23, self.m3, self.g, epsilon2);
        let (ax2, ay2) = (ax21 + ax23, ay21 + ay23);

        let (dx31, dy31) = (x1-x3, y1-y3);
        let (ax31, ay31) = ThreeBody::accel_pair(dx31, dy31, self.m1, self.g, epsilon2);
        let (dx32, dy32) = (x2-x3, y2-y3);
        let (ax32, ay32) = ThreeBody::accel_pair(dx32, dy32, self.m2, self.g, epsilon2);
        let (ax3, ay3) = (ax31 + ax32, ay31 + ay32);

        dy[6] = ax1;
        dy[7] = ax2;
        dy[8] = ax3;
        dy[9] = ay1;
        dy[10] = ay2;
        dy[11] = ay3;
        

    }
}

#[macroquad::main(conf())]
async fn main() {

    // state variables
    let mut x1:f32 = 0.3;
    let mut x2:f32 = 0.5;
    let mut x3:f32 = 0.8;
    let mut y1:f32 = 0.3;
    let mut y2:f32 = 0.5;
    let mut y3:f32 = 0.7;

    let vx1:f32 = 0.0;
    let vx2:f32 = 0.0;
    let vx3:f32 = 0.0;
    let vy1:f32 = 0.0;
    let vy2:f32 = 0.0;
    let vy3:f32 = 0.0;

    let mut state = State::from_vec(vec![
        x1, x2, x3, y1, y2, y3, vx1, vx2, vx3, vy1, vy2, vy3
    ]);

    let mut t = 0.0;
    let dt: f32= 0.001; 

    loop {

        clear_background(WHITE);

        let the_whole_system = ThreeBody{m1:10.0, m2:10.0, m3:10.0, g:1.0, epsilon:0.001};
        // initial y vector which we use the system to differentiate and update

        let mut delta = Rk4::new(the_whole_system, t, state.clone(), t + dt, dt);

        delta.integrate().unwrap();
        let new_state = delta.y_out().last().unwrap();
        t += dt;
        x1 = new_state[0];
        x2 = new_state[1];
        x3 = new_state[2];
        y1 = new_state[3];
        y2 = new_state[4];
        y3 = new_state[5];

        // vx1 = new_state[6];
        // vx2 = new_state[7];
        // vx3 = new_state[8];
        // vy1 = new_state[9];
        // vy2 = new_state[10];
        // vy3 = new_state[11];

        let to_screen = |x:f32,y:f32| (x*WIDTH, y*HEIGHT);

        let (sx1, sy1) = to_screen(x1, y1);
        let (sx2, sy2) = to_screen(x2, y2);
        let (sx3, sy3) = to_screen(x3, y3);

        draw_circle(sx1, sy1, 8.0, RED);
        draw_circle(sx2, sy2, 8.0, BLUE);
        draw_circle(sx3, sy3, 8.0, GREEN);

        state = *new_state;

        println!("{} {} {} {} {} {}", x1, y1, x2, y2, x3, y3);

        next_frame().await;
    }
}

