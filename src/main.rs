use actix::prelude::*;
use kiss3d::nalgebra as na;
use futures_util::future::join_all;
use std::path::Path;

const NUM_BOIDS: usize = 100;
const BOID_MODEL_PATH: &str = "boid_model";
const BOID_OBJ_FILE: &str = "boid.obj";

struct Boid {
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    delta_v: na::Vector3<f32>,
    flock: Option< Vec< Option<Addr<Boid>> > >,
    graphic: Option< kiss3d::scene::SceneNode >,
}

impl Boid {
    fn new(pos: na::Vector3<f32>, vel: na::Vector3<f32>) -> Self {
        Boid {
            position: pos,
            velocity: vel,
            delta_v: na::Vector3::default(),
            flock: None,
            graphic: None,
        }
    }

    fn observe_boid(&mut self, obs: BoidDataMsg) {
        unimplemented!();
    }
}

impl Actor for Boid {
    type Context = Context<Self>;
}

#[derive(Clone, Copy, Message)]
#[rtype(result = "()")]
struct BoidDataMsg {
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
}

impl Handler<BoidDataMsg> for Boid {
    type Result = ();

    fn handle(&mut self, msg: BoidDataMsg, _ctx: &mut Self::Context) {
        self.observe_boid(msg);
    }
}

#[derive(Message)]
#[rtype(result = "()")]
struct SetFlockMsg(Vec< Option< Addr<Boid> >>);

impl Handler<SetFlockMsg> for Boid {
    type Result = ();

    fn handle(&mut self, msg: SetFlockMsg, _ctx: &mut Self::Context) {
        self.flock = Some( msg.0 );
    }
}

#[derive(Message)]
#[rtype(result = "Vec<SendFut>")]
struct StartFlockUpdate;

type SendFut = Request<Boid, BoidDataMsg>;

impl Handler<StartFlockUpdate> for Boid {
    type Result = Vec<SendFut>;

    fn handle(&mut self, msg: StartFlockUpdate, _ctx: &mut Self::Context) -> Vec<SendFut>{
        let bdmsg = BoidDataMsg{ position: self.position, velocity: self.velocity };
        self.flock.as_ref().expect("flock not init").iter().filter_map(|oa| oa.as_ref().map(|a| a.send(bdmsg))).collect::<Vec<_>>()
    }
}

#[derive(Message)]
#[rtype(result = "()")]
struct CommitAndUpdateMsg;

impl Handler<CommitAndUpdateMsg> for Boid {
    type Result = ();

    fn handle(&mut self, msg: CommitAndUpdateMsg, _ctx: &mut Self::Context) -> () {
        unimplemented!();
    }
}

struct WindowManager {
    window: kiss3d::window::Window,
    bounding_box: (f32, f32, f32),
}

impl WindowManager {
    fn new() -> Self {
        const X_BOUND: f32 = 100f32;
        const Y_BOUND: f32 = 100f32;
        const Z_BOUND: f32 = 100f32;

        WindowManager {
            window: kiss3d::window::Window::new("RS Boids"),
            bounding_box: (X_BOUND, Y_BOUND, Z_BOUND),
        }
    }
}

impl Actor for WindowManager {
    type Context = Context<Self>;
}

#[derive(Message)]
#[rtype(result = "()")]
struct Render;

impl Handler<Render> for WindowManager {
    type Result = ();

    fn handle(&mut self, msg: Render, _ctx: &mut Self::Context) -> () {
        if !self.window.render() {
            System::current().stop();
        }
    }
}

#[derive(Message)]
#[rtype(result = "kiss3d::scene::SceneNode")]
struct GetBoidObjectMsg;

impl Handler<GetBoidObjectMsg> for WindowManager {
    type Result = kiss3d::scene::SceneNode;

    fn handle(&mut self, msg: GetBoidObjectMsg, _ctx: &mut Self::Context) -> kiss3d::scene::SceneNode {
        let boid_model_path = Path::new(BOID_MODEL_PATH);
        let boid_obj_path = boid_model_path.join(Path::new(BOID_OBJ_FILE));
        self.window.add_obj(&boid_obj_path, boid_model_path, na::Vector3::new(1f32, 1f32, 1f32))
    }
}



#[actix::main] // <- starts the system and block until future resolves
async fn main() {
    let mut boids = Vec::<Boid>::with_capacity(NUM_BOIDS);
    for _ in 0..NUM_BOIDS {
        boids.push(
            Boid::new(na::Vector3::default(), na::Vector3::default())
        )
    }

    let addrs = boids.into_iter().map(|x| x.start()).collect::<Vec<_>>();

    let init_fut = join_all(addrs.iter().enumerate().map( |(i, addr)| {
        let flock = addrs.clone().into_iter().enumerate().map(
            |(j, addr)| if i == j { None } else { Some(addr) }
        ).collect::<Vec<_>>();

        addr.send(SetFlockMsg(flock))
    }));

    let init_ok = init_fut.await.iter().all(|i| i.is_ok());

    let update_fut = join_all( addrs.iter().map(|addr| addr.send(StartFlockUpdate)) );
    let updates_fut = join_all( update_fut.await.into_iter().map(|res| res.expect("init send phase failed")).flatten() );

    let update_success = updates_fut.await.iter().all(|u| u.is_ok());

    let commit_fut = join_all( addrs.iter().map(|addr| addr.send(CommitAndUpdateMsg)) );

    let commit_ok = commit_fut.await.iter().all(|i| i.is_ok());

    // window_manager.send(Render);







}