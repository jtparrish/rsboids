use actix::prelude::*;
use kiss3d::nalgebra as na;
use futures_util::future::join_all;

const NUM_BOIDS: usize = 100;

struct Boid {
    position: na::Vector3<f32>,
    velocity: na::Vector3<f32>,
    delta_v: na::Vector3<f32>,
    flock: Option< Vec< Option<Addr<Boid>> > >,
    graphic: (),
}

impl Boid {
    fn new(pos: na::Vector3<f32>, vel: na::Vector3<f32>) -> Self {
        Boid {
            position: pos,
            velocity: vel,
            delta_v: na::Vector3::default(),
            flock: None,
            graphic: (),
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
    window: (),
    bounding_box: (),
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
        unimplemented!();
        // self.window.render();
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