// use actix::prelude::*;
// use kiss3d::nalgebra as na;
// use crate::window_manager::WindowManager;
// use super::Boid;
use super::*;
use crate::window_manager::messages as window_messages;

#[derive(Clone, Copy, Message)]
#[rtype(result = "()")]
pub struct BoidDataMsg {
    pub position: na::Vector3<f32>,
    pub velocity: na::Vector3<f32>,
}

impl Handler<BoidDataMsg> for Boid {
    type Result = ();

    fn handle(&mut self, msg: BoidDataMsg, _ctx: &mut Self::Context) {
        self.observe_boid(msg);
    }
}

#[derive(Message)]
#[rtype(result = "()")]
pub struct SetFlockMsg(pub Vec< Option< Addr<Boid> >>);

impl Handler<SetFlockMsg> for Boid {
    type Result = ();

    fn handle(&mut self, msg: SetFlockMsg, _ctx: &mut Self::Context) {
        self.flock = Some( msg.0 );
    }
}

#[derive(Message)]
#[rtype(result = "()")]
pub struct SetWindowManager(pub Addr<WindowManager>);

impl Handler<SetWindowManager> for Boid {
    type Result = ();

    fn handle(&mut self, msg: SetWindowManager, _ctx: &mut Self::Context) {
        self.window_manager = Some( msg.0 );
    }
}

#[derive(Message)]
#[rtype(result = "()")]
pub struct RegisterWithWindow;

type RegFut = ResponseActFuture<Boid, ()>;

impl Handler<RegisterWithWindow> for Boid {
    type Result = RegFut;

    fn handle(&mut self, _msg: RegisterWithWindow, _ctx: &mut Self::Context) -> Self::Result {
        Box::pin(
            self
            .window_manager
            .as_ref()
            .expect("window manager not init")
            .send(
                window_messages::RegisterBoid(
                    BoidDataMsg { position: self.position, velocity: self.velocity }
            ))
            .into_actor(self)
            .map(|id, boid, _ctx| { boid.id = id.expect("boid registration failed") })
        )
    }
}

#[derive(Message)]
#[rtype(result = "Vec<BoidUpdateFut>")]
pub struct StartFlockUpdate;

type BoidUpdateFut = Request<Boid, BoidDataMsg>;

impl Handler<StartFlockUpdate> for Boid {
    type Result = Vec<BoidUpdateFut>;

    fn handle(&mut self, msg: StartFlockUpdate, _ctx: &mut Self::Context) -> Vec<BoidUpdateFut>{
        let bdmsg = BoidDataMsg{ position: self.position, velocity: self.velocity };
        self.flock.as_ref().expect("flock not init").iter().filter_map(|oa| oa.as_ref().map(|a| a.send(bdmsg))).collect::<Vec<_>>()
    }
}

#[derive(Message)]
#[rtype(result = "Option<ModelUpdateFut>")]
pub struct CommitAndUpdateMsg;

type ModelUpdateFut = Request<WindowManager, window_messages::ModelUpdate>;

impl Handler<CommitAndUpdateMsg> for Boid {
    type Result = Option<ModelUpdateFut>;

    fn handle(&mut self, _msg: CommitAndUpdateMsg, _ctx: &mut Self::Context) -> Option<ModelUpdateFut> {
        //unimplemented!();
        let model_update = window_messages::ModelUpdate {
            id: self.id,
            translation: na::Translation3::from_vector(crate::TIME_STEP * na::Vector3::new(0f32, 0f32, 1f32)),//unimplemented!(),
            rot: na::UnitQuaternion::default(),//unimplemented!(),
        };
        Some( self.window_manager.as_ref().expect("window manager not init").send(model_update) )
    }
}