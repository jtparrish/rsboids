use actix::prelude::*;
use kiss3d::nalgebra as na;
use futures_util::future::{join_all, join};

const NUM_BOIDS: usize = 10;
const TIME_STEP: f32 = 1e-2;

mod boid_actor;
use boid_actor::Boid;
use boid_actor::messages as boid_messages;

mod window_manager;
use window_manager::WindowManager;
use window_manager::messages as window_messages;

#[actix::main] // <- starts the system and block until future resolves
async fn main() {
    let window = WindowManager::new();

    let win_addr = window.start();

    let mut boids = Vec::<Boid>::with_capacity(NUM_BOIDS);
    for _ in 0..NUM_BOIDS {
        boids.push(
            Boid::new(na::Vector3::default(), na::Vector3::default())
        )
    }

    let addrs = boids.into_iter().map(|x| x.start()).collect::<Vec<_>>();

    // initialize the flocks of all the boids
    let flock_init_fut = join_all(addrs.iter().enumerate().map( |(i, addr)| {
        let flock = addrs.clone().into_iter().enumerate().map(
            |(j, addr)| if i == j { None } else { Some(addr) }
        ).collect::<Vec<_>>();

        let win_addr = win_addr.clone();
        join(
            addr.send(boid_messages::SetFlockMsg(flock)),
            addr.send(boid_messages::SetWindowManager(win_addr)),
        )
    }));
    
    let flock_init_ok = flock_init_fut.await.iter().all(|(i0, i1)| i0.is_ok() && i1.is_ok());
    assert!(flock_init_ok, "flock init message failed");

    // register each boid with the window and create a model
    let init_models_fut = join_all(addrs.iter().map(|b_addr| b_addr.send(boid_messages::RegisterWithWindow)));
    let init_models_ok = init_models_fut.await.iter().all(|im| im.is_ok());
    assert!(init_models_ok, "model init message failed");

    loop {
        // perform an update on the boids
        let update_init_fut = join_all( addrs.iter().map(|addr| addr.send(boid_messages::StartFlockUpdate)) ); 
        let updates_fut = join_all( update_init_fut.await.into_iter().map(|res| res.expect("init update phase failed")).flatten() );
        let update_success = updates_fut.await.iter().all(|u| u.is_ok());
        assert!(update_success, "update message failed");


        // commit the updates to the boids states and send them to the window to update the models
        let commit_init_fut = join_all( addrs.iter().map(|addr| addr.send(boid_messages::CommitAndUpdateMsg)) );
        let commit_fut = join_all( commit_init_fut.await.into_iter().map(|res| res.expect("init commit phase failed").unwrap()) );
        let commit_ok = commit_fut.await.iter().all(|c| c.is_ok());
        assert!(commit_ok, "commit message failed");


        // render the scene
        let render_fut = win_addr.send(window_messages::Render);
        let render_ok = render_fut.await.is_ok();
        assert!(render_ok, "render message failed");
    }
}