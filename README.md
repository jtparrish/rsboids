# rsboids

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#what-are-boids">What are Boids?</a></li>
      </ul>
    </li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#how-it-works">How it Works</a></li>
  </ol>
</details>

## About The Project

rsboids is an implementation of a boids system built using Rust and the Actix actor framework (https://github.com/actix/actix) for Rust.

### What are Boids?

The original idea for boids comes from a 1987 paper by Craig Reynolds (https://dl.acm.org/doi/10.1145/37401.37406). Each boid observes the others positions and accelerates to maintain spacing with other boids and flock into formations. From simple rules that enforce spacing and favor the company of nearby groups, complex flocking behaviors can be generated naturally.

## Roadmap

- [x] Setup rendering
- [x] Setup render update messaging
- [x] Setup all-to-all boid messaging
- [ ] Setup the update logic
- [ ] Correct the Geometry
    - [ ] Make sure the position is calculated as the center of the boid
    - [ ] Ensure that roll is properly taken care of

## How it Works

Each boid is registered with the window manager actor through the use of a unique integer ID. Each update is intiated by sending an update message to each of the boids. When a boid receives this message, it sends its current position and velocity to every other boid. Once all of this communication is complete, a message is sent to each of the boids to conclude their update. Then each boid sends a position/orientation update to the window manager. After each boid has messaged the window manager, the window manager is instructed to render the scene.


