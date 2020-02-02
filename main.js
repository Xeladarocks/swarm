function setup() {
    createCanvas(500, 700);
    frameRate(60);
    textSize(15);

    dev = true;
    separationSlider = createSlider(0, 2, 1, 0.1);
    separationSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+20);
    cohesionSlider = createSlider(0, 2, 0, 0.1);
    cohesionSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+50);
    targetCohesionSlider = createSlider(0, 2, 0.8, 0.1);
    targetCohesionSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+80);
    alignSlider = createSlider(0, 2, 0.1, 0.1);
    alignSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+110);
    if(!dev) {
        separationSlider.hide();
        cohesionSlider.hide();
        targetCohesionSlider.hide();
        alignSlider.hide();
    }

    swarm = new Swarm();
    swarm.create_pool(21); // 17

    let incr = 490/swarm.boids.length;
    let dist = incr*2;

    pattern = [];
    for(let i=1; i < int((swarm.boids.length)/2); i++) {
        pattern.push({rad: radians(incr*i), dist: dist});
    }
    pattern.push({rad: radians(incr*int((swarm.boids.length)/2)), dist: dist});
    for(let i=int((swarm.boids.length+2)/2); i > 0; i--) {
        pattern.push({rad: radians(incr*i), dist: dist});
    }
    console.log(pattern)

    let target = pattern[0];
    swarm.boids[0].target.index = 0;
    swarm.boids[0].position = createVector(100, height-150);
    swarm.boids[0].fixed = true;
    swarm.boids[0].target.position = p5.Vector.add(swarm.boids[0].position, createVector(sin(target.rad)*target.dist, cos(target.rad)*target.dist));
    target.claimed = false;
}

function draw() {
    background(250);
    if(dev) {
        fill(0);
        noStroke();
        text("separation", 30+separationSlider.width, separationSlider.height+15);
        text("align", 30+alignSlider.width, alignSlider.height+45);
        text("cohesion", 30+cohesionSlider.width, cohesionSlider.height+75);
        text("target cohesion", 30+targetCohesionSlider.width, targetCohesionSlider.height+105);
    }

    for(let i=0; i < swarm.boids.length; i++) {
        let boid = swarm.boids[i];
        boid.edges();
        let flock = boid.detect_neighborhood(swarm.boids);
        let master = null;
        if(flock.length > 0) {
            boid.flock(flock);
            if(dev)boid.drawNeighbors(flock);
            master = boid.getNearestFixed(flock);
        }
        if(!boid.fixed) {
            if(master && master.target.child === boid) {
                boid.listenMaster(master);
            } else if(master) {
                boid.followGuide(master, 1);
            } else { // no target, look for new master
                boid.acceleration.add(createVector(random(-1, 1), random(-1, 1)));
            }
            boid.update();
        }

        boid.draw();
    }
}