function setup() {
    createCanvas(400, 400);
    frameRate(60);
    textSize(15);

    dev = false;
    separationSlider = createSlider(0, 2, 0.5, 0.1);
    separationSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+20);
    cohesionSlider = createSlider(0, 2, 0, 0.1);
    cohesionSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+50);
    targetCohesionSlider = createSlider(0, 2, 0.8, 0.1);
    targetCohesionSlider.position((windowWidth-width)/2+20, (windowHeight-height)/2+80);
    if(!dev) {
        separationSlider.hide();
        cohesionSlider.hide();
        targetCohesionSlider.hide();
    }

    swarm = new Swarm();
    swarm.create_pool(16);

    
    let incr = 360/swarm.boids.length;
    pattern = [];
    for(let i=1; i < swarm.boids.length+3; i++) {
        pattern.push([{rad: radians(incr*i), dist: 50}]);
    }

    let target = pattern[0][0];
    swarm.boids[0].target.index = 0;
    swarm.boids[0].position = createVector(75, height/2+25);
    swarm.boids[0].fixed = true;
    swarm.boids[0].target.position = p5.Vector.add(swarm.boids[0].position, createVector(sin(target.rad)*target.dist, cos(target.rad)*target.dist));
    pattern[0][0].done = true;
}

function draw() {
    background(250);
    if(dev) {
        fill(0);
        noStroke();
        text("separation", 30+separationSlider.width, separationSlider.height+15);
        text("cohesion", 30+cohesionSlider.width, cohesionSlider.height+45);
        text("target cohesion", 30+cohesionSlider.width, cohesionSlider.height+75);
    }

    for(let i=0; i < swarm.boids.length; i++) {
        let boid = swarm.boids[i];
        boid.edges();
        let flock = boid.detect_neighborhood(swarm.boids);
        let master = null;
        if(flock.length > 0) {
            boid.flock(flock);
            boid.drawNeighbors(flock);
            master = boid.getNearestFixed(flock);
        }
        if(master) {
            if(dist(boid.position.x, boid.position.y, master.target.position.x, master.target.position.y) < Boid.close_threshold) {
                boid.fixed = true;
                master.target.fixed = true;
                boid.target.index = master.target.index + 1;
                pattern_target = pattern[boid.target.index][0];
                boid.target.position = p5.Vector.add(master.target.position, createVector(sin(pattern_target.rad)*pattern_target.dist, cos(pattern_target.rad)*pattern_target.dist));
            }
            let attraction = boid.cohesion([master.target]);
            attraction.mult(targetCohesionSlider.value());
            boid.acceleration.add(attraction);
        } else
            boid.acceleration.add(createVector(random(-1, 1), random(-1, 1)));
        if(!boid.fixed) {
            boid.update();
        }

        boid.draw();
    }
}