function setup() {
    createCanvas(500, 700);
    frameRate(60);
    textSize(15);

    dev = false;
    separationSlider = createSlider(0, 2, 1, 0.1);
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
    swarm.create_pool(17); // 17

    pattern = [
        [{rad: radians(30), dist: 75}], // 1
        [{rad: radians(60), dist: 75}],
        [{rad: radians(90), dist: 75}],
        [{rad: radians(120), dist: 75}],
        [{rad: radians(150), dist: 75}],
        [{rad: radians(180), dist: 75}], // 5
        [{rad: radians(210), dist: 75}],
        [{rad: radians(240), dist: 75}],
        [{rad: radians(240), dist: 75}],
        [{rad: radians(210), dist: 75}],
        [{rad: radians(180), dist: 75}], // 10
        [{rad: radians(150), dist: 75}],
        [{rad: radians(120), dist: 75}],
        [{rad: radians(90), dist: 75}],
        [{rad: radians(60), dist: 75}],
        [{rad: radians(30), dist: 75}], // 15
        [{rad: radians(0), dist: 0}],

    ];

    let target = pattern[0][0];
    swarm.boids[0].target.index = 0;
    swarm.boids[0].position = createVector(100, height-150);
    swarm.boids[0].fixed = true;
    swarm.boids[0].target.position = p5.Vector.add(swarm.boids[0].position, createVector(sin(target.rad)*target.dist, cos(target.rad)*target.dist));
    pattern[0][0].claimed = false;
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
        if(!boid.fixed) {
            if(master && master.target.child === boid) {
                boid.listenMaster(master);
            } else if(master) {
                boid.followGuide(master, 0.5);
            } else { // no target, look for new master
                boid.acceleration.add(createVector(random(-1, 1), random(-1, 1)));
            }
            boid.update();
        }

        boid.draw();
    }
}