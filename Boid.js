class Boid {
    static radius = 20;
    static detection_radius = 200;
    static avoidance_range = 50;
    static close_threshold = 2;
    constructor(x, y) {
        // local boid
        this.position = createVector(x, y);
        this.velocity = p5.Vector.random2D();
        this.velocity.setMag(random(2, 4));
        this.acceleration = createVector();
        this.maxForce = 1;
        this.maxSpeed = 4;

        this.radius = Boid.radius;
        this.detection_radius = Boid.detection_radius;
        this.avoidance_range = Boid.avoidance_range;
        this.close_threshold = Boid.close_threshold;

        // organization
        this.fixed = false;
        this.target = {
            position: createVector(),
            fixed: false,
            index: null,
            child: null
        }
    }
    draw() {
        fill(color(0, 0, 255));
        stroke(color(0, 0, 255));
        circle(this.position.x, this.position.y, Boid.radius);
        fill(0, 0, 0, 0);
        stroke(0, 255, 0);
        circle(this.position.x, this.position.y, Boid.detection_radius)
    }

    update() {
        this.position.add(this.velocity.copy().mult(deltaTime/50));
        this.velocity.add(this.acceleration);
        this.velocity.limit(this.maxSpeed);
        this.acceleration.mult(0);
    }

    // returns array that contains boid's neighbors
    detect_neighborhood(boids) {
        var neighbors = [];
        for(let i=0; i<boids.length; i++) {
            let boid = boids[i];
            if(boid == this)continue;
            let distance = dist(this.position.x, this.position.y, boid.position.x, boid.position.y);
            if ((distance < Boid.detection_radius/2+Boid.radius/2)) { //overlay
                neighbors.push(boid);
            }
        }
        return neighbors;
    }

    flock(boids) {
        let alignment = this.align(boids);
        let cohesion = this.cohesion(boids);
        let separation = this.separation(boids);
    
        alignment.mult(alignSlider.value());
        cohesion.mult(cohesionSlider.value());
        separation.mult(separationSlider.value());
    
        this.acceleration.add(alignment);
        this.acceleration.add(cohesion);
        this.acceleration.add(separation);
    }

    align(boids) {
        let steering = createVector();
        let total = 0;
        for (let other of boids) {
            let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
            if (other != this && d < Boid.avoidance_range) {
                steering.add(other.velocity);
                total++;
            }
        }
        if (total > 0) {
            steering.div(total);
            steering.setMag(this.maxSpeed);
            steering.sub(this.velocity);
            steering.limit(this.maxForce);
        }
        return steering;
    }

    separation(boids) {
        let steering = createVector();
        let total = 0;
        for (let other of boids) {
          let d = dist(
            this.position.x,
            this.position.y,
            other.position.x,
            other.position.y
          );
          if (other != this && d < Boid.avoidance_range) {
            let diff = p5.Vector.sub(this.position, other.position);
            diff.div(d * d);
            steering.add(diff);
            total++;
          }
        }
        if (total > 0) {
            steering.div(total);
            steering.setMag(this.maxSpeed);
            steering.sub(this.velocity);
            steering.limit(this.maxForce);
        }
        return steering;
    }

    cohesion(boids) {
        let perceptionRadius = 100;
        let steering = createVector();
        let total = 0;
        for (let other of boids) {
          let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
            if (other != this && d < perceptionRadius) {
                steering.add(other.position);
                total++;
            }
        }
        if (total > 0) {
            steering.div(total);
            steering.sub(this.position);
            steering.setMag(this.maxSpeed);
            steering.sub(this.velocity);
            steering.limit(this.maxForce);
        }
        return steering;
    }

    edges() {
        if (this.position.x+Boid.radius/2 > width) {
            this.velocity.x = -1;
        } else if (this.position.x-Boid.radius/2 < 0) {
            this.velocity.x = 1;
        }
        if (this.position.y+Boid.radius/2 > height) {
            this.velocity.y = -1;
        } else if (this.position.y-Boid.radius/2 < 0) {
            this.velocity.y = 1;
        }
    }

    getNearestFixed(flock) {
        let nearest_d = Infinity;
        let nearest_boid = null;
        for(let i=0; i < flock.length; i++) {
            if(flock[i].fixed) { // old child or new child
                let d = dist(this.position.x, this.position.y, flock[i].position.x, flock[i].position.y);
                if(d < nearest_d) {
                    nearest_d = d;
                    nearest_boid = flock[i];
                }
                if(!flock[i].target.child && !this.fixed) flock[i].target.child = this;
            }
        }
        return nearest_boid;
    }

    listenMaster(master) {
        if(dist(this.position.x, this.position.y, master.target.position.x, master.target.position.y) < this.close_threshold) {
            this.fixed = true;
            master.target.fixed = true;
            this.target.index = master.target.index + 1;
            let pattern_target = pattern[this.target.index];
            this.target.position = p5.Vector.add(master.target.position, createVector(sin(pattern_target.rad)*pattern_target.dist, cos(pattern_target.rad)*pattern_target.dist));
        }
        this.followGuide(master, 1.5);
    }

    followGuide(guide, weight=1) {
        let attraction = this.cohesion([guide.target]);
        attraction.mult(targetCohesionSlider.value()*weight);
        this.acceleration.add(attraction);
    }
}
