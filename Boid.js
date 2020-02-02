class Boid {
    static radius = 20;
    static detection_radius = 200;
    static avoidance_range = 50;
    static close_threshold = 5;
    constructor(x, y) {
        // local boid
        this.position = createVector(x, y);
        this.velocity = p5.Vector.random2D();
        this.velocity.setMag(random(2, 4));
        this.acceleration = createVector();
        this.maxForce = 1;
        this.maxSpeed = 4;

        // organization
        this.fixed = false;
        this.target = {
            position: createVector(),
            fixed: false,
            index: null
        }
    }
    draw() {
        noStroke();
        if(this.fixed)fill(0, 255, 0);
        else fill(0, 0, 255);
        circle(this.position.x, this.position.y, Boid.radius); // draw boid
        noFill(); stroke(0, 255, 0);
        circle(this.position.x, this.position.y, Boid.detection_radius) // draw boid detection
        stroke(255, 0, 0);
        circle(this.position.x, this.position.y, Boid.avoidance_range) // draw boid detection
    }
    drawNeighbors(flock) {
        noFill();
        stroke(100);
        for(let i=0; i < flock.length; i++) {
            line(this.position.x, this.position.y, flock[i].position.x, flock[i].position.y);
        }
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

    getNearestFixed(flock) {
        let nearest_d = Infinity;
        let nearest_boid = null;
        for(let i=0; i < flock.length; i++) {
            if(flock[i].fixed) {
                let d = dist(this.position.x, this.position.y, flock[i].position.x, flock[i].position.y);
                if(d < nearest_d) {
                    nearest_d = d;
                    nearest_boid = flock[i];
                }
            }
        }
        return nearest_boid;
    }

    flock(boids) {
        let separation = this.separation(boids);
        let cohesion = this.cohesion(boids);
    
        separation.mult(separationSlider.value());
        cohesion.mult(cohesionSlider.value());
        
        this.acceleration.add(separation);
        this.acceleration.add(cohesion);
    }

    separation(boids) {
        let steering = createVector();
        let total = 0;
        for (let other of boids) {
            let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
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
        let steering = createVector();
        let total = 0;
        for (let other of boids) {
          let d = dist(this.position.x, this.position.y, other.position.x, other.position.y);
            if (other != this && d < Boid.detection_radius) {
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
}