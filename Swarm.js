class Swarm {
    constructor(amount=-1) {
        this.boids = [];
        if(amount > 0) {
            this.create_pool(amount);
        }
    }
    
    create_pool(amount) {
        for(let i=0; i < amount; i++) {
            let x = random() * width;
            let y = random() * height;
            this.boids.push(new Boid(x, y));
        }
    }
}