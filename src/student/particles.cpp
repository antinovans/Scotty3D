
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Scene_Particles::Particle::update(const PT::Object& scene, float dt, float radius) {

    // TODO(Animation): Task 4

    // Compute the trajectory of this particle for the next dt seconds.
    float tau = 0.001f;
    float tempdt = dt;
    // (1) Build a ray representing the particle's path if it travelled at constant velocity.
    while(tempdt > 0) {
        Ray path(pos, velocity);
        // (2) Intersect the ray with the scene and account for collisions. Be careful when placing
        // collision points using the particle radius. Move the particle to its next position.
        auto intersection = scene.hit(path);
        if(intersection.hit) {
            float distance = (pos - intersection.position).norm();
            if(distance <= radius) {
                // hit
                velocity = velocity - 2 * dot(velocity, intersection.normal) * intersection.normal;
            }
        }
        // (3) Account for acceleration due to gravity.
        velocity += tau * acceleration;
        pos += tau * velocity;
        // (4) Repeat until the entire time step has been consumed.
        tempdt -= tau;
    }
    // (5) Decrease the particle's age and return whether it should die.
    age -= dt;
    return age > 0;
}
