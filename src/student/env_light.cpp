
#include "../rays/env_light.h"

#include <limits>
#include <iostream>

namespace PT {

Vec3 Env_Map::sample() const {

    // TODO (PathTracer): Task 7

    // First, implement Samplers::Sphere::Uniform so the following line works.
    // Second, implement Samplers::Sphere::Image and swap to image_sampler

    // return uniform_sampler.sample();
    return image_sampler.sample();
}

float Env_Map::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // First, return the pdf for a uniform spherical distribution.
    // float pdf = 1.0f/(4.0f * PI_F);
    // Second, swap to image_sampler.pdf()
    float pdf = image_sampler.pdf(dir);
    return pdf;
}
Spectrum Env_Map::evaluate(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // Compute emitted radiance along a given direction by finding the corresponding
    // pixels in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 nearest pixels.
    float phi = atan2(dir.unit().z, dir.unit().x);
    if(phi < 0)
    {
        phi += 2.0f * PI_F;
    }
    float theta = PI_F - acos(dir.unit().y);
    theta = clamp(theta, 0.0f, PI_F);
    phi = clamp(phi, 0.0f, 2.0f * PI_F);
    //bilinear interpolation
    size_t fu = (size_t)floor((phi/(2*PI_F)) * (image.dimension().first - 1));
    size_t fv = (size_t)floor((theta/PI_F) * (image.dimension().second - 1));
    size_t cu = (size_t)ceil((phi/(2*PI_F)) * (image.dimension().first - 1));
    size_t cv = (size_t)ceil((theta/PI_F) * (image.dimension().second - 1));

    float s = (phi/(2*PI_F)) * (image.dimension().first - 1) - fu;
    float t = (theta/PI_F) * (image.dimension().second - 1) - fv;
    Spectrum s00 = image.at(fu,fv);
    Spectrum s10 = image.at(cu,fv);
    Spectrum s01 = image.at(fu,cv);
    Spectrum s11 = image.at(cu,cv);
    Spectrum spectrum = (1 - t)*((1 - s)* s00 + s*s10) + t * ((1 - s)* s01 + s*s11);

    // size_t fu = (size_t)((phi/(2*PI_F)) * image.dimension().first - 1);
    // size_t fv = (size_t)((theta/PI_F) * image.dimension().second - 1);
    // std::cout << "fu:" << fu << "fv " << fv<< std::endl;
    // Spectrum spectrum = image.at(fu,fv);
    return spectrum;
    // return Spectrum{};
}

Vec3 Env_Hemisphere::sample() const {
    return sampler.sample();
}

float Env_Hemisphere::pdf(Vec3 dir) const {
    return 1.0f / (2.0f * PI_F);
}

Spectrum Env_Hemisphere::evaluate(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Vec3 Env_Sphere::sample() const {
    return sampler.sample();
}

float Env_Sphere::pdf(Vec3 dir) const {
    return 1.0f / (4.0f * PI_F);
}

Spectrum Env_Sphere::evaluate(Vec3) const {
    return radiance;
}

} // namespace PT
