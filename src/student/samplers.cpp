
#include "../rays/samplers.h"
#include "../util/rand.h"

namespace Samplers {

Vec2 Rect::sample() const {

    // TODO (PathTracer): Task 1

    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()
    float x = RNG::unit() * size.x;
    float y = RNG::unit() * size.y;
    return Vec2{x,y};
}

Vec3 Sphere::Uniform::sample() const {

    // TODO (PathTracer): Task 7
    float Xi1;
    if(RNG::coin_flip(0.5f))
    {
        Xi1 = RNG::unit();
    }
    else{
        Xi1 = -1 * RNG::unit();
    }
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    theta = clamp(theta, 0.0f, PI_F);
    phi = clamp(phi, 0.0f, 2.0f * PI_F);

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
    // Generate a uniformly random point on the unit sphere.
    // Tip: start with Hemisphere::Uniform

    // return Vec3{};
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7

    // Set up importance sampling data structures for a spherical environment map image.
    // You may make use of the _pdf, _cdf, and total members, or create your own.
    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    for(size_t j = 0; j < h; j++)
    {
        for(size_t i = 0; i < w; i++)
        {
            float theta = PI_F - ((float)j/(float)h)*PI_F;
            theta = clamp(theta, 0.0f, PI_F);
            float pdf = image.at(i,j).luma()*sin(theta);
            total += pdf;
            _pdf.push_back(pdf);
            _cdf.push_back(total);
        }
    }
    for(auto& p : _pdf)
    {
        p *= 1.0f/total;
    }
    for(auto& c : _cdf)
    {
        c *= 1.0f/total;
    }
}

Vec3 Sphere::Image::sample() const {

    // TODO (PathTracer): Task 7

    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound
    float p = RNG::unit();
    auto i = std::upper_bound(_cdf.begin(), _cdf.end(), p);

    size_t index = i - _cdf.begin();
    float theta = PI_F - PI_F * (index / w)/(float)h;
    float phi = 2.0f * PI_F* (index % w)/(float)w;
    theta = clamp(theta, 0.0f, PI_F);
    phi = clamp(phi, 0.0f, 2.0f * PI_F);

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

float Sphere::Image::pdf(Vec3 dir) const {

    // TODO (PathTracer): Task 7

    // What is the PDF of this distribution at a particular direction?
    float phi = atan2(dir.z, dir.x);
    if(phi < 0)
    {
        phi += 2.0f * PI_F;
    }
    // float theta = PI_F - acos(dir.y);
    float theta = acosf(dir.y);

    size_t phi_i = (size_t)std::round((w-1) * phi/(2.0f * PI_F));
    size_t theta_i = (size_t)std::round((h-1) * (PI_F - theta)/PI_F);
    size_t index = theta_i * w + phi_i;
    float Jacobian = w*h/(2.0f * PI_F * PI_F * sinf(theta));
    return _pdf[index] * Jacobian;
}

Vec3 Point::sample() const {
    return point;
}

Vec3 Triangle::sample() const {
    float u = std::sqrt(RNG::unit());
    float v = RNG::unit();
    float a = u * (1.0f - v);
    float b = u * v;
    return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

Vec3 Hemisphere::Uniform::sample() const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    return Vec3(xs, ys, zs);
}

Vec3 Hemisphere::Cosine::sample() const {

    float phi = RNG::unit() * 2.0f * PI_F;
    float cos_t = std::sqrt(RNG::unit());

    float sin_t = std::sqrt(1 - cos_t * cos_t);
    float x = std::cos(phi) * sin_t;
    float z = std::sin(phi) * sin_t;
    float y = cos_t;

    return Vec3(x, y, z);
}

} // namespace Samplers
