
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    // TODO (PathTracer): Task 1

    // Generate a ray that uniformly samples pixel (x,y) and return the incoming light.
    // The following code generates a ray at the bottom left of the pixel every time.

    // Tip: Samplers::Rect
    // Tip: log_ray is useful for debugging
    // Vec2 xy((float)x, (float)y);
    // Vec2 wh((float)out_w, (float)out_h);
    // Samplers::Rect rect;
    // Ray ray = camera.generate_ray((xy + rect.sample()) / wh);
    // ray.depth = max_depth;
    // if(RNG::coin_flip(0.0005f)) log_ray(ray, 10.0f);
    // auto [emissive, reflected] = trace(ray);
    // return emissive + reflected;
    Spectrum totalE(0, 0, 0);
    Spectrum totalR(0, 0, 0);
    for(size_t i = 0; i < n_samples; i++) {
        Vec2 xy((float)x, (float)y);
        Vec2 wh((float)out_w, (float)out_h);
        Samplers::Rect rect;
        Ray ray = camera.generate_ray((xy + rect.sample()) / wh);
        ray.depth = max_depth;
        if(RNG::coin_flip(0.0005f)) log_ray(ray, 10.0f);

        // Pathtracer::trace() returns the incoming light split into emissive and reflected
        // components. auto [emissive, reflected] = trace(ray);
        totalE += trace(ray).first;
        totalR += trace(ray).second;
    }
    totalE *= 1.0f / n_samples;
    totalR *= 1.0f / n_samples;
    return totalE + totalR;
}

Spectrum Pathtracer::sample_indirect_lighting(const Shading_Info& hit) {

    // TODO (PathTrace): Task 4

    // This function computes a single-sample Monte Carlo estimate of the _indirect_
    // lighting at our ray intersection point.

    // (1) Randomly sample a new ray direction from the BSDF distribution using BSDF::scatter().
    Scatter scatter = hit.bsdf.scatter(hit.out_dir);
    // // (2) Create a new world-space ray and call Pathtracer::trace() to get incoming light. You
    // // should modify time_bounds so that the ray does not intersect at time = 0. Remember to
    // // set the new depth value.
    Vec3 scatterWDir = hit.object_to_world.rotate(scatter.direction);
    Ray wsray(hit.pos, scatterWDir);
    wsray.dist_bounds.x = 0.001f;
    wsray.depth = hit.depth - 1;
    Spectrum indir = Pathtracer::trace(wsray).second;

    // (3) Add contribution due to incoming light scaled by BSDF attenuation. Whether you
    // compute the BSDF scattering PDF should depend on if the BSDF is a discrete distribution
    // (see BSDF::is_discrete()).
    indir *= scatter.attenuation;
    if(!hit.bsdf.is_discrete()) {
        indir *= 1.0f / hit.bsdf.pdf(hit.out_dir, scatter.direction);
    }
    // You should only use the indirect component of incoming light (the second value returned
    // by Pathtracer::trace()), as the direct component will be computed in
    // Pathtracer::sample_direct_lighting().
    return indir;

    // Spectrum radiance;
    // return radiance;
}

Spectrum Pathtracer::sample_direct_lighting(const Shading_Info& hit) {

    // This function computes a Monte Carlo estimate of the _direct_ lighting at our ray
    // intersection point by sampling both the BSDF and area lights.

    // Point lights are handled separately, as they cannot be intersected by tracing rays
    // into the scene.
    Spectrum radiance = point_lighting(hit);
    // // TODO (PathTrace): Task 4

    // For task 4, this function should perform almost the same sampling procedure as
    // Pathtracer::sample_indirect_lighting(), but instead accumulates the emissive component of
    // incoming light (the first value returned by Pathtracer::trace()). Note that since we only
    // want emissive, we can trace a ray with depth = 0.
    // Scatter scatter = hit.bsdf.scatter(hit.out_dir);
    // Vec3 scatterWDir = hit.object_to_world.rotate(scatter.direction);
    // Ray wsray(hit.pos, scatterWDir);
    // wsray.dist_bounds.x = 0.001f;
    // wsray.depth = 0;
    // Spectrum indir = Pathtracer::trace(wsray).first;
    // indir *= scatter.attenuation;
    // if(!hit.bsdf.is_discrete()) {
    //     indir *= 1.0f / hit.bsdf.pdf(hit.out_dir, scatter.direction);
    // }
    // TODO (PathTrace): Task 6

    // For task 6, we want to upgrade our direct light sampling procedure to also
    // sample area lights using mixture sampling.

    // (1) If the BSDF is discrete, we don't need to bother sampling lights: the behavior
    // should be the same as task 4.
    Spectrum indir;
    if(hit.bsdf.is_discrete()) {
        Scatter scatter = hit.bsdf.scatter(hit.out_dir);
        Vec3 scatterWDir = hit.object_to_world.rotate(scatter.direction);
        Ray wsray(hit.pos, scatterWDir);
        wsray.dist_bounds.x = 0.001f;
        wsray.depth = 0;
        indir = Pathtracer::trace(wsray).first;
        indir *= scatter.attenuation;
    }
    // (2) Otherwise, we should randomly choose whether we get our sample from `BSDF::scatter`
    // or `Pathtracer::sample_area_lights`. Note that `Pathtracer::sample_area_lights` returns
    // a world-space direction pointing toward an area light. Choose between the strategies
    // with equal probability.
    else {
        Vec3 scatterWDirArea = sample_area_lights(hit.pos);
        Scatter scatter = hit.bsdf.scatter(hit.out_dir);
        Vec3 scatterWDirBSDF = hit.object_to_world.rotate(scatter.direction);
        float pdf = (area_lights_pdf(hit.pos, scatterWDirArea) + hit.bsdf.pdf(hit.out_dir, scatter.direction))/ 2;
        if(RNG::coin_flip(0.5f)) // sample_area_lights
        {
            Ray wsray(hit.pos, scatterWDirArea);
            wsray.dist_bounds.x = 0.001f;
            wsray.depth = 0;
            indir = Pathtracer::trace(wsray).first;
            indir *= hit.bsdf.evaluate(hit.out_dir, hit.world_to_object.rotate(scatterWDirArea));
            indir *= 1.0f / pdf;
            // if(RNG::unit() < 0.001f)
            // {
            //     log_ray(wsray, 2.0f);
            // }
        } else {
            Ray wsray(hit.pos, scatterWDirBSDF);
            wsray.dist_bounds.x = 0.001f;
            wsray.depth = 0;
            indir = Pathtracer::trace(wsray).first;
            indir *= scatter.attenuation;
            indir *= 1.0f / pdf;
            // if(RNG::unit() < 0.001f)
            // {
            //     log_ray(wsray, 2.0f);
            // }
        }
        // (3) Create a new world-space ray and call Pathtracer::trace() to get incoming light. You
        // should modify time_bounds so that the ray does not intersect at time = 0. We are again
        // only interested in the emissive component, so the ray depth can be zero.
        // (4) Add estimate of incoming light scaled by BSDF attenuation. Given a sample,
        // we don't know whether it came from the BSDF or the light, so you should use
        // BSDF::evaluate(), BSDF::pdf(), and Pathtracer::area_lights_pdf() to compute the proper
        // weighting. What is the PDF of our sample, given it could have been produced from either
        // source?

    }
    return radiance + indir;
}

std::pair<Spectrum, Spectrum> Pathtracer::trace(const Ray& ray) {

    // This function orchestrates the path tracing process. For convenience, it
    // returns the incoming light along a ray in two components: emitted from the
    // surface the ray hits, and reflected through that point from other sources.

    // Trace ray into scene.
    Trace result = scene.hit(ray);
    if(!result.hit) {

        // If no surfaces were hit, sample the environemnt map.
        if(env_light.has_value()) {
            return {env_light.value().evaluate(ray.dir), {}};
        }
        return {};
    }

    // If we're using a two-sided material, treat back-faces the same as front-faces
    const BSDF& bsdf = materials[result.material];
    if(!bsdf.is_sided() && dot(result.normal, ray.dir) > 0.0f) {
        result.normal = -result.normal;
    }

    // TODO (PathTracer): Task 4
    // You will want to change the default normal_colors in debug.h, or delete this early out.
    // if(debug_data.normal_colors) return {Spectrum::direction(result.normal), {}};

    // If the BSDF is emissive, stop tracing and return the emitted light
    Spectrum emissive = bsdf.emissive();
    if(emissive.luma() > 0.0f) return {emissive, {}};

    // If the ray has reached maximum depth, stop tracing
    if(ray.depth == 0) return {};

    // Set up shading information
    Mat4 object_to_world = Mat4::rotate_to(result.normal);
    Mat4 world_to_object = object_to_world.T();
    Vec3 out_dir = world_to_object.rotate(ray.point - result.position).unit();

    Shading_Info hit = {bsdf,    world_to_object, object_to_world, result.position,
                        out_dir, result.normal,   ray.depth};

    // Sample and return light reflected through the intersection
    return {{}, sample_direct_lighting(hit) + sample_indirect_lighting(hit)};
}

} // namespace PT
