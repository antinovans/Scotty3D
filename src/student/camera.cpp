
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    Samplers::Rect rect(Vec2(aperture,aperture));
    float screenH = 2 * focal_dist * tan((float)Radians(vert_fov / 2));
    float screenW = screenH * aspect_ratio ;
    Vec3 sampleSpaceCor = Vec3(screenW * (screen_coord.x - rect.sample().x - 0.5f), screenH * (screen_coord.y -rect.sample().y - 0.5f), -focal_dist);
    Ray ray = Ray(position, sampleSpaceCor);
    ray.transform(iview);
    ray.point = position;
    return ray;
}
