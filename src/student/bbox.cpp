
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    if(empty()) {
        return false;
    }
    times.x = ray.dist_bounds.x;
    times.y = ray.dist_bounds.y;
    for(int d = 0; d < 3; d++) {
        float t0 = std::min((min[d] - ray.point[d]) * (1 / ray.dir[d]),
                            (max[d] - ray.point[d]) * (1 / ray.dir[d]));
        float t1 = std::max((min[d] - ray.point[d]) * (1 / ray.dir[d]),
                            (max[d] - ray.point[d]) * (1 / ray.dir[d]));
        times.x = std::max(times.x, t0);
        times.y = std::min(times.y, t1);
        if(times.x > times.y) return false;
    }

    return times.x < ray.dist_bounds.y;
}
