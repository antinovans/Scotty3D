
#include "../rays/shapes.h"
#include "debug.h"
#include <iostream>

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    
    Trace ret;
    ret.origin = ray.point;
    float a = dot(ray.dir, ray.dir);
    float b = 2 * dot(ret.origin,ray.dir);
    float c = dot(ret.origin,ret.origin) - radius*radius;
    float delta = b*b - 4*a*c;
    if(delta < 0)
    {
        ret.hit = false;       // was there an intersection?
        ret.distance = 0.0f;   // at what distance did the intersection occur?
        ret.position = Vec3{}; // where was the intersection?
        ret.normal = Vec3{};   // what was the surface normal at the intersection?
        return ret;
    }
    float t1 = (-b + sqrt(delta))/2*a;
    float t2 = (-b - sqrt(delta))/2*a;
    float tmin = (t1 <= t2) ? t1 : t2;
    float tmax = (t1 <= t2) ? t2 : t1;
    //no intersection
    if(tmax < ray.dist_bounds.x || tmin > ray.dist_bounds.y || (tmin > ray.dist_bounds.x && ray.dist_bounds.y < tmax))
    {
        ret.hit = false;
        ret.distance = 0.0f; 
        ret.position = Vec3{};
        ret.normal = Vec3{};
        return ret;
    }
    //corner case
    if(tmin < ray.dist_bounds.x)
    {
        ret.hit = true;       
        ret.distance = tmax;   
        ret.position = ray.point + tmax * ray.dir; 
        ret.normal = ret.position.unit();   
        return ret;
    }
    ret.hit = true;       
    ret.distance = tmin;   
    ret.position = ray.point + tmin*ray.dir; 
    ret.normal = ret.position.unit();   
    // std::cout << "hit"  << ret.position.unit() <<std::endl;
    return ret;
    
}

} // namespace PT
