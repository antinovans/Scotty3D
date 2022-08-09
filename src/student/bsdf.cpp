
#include "../rays/bsdf.h"
#include "../util/rand.h"

namespace PT {

static Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 5
    // Return reflection of dir about the surface normal (0,1,0).
    Vec3 direction = dir * -1 + 2 * (dot(dir, Vec3(0, 1, 0))) * Vec3(0, 1, 0);
    return direction;
}

static Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    float cosTheta = dot(out_dir.unit(), Vec3(0, 1, 0));
    float nt, ni;
    // TODO (PathTracer): Task 5
    // Use Snell's Law to refract out_dir through the surface.
    // Return the refracted direction. Set was_internal to true if
    // refraction does not occur due to total internal reflection,
    // and false otherwise.
    Vec3 in_dir;
    if(cosTheta < 0) {
        // nt = ior, ni = 1
        nt = index_of_refraction;
        ni = 1.0f;
    } else {
        nt = 1.0f;
        ni = index_of_refraction;
    }
    in_dir.x = (nt / ni) * (-out_dir.unit().x);
    in_dir.z = (nt / ni) * (-out_dir.unit().z);
    in_dir.y = sqrt(1- in_dir.x*in_dir.x - in_dir.z*in_dir.z);
    if(in_dir.y * out_dir.y > 0)
    {
        in_dir.y *= -1;
    }
    if(1 - (1.0f / index_of_refraction) * (1.0f / index_of_refraction) * (1 - cosTheta * cosTheta) <
       0) {
        was_internal = true;
    }

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    return in_dir;
}

Scatter BSDF_Lambertian::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 4

    // Sample the BSDF distribution using the cosine-weighted hemisphere sampler.
    // You can use BSDF_Lambertian::evaluate() to compute attenuation.
    Scatter ret;
    ret.direction = Vec3{sampler.sample()};
    ret.attenuation = Spectrum{evaluate(sampler.sample(), out_dir)};
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the ratio of reflected/incoming radiance when light from in_dir
    // is reflected through out_dir: albedo * cos(theta).
    float cosTheta = dot(in_dir.unit(), Vec3(0, 1, 0));
    Spectrum output = albedo * cosTheta;
    return output;
}

float BSDF_Lambertian::pdf(Vec3 out_dir, Vec3 in_dir) const {

    // TODO (PathTracer): Task 4

    // Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
    float cosTheta = dot(out_dir.unit(), Vec3(0, 1, 0));
    cosTheta = clamp(cosTheta, 0.0f, 1.0f);
    return cosTheta / PI_F;

    // return 0.0f;
}

Scatter BSDF_Mirror::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5

    Scatter ret;
    ret.direction = reflect(out_dir);
    ret.attenuation = reflectance;
    return ret;
}

Scatter BSDF_Glass::scatter(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    Scatter ret;
    bool internal = false;
    Vec3 in_dir = refract(out_dir, index_of_refraction, internal);
    if(internal) {
        ret.direction = reflect(out_dir);
        ret.attenuation = reflectance;
    } else {
        // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
        float cosTheta = dot(in_dir.unit(), Vec3(0, 1, 0));
        float absCosTheta = cosTheta > 0 ? cosTheta: -cosTheta;
        float R0 = ((1.0f - index_of_refraction) / (1.0f + index_of_refraction)) *
                   ((1.0f - index_of_refraction) / (1.0f + index_of_refraction));
        float Fr = R0 + (1.0f - R0) * (1 - absCosTheta) * (1 - absCosTheta) * (1 - absCosTheta) *
                            (1 - absCosTheta) * (1 - absCosTheta);
        // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip:
        // RNG::coin_flip
        if(RNG::coin_flip(Fr)) {
            // reflect
            ret.direction = reflect(out_dir);
            ret.attenuation = reflectance;
        } else {
            // refract
            ret.direction = in_dir;
            if(cosTheta > 0) {
                // from air
                ret.attenuation = transmittance * 1 / (index_of_refraction * index_of_refraction);
            } else {
                ret.attenuation = transmittance * (index_of_refraction * index_of_refraction);
            }
        }
    }
    // (3) Compute attenuation based on reflectance or transmittance
    return ret;
    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
    // What happens upon total internal reflection?
}

Scatter BSDF_Refract::scatter(Vec3 out_dir) const {

    // OPTIONAL (PathTracer): Task 5

    // When debugging BSDF_Glass, it may be useful to compare to a pure-refraction BSDF

    Scatter ret;
    ret.direction = Vec3();
    ret.attenuation = Spectrum{};
    return ret;
}

Spectrum BSDF_Diffuse::emissive() const {
    return radiance;
}

} // namespace PT
