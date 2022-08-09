
#include "../geometry/spline.h"
#include "debug.h"

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents
    auto quadratic = time * time;
    auto cubic = time * time * time;
    auto h00 = 2*cubic - 3*quadratic + 1;
    auto h10 = cubic - 2*quadratic + time;
    auto h01 = -2*cubic +3* quadratic;
    auto h11 = cubic - quadratic;

    T spline = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    // return T();
    return spline;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.
    if(control_points.size() < 1)
    {
        return T();
    }
    else if(control_points.size() == 1)
    {
        return control_points.begin()->second;
    }
    else{
        // std::map<float, T>::iterator k2;
        auto k2 = control_points.upper_bound(time);
        //begin & end case
        if(k2 == control_points.begin())
        {
            return k2->second;
        }
        if(k2 == control_points.end())
        {
            k2--;
            return k2->second; 
        }
        //assigning value for k1 and k2
        auto t2 = k2->first;
        const T& p2 = k2->second;
        auto k1 = k2;
        k1--;
        auto t1 = k1->first;
        const T& p1 = k1->second;
        //assigning value for k0
        auto k0 = k1;
        k0--;
        auto t0 = k1 == control_points.begin() ? 2*t1 - t2 : k0->first;
        const T& p0 = k1 == control_points.begin() ? 2*p1 - p2 : k0->second;
        //assigning value for k3
        auto k3 = k2;
        k3++;
        auto t3 = k3 == control_points.end() ? 2*t2 - t1 : k3->first;
        const T& p3 = k3 == control_points.end() ? 2*p2 - p1 : k3->second;

        auto interval = t2 - t1;
        const T& m1 = (p2 - p0) * interval/ (t2 - t0);
        const T& m2 = (p3 - p1) * interval/ (t3 - t1);
        return cubic_unit_spline((time - t1)/interval, p1, p2, m1, m2);
    }
    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...

    // return cubic_unit_spline(0.0f, T(), T(), T(), T());
}
