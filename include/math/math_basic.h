//
// Created by waxz on 23-1-31.
//

#ifndef SCAN_REPUBLISHER_MATH_BASIC_H
#define SCAN_REPUBLISHER_MATH_BASIC_H

#include <cmath>
#include <type_traits>

#define angle_normalise_zero(angle) std::abs(angle) < M_PI ? (angle) : ( angle + ((angle) > 0.0 ? -M_PI*2: M_PI*2) )
#define angle_normalise(angle, angle_mean)  ((std::abs(angle - angle_mean ) < M_PI )? (angle) : ( angle + ((angle - angle_mean) > 0.0 ? -M_PI*2.0: M_PI*2.0) ))

namespace math{
    //https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
    //Actually implements signum (-1, 0, or 1). Implementations here using copysign only return -1 or 1, which is not signum
    /*
     * params: number
     * return: signum (-1, 0, or 1)
     */
    template <typename T> inline constexpr
    int signum(T x, std::false_type is_signed) {
        return T(0) < x;
    }

    template <typename T> inline constexpr
    int signum(T x, std::true_type is_signed) {
        return (T(0) < x) - (x < T(0));
    }

    template <typename T> inline constexpr
    int signum(T x) {
        return signum(x, std::is_signed<T>());
    }

}

#endif //SCAN_REPUBLISHER_MATH_BASIC_H
