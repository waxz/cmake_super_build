//
// Created by waxz on 5/8/23.
//

#ifndef CMAKE_SUPER_BUILD_GEOMETRY_H
#define CMAKE_SUPER_BUILD_GEOMETRY_H


namespace math{
    //https://gist.github.com/TimSC/47203a0f5f15293d2099507ba5da44e6#file-linelineintersect-cpp
/** Calculate determinant of matrix:
	[a b]
	[c d]
*/
    inline float Det(float a, float b, float c, float d)
    {
        return a*d - b*c;
    }
///Calculate intersection of two lines.
///\return true if found, false if not found or error
    inline bool LineLineIntersect(float x1, float y1, //Line 1 start
                           float x2, float y2, //Line 1 end
                           float x3, float y3, //Line 2 start
                           float x4, float y4, //Line 2 end
                           float &ixOut, float &iyOut) //Output
    {
        //http://mathworld.wolfram.com/Line-LineIntersection.html

        float detL1 = Det(x1, y1, x2, y2);
        float detL2 = Det(x3, y3, x4, y4);
        float x1mx2 = x1 - x2;
        float x3mx4 = x3 - x4;
        float y1my2 = y1 - y2;
        float y3my4 = y3 - y4;

        float xnom = Det(detL1, x1mx2, detL2, x3mx4);
        float ynom = Det(detL1, y1my2, detL2, y3my4);
        float denom = Det(x1mx2, y1my2, x3mx4, y3my4);
        if(denom == 0.0)//Lines don't seem to cross
        {
            ixOut = NAN;
            iyOut = NAN;
            return false;
        }

        ixOut = xnom / denom;
        iyOut = ynom / denom;
        if(!std::isfinite(ixOut) || !std::isfinite(iyOut)) //Probably a numerical issue
            return false;

        return true; //All OK
    }



    inline void to_yaw( float x,  float y,  float z,  float w, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);


// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }
    inline void toEulerAngle(const float x, const float y, const float z, const float w, float &roll, float &pitch, float &yaw) {
// roll (x-axis rotation)
        float sinr_cosp = +2.0 * (w * x + y * z);
        float cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
        roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
        float sinp = +2.0 * (w * y - z * x);
        if (fabs(sinp) >= 1)
            pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = asin(sinp);

// yaw (z-axis rotation)
        float siny_cosp = +2.0 * (w * z + x * y);
        float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
        yaw = atan2(siny_cosp, cosy_cosp);
//    return yaw;
    }


    inline void yaw_to_quaternion(float yaw, double &qx, double &qy, double &qz, double &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    }
    inline void yaw_to_quaternion(float yaw, float &qx, float &qy, float &qz, float &qw) {
        float roll = 0.0;
        float pitch = 0.0;

        qx = 0.0;
        qy = 0.0;
        qz = sin(yaw * 0.5f);
        qw = cos(yaw * 0.5f);

        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);

    }
}

#endif //CMAKE_SUPER_BUILD_GEOMETRY_H
