#ifndef UTILS_H
#define UTILS_H

class Utils {
public:
    static inline float wrap_360(float a) { float x = std::fmod(a, 360.0f); return (x < 0) ? x + 360.0f : x; }
    static inline float wrap_pm180(float a) { float x = std::fmod(a + 180.0f, 360.0f); if (x < 0) x += 360.0f; return x - 180.0f; }
    float wrapError(float a){
    if (a > 180.0f) return a - 360.0f;
    else if (a < -180.0f) return a + 360.0f;
    else return a;
    }
    float grad2rad(float deg){ return deg * static_cast<float>(M_PI) / 180.0f; }
    float rad2grad(float rad){ return rad * 180.0f / static_cast<float>(M_PI); }

  static inline float pointAngX(float ang, float r){ return std::cos(ang) * r; }
  static inline float pointAngY(float ang, float r){ return std::sin(ang) * r; }
  static inline float getDiffAngle(float ang1, float r1, float ang2, float r2){
    const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
    const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
    return std::atan2(dy, dx); // rad
  }
  static inline float getEuclideanDistance(float ang1, float r1, float ang2, float r2){
    const float dx = pointAngX(ang2, r2) - pointAngX(ang1, r1);
    const float dy = pointAngY(ang2, r2) - pointAngY(ang1, r1);
    return std::hypot(dx, dy);
  }
};
#endif // UTILS_H