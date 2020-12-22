#pragma once

#include <cmath>
#include <iostream>
class Vector3f {
 public:
  Vector3f() : x(0.0f), y(0.0f), z(0.0f) {}
  Vector3f(float xx) : x(xx), y(xx), z(xx) {}
  Vector3f(float xx, float yy, float zz) : x(xx), y(yy), z(zz) {}
  Vector3f operator*(const float& r) const {
    return Vector3f(x * r, y * r, z * r);
  }
  Vector3f operator/(const float& r) const {
    return Vector3f(x / r, y / r, z / r);
  }

  Vector3f operator*(const Vector3f& v) const {
    return Vector3f(x * v.x, y * v.y, z * v.z);
  }
  Vector3f operator-(const Vector3f& v) const {
    return Vector3f(x - v.x, y - v.y, z - v.z);
  }
  Vector3f operator+(const Vector3f& v) const {
    return Vector3f(x + v.x, y + v.y, z + v.z);
  }
  Vector3f operator-() const { return Vector3f(-x, -y, -z); }
  Vector3f& operator+=(const Vector3f& v) {
    x += v.x, y += v.y, z += v.z;
    return *this;
  }
  float operator[](int i) {
    assert(i >= 0 && i <= 2);
    if (i == 0) {
      return x;
    } else if (i == 1) {
      return y;
    }
    return z;
  }
  friend Vector3f operator*(const float& r, const Vector3f& v) {
    return Vector3f(v.x * r, v.y * r, v.z * r);
  }
  friend std::ostream& operator<<(std::ostream& os, const Vector3f& v) {
    return os << v.x << ", " << v.y << ", " << v.z;
  }
  float x, y, z;
};

class Vector3i {
 public:
  int x, y, z;
  Vector3i() : x(0), y(0), z(0) {}
  Vector3i(int xx) : x(xx), y(xx), z(xx) {}
  Vector3i(int xx, int yy, int zz) : x(xx), y(yy), z(zz) {}
  Vector3i operator*(const int& r) const {
    return Vector3i(x * r, y * r, z * r);
  }

  Vector3i operator*(const Vector3i& v) const {
    return Vector3i(x * v.x, y * v.y, z * v.z);
  }
  Vector3i operator-(const Vector3i& v) const {
    return Vector3i(x - v.x, y - v.y, z - v.z);
  }
  Vector3i operator+(const Vector3i& v) const {
    return Vector3i(x + v.x, y + v.y, z + v.z);
  }
  int operator[](int i) {
    assert(i >= 0 && i <= 2);
    if (i == 0) {
      return x;
    } else if (i == 1) {
      return y;
    }
    return z;
  }
  friend std::ostream& operator<<(std::ostream& os, const Vector3i& v) {
    return os << v.x << ", " << v.y << ", " << v.z;
  }
};

inline float dotProduct(const Vector3f& a, const Vector3f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vector3f crossProduct(const Vector3f& a, const Vector3f& b) {
  return Vector3f(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}

inline Vector3f normalize(const Vector3f& v) {
  float mag2 = v.x * v.x + v.y * v.y + v.z * v.z;
  if (mag2 > 0) {
    float invMag = 1 / sqrtf(mag2);
    return Vector3f(v.x * invMag, v.y * invMag, v.z * invMag);
  }

  return v;
}

inline float vlen(const Vector3f& v) {
  float mag = v.x * v.x + v.y * v.y + v.z * v.z;
  return sqrtf(mag);
}