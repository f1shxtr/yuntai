// Core/Src/matrix.cpp
#include "matrix.h"
#include <cmath>
#include <cstring>

// 3x3 转置：res = mat^T
void Matrix33fTrans(const float mat[3][3], float res[3][3]) {
  res[0][0] = mat[0][0];
  res[0][1] = mat[1][0];
  res[0][2] = mat[2][0];

  res[1][0] = mat[0][1];
  res[1][1] = mat[1][1];
  res[1][2] = mat[2][1];

  res[2][0] = mat[0][2];
  res[2][1] = mat[1][2];
  res[2][2] = mat[2][2];
}

// 3x3 矩阵乘向量：res = mat * vec
void Matrix33fMultVector3f(const float mat[3][3], const float vec[3], float res[3]) {
  res[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2];
  res[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2];
  res[2] = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2];
}

void Vector3fAdd(const float a[3], const float b[3], float res[3]) {
  res[0] = a[0] + b[0];
  res[1] = a[1] + b[1];
  res[2] = a[2] + b[2];
}

void Vector3fSub(const float a[3], const float b[3], float res[3]) {
  res[0] = a[0] - b[0];
  res[1] = a[1] - b[1];
  res[2] = a[2] - b[2];
}

void Vector3fScale(const float k, const float vec[3], float res[3]) {
  res[0] = k * vec[0];
  res[1] = k * vec[1];
  res[2] = k * vec[2];
}

float Vector3fDot(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void Vector3fCross(const float a[3], const float b[3], float res[3]) {
  res[0] = a[1]*b[2] - a[2]*b[1];
  res[1] = a[2]*b[0] - a[0]*b[2];
  res[2] = a[0]*b[1] - a[1]*b[0];
}

float Vector3fNorm(const float vec[3]) {
  return std::sqrt(Vector3fDot(vec, vec));
}

float Vector4fNorm(const float vec[4]) {
  float s = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] + vec[3]*vec[3];
  return std::sqrt(s);
}

void Vector3fUnit(const float vec[3], float res[3]) {
  float n = Vector3fNorm(vec);
  if (n > 1e-12f) {
    res[0] = vec[0] / n;
    res[1] = vec[1] / n;
    res[2] = vec[2] / n;
  } else {
    // 若为零向量则返回零向量（避免 NaN）
    res[0] = res[1] = res[2] = 0.0f;
  }
}

void Vector4fUnit(const float vec[4], float res[4]) {
  float n = Vector4fNorm(vec);
  if (n > 1e-12f) {
    res[0] = vec[0] / n;
    res[1] = vec[1] / n;
    res[2] = vec[2] / n;
    res[3] = vec[3] / n;
  } else {
    res[0] = res[1] = res[2] = res[3] = 0.0f;
  }
}

void VectorCopy3f(const float src[3], float dst[3]) {
  std::memcpy(dst, src, 3 * sizeof(float));
}
