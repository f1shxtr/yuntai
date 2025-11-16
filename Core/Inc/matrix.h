// Core/Inc/matrix.h
#ifndef MATRIX_H
#define MATRIX_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

  typedef float float32_t;
  typedef int32_t arm_status;

#define ARM_MATH_SUCCESS ((arm_status)0)
#define ARM_MATH_SIZE_MISMATCH ((arm_status)-1)
#define ARM_MATH_ARGUMENT_ERROR ((arm_status)-2)
#ifndef ARM_MATH_SINGULAR
#define ARM_MATH_SINGULAR ((arm_status)-3)
#endif

  // 简单矩阵头，按项目需要可扩展
  typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float32_t* pData;
  } arm_matrix_instance_f32;
  typedef arm_matrix_instance_f32 Matrix;

  // 基本矩阵/向量工具（mahony.cpp 依赖）
  void Matrix33fTrans(const float mat[3][3], float res[3][3]);
  void Matrix33fMultVector3f(const float mat[3][3], const float vec[3], float res[3]);

  void Vector3fAdd(const float a[3], const float b[3], float res[3]);
  void Vector3fSub(const float a[3], const float b[3], float res[3]);
  void Vector3fScale(const float k, const float vec[3], float res[3]);
  float Vector3fDot(const float a[3], const float b[3]);
  void Vector3fCross(const float a[3], const float b[3], float res[3]);
  float Vector3fNorm(const float vec[3]);
  float Vector4fNorm(const float vec[4]);
  void Vector3fUnit(const float vec[3], float res[3]);
  void Vector4fUnit(const float vec[4], float res[4]);

  // 额外通用函数（可选）
  void VectorCopy3f(const float src[3], float dst[3]);

#ifdef __cplusplus
}
#endif

#endif // MATRIX_H
