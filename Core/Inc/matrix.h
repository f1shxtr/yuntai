/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "lib/arm_math/arm_math.h"

template <int n_row, int n_col>
class Matrix_f32 {
  template <int n_row_, int n_col_>
  friend class Matrix_f32;

 protected:
  arm_matrix_instance_f32 inst_;
  float32_t data_[n_row][n_col];

 public:
  Matrix_f32();
  Matrix_f32(const float32_t arr[n_row][n_col]);
  Matrix_f32(const Matrix_f32<n_row, n_col>& mat);

  Matrix_f32<n_row, n_col>& operator=(const Matrix_f32<n_row, n_col>& mat);

  int rowNum() const;
  int colNum() const;

  float32_t& at(int idx);
  float32_t& at(int row_idx, int col_idx);

  Matrix_f32<n_row, n_col> operator+(const Matrix_f32<n_row, n_col>& mat) const;
  Matrix_f32<n_row, n_col>& operator+=(const Matrix_f32<n_row, n_col>& mat);

  Matrix_f32<n_row, n_col> operator-(const Matrix_f32<n_row, n_col>& mat) const;
  Matrix_f32<n_row, n_col>& operator-=(const Matrix_f32<n_row, n_col>& mat);

  // 矩阵点积
  // C=AB -> C[i][j] = sum{ A[i][k] * B[k][j] }
  template <int n_col2>
  Matrix_f32<n_row, n_col2> operator*(
      const Matrix_f32<n_col, n_col2>& mat) const;

  // 元素乘法(Hadamard积)
  // C=AB -> C[i][j] = A[i][j] * B[i][j]
  Matrix_f32<n_row, n_col> cwiseProduct(
      const Matrix_f32<n_row, n_col>& mat) const;

  // 矩阵数乘
  Matrix_f32<n_row, n_col> operator*(const float32_t& gain) const;
  Matrix_f32<n_row, n_col>& operator*=(const float32_t& gain);
  template <int n_row_, int n_col_>
  friend Matrix_f32<n_row_, n_col_> operator*(
      const float32_t& gain, const Matrix_f32<n_row_, n_col_>& mat);

  Matrix_f32<n_col, n_row> transpose() const;

  Matrix_f32<n_row, n_col> inverse() const;
};

// Matrix and vector based on arm_math
typedef arm_matrix_instance_f32 Matrix;

// Matrix initialization
#define MatrixInit(pMat, nRows, nCols, pData) \
  arm_mat_init_f32(pMat, nRows, nCols, pData)
// Pay attention to the size of storage space when resize matrix
arm_status MatrixResize(Matrix* mat, uint16_t nRows, uint16_t nCols);

// Matrix calculation
// return: ARM_MATH_SIZE_MISMATCH or ARM_MATH_SUCCESS
#define MatrixAdd(pSrcA, pSrcB, pDst) arm_mat_add_f32(pSrcA, pSrcB, pDst)
#define MatrixSub(pSrcA, pSrcB, pDst) arm_mat_sub_f32(pSrcA, pSrcB, pDst)
#define MatrixScale(pSrc, scale, pDst) arm_mat_scale_f32(pSrc, scale, pDst)
#define MatrixMult(pSrcA, pSrcB, pDst) arm_mat_mult_f32(pSrcA, pSrcB, pDst)
#define MatrixTrans(pSrc, pDst) arm_mat_trans_f32(pSrc, pDst)
// #define MatrixInv(pSrc, pDst) arm_mat_inverse_f32(pSrc, pDst)
// Matrix inverse calculation
// arm_mat_inverse_f32() in arm_math is incorrect!
arm_status MatrixInv(const Matrix* pSrc, Matrix* pDst);

// Vector calculation
#define VectorAdd(pSrcA, pSrcB, pDst, blockSize) \
  arm_add_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorSub(pSrcA, pSrcB, pDst, blockSize) \
  arm_sub_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorScale(pSrc, scale, pDst, blockSize) \
  arm_scale_f32(pSrc, scale, pDst, blockSize)
#define VectorAbs(pSrc, pDst, blockSize) arm_abs_f32(pSrc, pDst, blockSize)
#define VectorDot(pSrcA, pSrcB, blockSize, pRes) \
  arm_dot_prod_f32(pSrcA, pSrcB, blockSize, pRes)
#define VectorMult(pSrcA, pSrcB, pDst, blockSize) \
  arm_mult_f32(pSrcA, pSrcB, pDst, blockSize)
#define VectorOffset(pSrc, offset, pDst, blockSize) \
  arm_offset_f32(pSrc, offset, pDst, blockSize)
#define VectorNegate(pSrc, pDst, blockSize) \
  arm_negate_f32(pSrc, pDst, blockSize)
#define VectorCopy(pSrc, pDst, blockSize) arm_copy_f32(pSrc, pDst, blockSize)
#define VectorFill(value, pDst, blockSize) arm_fill_f32(value, pDst, blockSize)
#define VectorPower(pSrc, blockSize, pRes) arm_power_f32(pSrc, blockSize, pRes)

// Statistics
#define VectorMean(pSrc, blockSize, pRes) arm_mean_f32(pSrc, blockSize, pRes)
#define VectorVar(pSrc, blockSize, pRes) arm_var_f32(pSrc, blockSize, pRes)
#define VectorRms(pSrc, blockSize, pRes) arm_rms_f32(pSrc, blockSize, pRes)
#define VectorStd(pSrc, blockSize, pRes) arm_std_f32(pSrc, blockSize, pRes)
#define VectorMin(pSrc, blockSize, pRes, pIndex) \
  arm_min_f32(pSrc, blockSize, pRes, pIndex)
#define VectorMax(pSrc, blockSize, pRes, pIndex) \
  arm_max_f32(pSrc, blockSize, pRes, pIndex)

// Vector calculation impementation
void Matrix33fTrans(float mat[3][3], float res[3][3]);
void Matrix33fMultVector3f(float mat[3][3], const float vec[3], float res[3]);
void Vector3fAdd(const float a[3], const float b[3], float res[3]);
void Vector3fSub(const float a[3], const float b[3], float res[3]);
void Vector3fScale(const float k, const float vec[3], float res[3]);
float Vector3fDot(const float a[3], const float b[3]);
void Vector3fCross(const float a[3], const float b[3], float res[3]);
float Vector3fNorm(const float vec[3]);
float Vector4fNorm(const float vec[4]);
void Vector3fUnit(const float vec[3], float res[3]);
void Vector4fUnit(const float vec[4], float res[4]);

#endif  // MATRIX_H