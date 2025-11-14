/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix calculation. 矩阵运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#include "matrix.h"
#include <cmath>

template <int n_row, int n_col>
Matrix_f32<n_row, n_col>::Matrix_f32() {
  arm_mat_init_f32(&inst_, n_row, n_col, (float32_t*)data_);
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col>::Matrix_f32(const float32_t arr[n_row][n_col]) {
  arm_mat_init_f32(&inst_, n_row, n_col, (float32_t*)data_);
  memcpy(data_, arr, sizeof(data_));
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col>::Matrix_f32(const Matrix_f32<n_row, n_col>& mat) {
  arm_mat_init_f32(&inst_, n_row, n_col, (float32_t*)data_);
  memcpy(data_, mat.data_, sizeof(data_));
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col>& Matrix_f32<n_row, n_col>::operator=(
    const Matrix_f32<n_row, n_col>& mat) {
  arm_mat_init_f32(&inst_, n_row, n_col, (float32_t*)data_);
  memcpy(data_, mat.data_, sizeof(data_));
  return *this;
}

template <int n_row, int n_col>
int Matrix_f32<n_row, n_col>::rowNum() const {
  return n_row;
}
template <int n_row, int n_col>
int Matrix_f32<n_row, n_col>::colNum() const {
  return n_col;
}

template <int n_row, int n_col>
float32_t& Matrix_f32<n_row, n_col>::at(int idx) {
  return ((float32_t*)data_)[idx];
}
template <int n_row, int n_col>
float32_t& Matrix_f32<n_row, n_col>::at(int row_idx, int col_idx) {
  return data_[row_idx][col_idx];
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col> Matrix_f32<n_row, n_col>::operator+(
    const Matrix_f32<n_row, n_col>& mat) const {
  Matrix_f32<n_row, n_col> result;
  arm_mat_add_f32(&this->inst_, &mat.inst_, &result.inst_);
  return result;
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col>& Matrix_f32<n_row, n_col>::operator+=(
    const Matrix_f32<n_row, n_col>& mat) {
  arm_mat_add_f32(&this->inst_, &mat.inst_, &this->inst_);
  return *this;
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col> Matrix_f32<n_row, n_col>::operator-(
    const Matrix_f32<n_row, n_col>& mat) const {
  Matrix_f32<n_row, n_col> result;
  arm_mat_sub_f32(&this->inst_, &mat.inst_, &result.inst_);
  return result;
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col>& Matrix_f32<n_row, n_col>::operator-=(
    const Matrix_f32<n_row, n_col>& mat) {
  arm_mat_sub_f32(&this->inst_, &mat.inst_, &this->inst_);
  return *this;
}

template <int n_row, int n_col>
template <int n_col2>
Matrix_f32<n_row, n_col2> Matrix_f32<n_row, n_col>::operator*(
    const Matrix_f32<n_col, n_col2>& mat) const {
  Matrix_f32<n_row, n_col2> result;
  arm_mat_mult_f32(&this->inst_, &mat.inst_, &result.inst_);
  return result;
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col> Matrix_f32<n_row, n_col>::cwiseProduct(
    const Matrix_f32<n_row, n_col>& mat) const {
  Matrix_f32<n_row, n_col> result;
  {
    float32_t* pIn1 = (float32_t*)this->data_; /* input data matrix pointer A */
    float32_t* pIn2 = (float32_t*)mat.data_; /* input data matrix pointer B  */
    float32_t* pOut = (float32_t*)result.data_; /* output data matrix pointer */

    float32_t inA1, inA2, inB1, inB2, out1, out2; /* temporary variables */

    uint32_t numSamples; /* total number of elements in the matrix  */
    uint32_t blkCnt;     /* loop counters */

    /* Total number of samples in the input matrix */
    numSamples = (uint32_t)n_row * n_col;

    /* Run the below code for Cortex-M4 and Cortex-M3 */

    /* Loop unrolling */
    blkCnt = numSamples >> 2U;

    /* First part of the processing with loop unrolling.  Compute 4 outputs at a
     *time.
     ** a second loop below computes the remaining 1 to 3 samples. */
    while (blkCnt > 0U) {
      /* C(m,n) = A(m,n) + B(m,n) */
      /* Multiply and then store the results in the destination buffer. */
      /* Read values from source A */
      inA1 = pIn1[0];

      /* Read values from source B */
      inB1 = pIn2[0];

      /* Read values from source A */
      inA2 = pIn1[1];

      /* out = sourceA * sourceB */
      out1 = inA1 * inB1;

      /* Read values from source B */
      inB2 = pIn2[1];

      /* Read values from source A */
      inA1 = pIn1[2];

      /* out = sourceA * sourceB */
      out2 = inA2 * inB2;

      /* Read values from source B */
      inB1 = pIn2[2];

      /* Store result in destination */
      pOut[0] = out1;
      pOut[1] = out2;

      /* Read values from source A */
      inA2 = pIn1[3];

      /* Read values from source B */
      inB2 = pIn2[3];

      /* out = sourceA + sourceB */
      out1 = inA1 * inB1;

      /* out = sourceA * sourceB */
      out2 = inA2 * inB2;

      /* Store result in destination */
      pOut[2] = out1;

      /* Store result in destination */
      pOut[3] = out2;

      /* update pointers to process next sampels */
      pIn1 += 4U;
      pIn2 += 4U;
      pOut += 4U;
      /* Decrement the loop counter */
      blkCnt--;
    }

    /* If the numSamples is not a multiple of 4, compute any remaining output
     *samples here.
     ** No loop unrolling is used. */
    blkCnt = numSamples % 0x4U;

    while (blkCnt > 0U) {
      /* C(m,n) = A(m,n) * B(m,n) */
      /* Multiply and then store the results in the destination buffer. */
      *pOut++ = (*pIn1++) * (*pIn2++);

      /* Decrement the loop counter */
      blkCnt--;
    }
  }
  return result;
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col> Matrix_f32<n_row, n_col>::operator*(
    const float32_t& gain) const {
  Matrix_f32<n_row, n_col> result;
  arm_mat_scale_f32(&this->inst_, gain, &result.inst_);
  return result;
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col>& Matrix_f32<n_row, n_col>::operator*=(
    const float32_t& gain) {
  arm_mat_scale_f32(&this->inst_, gain, &this->inst_);
  return *this;
}
template <int n_row, int n_col>
Matrix_f32<n_row, n_col> operator*(const float32_t& gain,
                                   const Matrix_f32<n_row, n_col>& mat) {
  Matrix_f32<n_row, n_col> result;
  arm_mat_scale_f32(&mat.inst_, gain, &result.inst_);
  return result;
}

template <int n_row, int n_col>
Matrix_f32<n_col, n_row> Matrix_f32<n_row, n_col>::transpose() const {
  Matrix_f32<n_col, n_row> result;
  arm_mat_trans_f32(&this->inst_, &result.inst_);
  return result;
}

template <int n_row, int n_col>
Matrix_f32<n_row, n_col> Matrix_f32<n_row, n_col>::inverse() const {
  Matrix_f32<n_row, n_col> mat(*this);
  Matrix_f32<n_row, n_col> result;
  arm_mat_inverse_f32(&mat.inst_, &result.inst_);
  return result;
}

template class Matrix_f32<10, 10>;
template class Matrix_f32<10, 4>;
template class Matrix_f32<10, 1>;
template class Matrix_f32<4, 10>;
template class Matrix_f32<4, 1>;
template class Matrix_f32<2, 2>;
template class Matrix_f32<2, 1>;

template Matrix_f32<10, 1> Matrix_f32<10, 10>::operator*
    <1>(const Matrix_f32<10, 1>& mat) const;
template Matrix_f32<10, 1> Matrix_f32<10, 4>::operator*
    <1>(const Matrix_f32<4, 1>& mat) const;
template Matrix_f32<4, 1> Matrix_f32<4, 10>::operator*
    <1>(const Matrix_f32<10, 1>& mat) const;
template Matrix_f32<2, 1> Matrix_f32<2, 2>::operator*
    <1>(const Matrix_f32<2, 1>& mat) const;

// Pay attention to the size of storage space
arm_status MatrixResize(Matrix* mat, uint16_t nRows, uint16_t nCols) {
  mat->numRows = nRows;
  mat->numCols = nCols;
  return ARM_MATH_SUCCESS;
}

// Matrix inverse calculation (Gaussian elimination)
arm_status MatrixInv(const Matrix* pSrc, Matrix* pDst) {
  // Check matrix size
  if (pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols ||
      pSrc->numRows != pSrc->numCols) {
    return ARM_MATH_SIZE_MISMATCH;
  }
  int n = pSrc->numRows;

  // extended matrix [A|I]
  Matrix ext_mat;
  float* ext_mat_data = new float[2 * n * n];
  MatrixInit(&ext_mat, n, 2 * n, ext_mat_data);
  VectorFill(0, ext_mat_data, 2 * n * n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      ext_mat_data[2 * n * i + j] = *(pSrc->pData + i * n + j);
    }
    ext_mat_data[2 * n * i + n + i] = 1;
  }

  // elimination
  for (int i = 0; i < n; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat_data[2 * n * i + i]);
    int abs_max_row = i;
    for (int row = i; row < n; row++) {
      if (abs_max < fabs(ext_mat_data[2 * n * row + i])) {
        abs_max = fabs(ext_mat_data[2 * n * row + i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      delete[] ext_mat_data;
      return ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // row exchange
      float tmp;
      for (int j = i; j < 2 * n; j++) {
        tmp = ext_mat_data[2 * n * i + j];
        ext_mat_data[2 * n * i + j] = ext_mat_data[2 * n * abs_max_row + j];
        ext_mat_data[2 * n * abs_max_row + j] = tmp;
      }
    }
    float k = 1.f / ext_mat_data[2 * n * i + i];
    for (int col = i; col < 2 * n; col++) {
      ext_mat_data[2 * n * i + col] *= k;
    }
    for (int row = 0; row < n; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat_data[2 * n * row + i];
      for (int j = i; j < 2 * n; j++) {
        ext_mat_data[2 * n * row + j] -= k * ext_mat_data[2 * n * i + j];
      }
    }
  }

  // inv = ext_mat(:,n+1:2n)
  for (int row = 0; row < n; row++) {
    memcpy((float*)pDst->pData + n * row, &ext_mat_data[2 * n * row + n],
           n * sizeof(float));
  }

  delete[] ext_mat_data;
  return ARM_MATH_SUCCESS;
}

void Matrix33fTrans(float mat[3][3], float res[3][3]) {
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

void Matrix33fMultVector3f(float mat[3][3], const float vec[3], float res[3]) {
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
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Vector3fCross(const float a[3], const float b[3], float res[3]) {
  res[0] = a[1] * b[2] - a[2] * b[1];
  res[1] = a[2] * b[0] - a[0] * b[2];
  res[2] = a[0] * b[1] - a[1] * b[0];
}

float Vector3fNorm(const float vec[3]) {
  return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

float Vector4fNorm(const float vec[4]) {
  return sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2] +
               vec[3] * vec[3]);
}

void Vector3fUnit(const float vec[3], float res[3]) {
  float norm = Vector3fNorm(vec);
  if (norm > 1e-8f) {
    res[0] = vec[0] / norm;
    res[1] = vec[1] / norm;
    res[2] = vec[2] / norm;
  } else {
    res[0] = vec[0];
    res[1] = vec[1];
    res[2] = vec[2];
  }
}

void Vector4fUnit(const float vec[4], float res[4]) {
  float norm = Vector4fNorm(vec);
  if (norm > 1e-8f) {
    res[0] = vec[0] / norm;
    res[1] = vec[1] / norm;
    res[2] = vec[2] / norm;
    res[3] = vec[3] / norm;
  } else {
    res[0] = vec[0];
    res[1] = vec[1];
    res[2] = vec[2];
    res[3] = vec[3];
  }
}
