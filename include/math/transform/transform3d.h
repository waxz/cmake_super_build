//
// Created by waxz on 5/14/24.
//

#ifndef LIBROSCPP_TRANSFORM3D_H
#define LIBROSCPP_TRANSFORM3D_H



#ifdef __cplusplus
extern "C" {
#endif

    // [3*4] * [3*n]
    inline int se3_mul(float *MatA,float* MatB,float* MatC, size_t MatBCol){

        for (size_t j = 0; j < MatBCol; ++j){
            // Calculate the j-th column of the result in-place (in B) using the helper array
            for (size_t i = 0; i < 3; ++i) {
                size_t l = i*3 + j;
                MatC[l] = 0;
                for (size_t k = 0; k < 3; ++k) {
                    MatC[l] += MatA[i*4 + k] * MatB[k * 3 + j];
                }
                MatC[l] +=  MatA[i*4 + 3];

            }
        }
        return 0;
    }

inline int matrixf_mul(float *MatA, size_t MatARow, size_t MatACol, float* MatB,size_t MatBRow, size_t MatBCol ) {
    const size_t MAX_DIM = 4;
    if(MatACol!= MatBRow ||  MatACol> MAX_DIM){
        return -1;
    }
    int A[3][3] = {{11, 12, 13},
                   {21, 22, 23},
                   {31, 32, 33}};
    int B[3][5] = {{11, 12, 13, 14, 15},
                   {21, 22, 23, 24, 25},
                   {31, 32, 33, 34, 35}};
    int C[3]; // Helper array
    for (size_t j = 0; j < MatBCol; ++j) {
        // Copy the j-th column of B into the helper array
        for (size_t i = 0; i < MatBRow; ++i) {
            C[i] = B[i][j];
        }
        // Calculate the j-th column of the result in-place (in B) using the helper array
        for (size_t i = 0; i < MatACol; ++i) {
            B[i][j] = 0;
            for (size_t k = 0; k < MatACol; ++k) {
                B[i][j] += A[i][k] * C[k];
            }
        }
    }
    return 0;
};

#ifdef __cplusplus
}
#endif
#endif //LIBROSCPP_TRANSFORM3D_H
