#include "Matrix.h"

Matrix Matrix::eye(int n_){ 
    Matrix I(n_,n_); 
    for(int i=0; i<n_; i++) 
        I(i,i)=1.0; 
    return I; 
}

Matrix Matrix::zeros(int r, int c){ 
    return Matrix(r,c,0.0); 
}

Matrix Matrix::transpose(const Matrix& A){ 
    Matrix B(A.c, A.r); 
    for(int i=0;i<A.r;i++) 
        for(int j=0;j<A.c;j++) 
            B(j,i)=A(i,j); 
    return B; 
}

Matrix operator+(const Matrix& A, const Matrix& B){ 
    assert(A.r==B.r && A.c==B.c); 
    Matrix C(A.r,A.c); 
    for(int i=0;i<A.r*A.c;i++) 
        C.d[i]=A.d[i]+B.d[i]; 
    return C; 
}

Matrix operator-(const Matrix& A, const Matrix& B){ 
    assert(A.r==B.r && A.c==B.c); 
    Matrix C(A.r,A.c); 
    for(int i=0;i<A.r*A.c;i++) 
        C.d[i]=A.d[i]-B.d[i]; 
    return C; 
}

Matrix Matrix::mul(const Matrix& A, const Matrix& B){ 
    assert(A.c==B.r); 
    Matrix C(A.r,B.c,0.0); 
    for(int i=0;i<A.r;i++) 
        for(int k=0;k<A.c;k++){ 
            float aik=A(i,k); 
            for(int j=0;j<B.c;j++) 
                C(i,j)+=aik * B(k,j); 
        } 
    return C; 
}

Matrix Matrix::smul(float s, const Matrix& A){ 
    Matrix C(A.r,A.c); 
    for(int i=0;i<A.r*A.c;i++) 
        C.d[i]=s*A.d[i]; 
    return C; 
}

void Matrix::insertBlock(Matrix& dst, int r0, int c0, const Matrix& src){
    assert(r0+src.r <= dst.r && c0+src.c <= dst.c);
    for(int i=0;i<src.r;i++) 
        for(int j=0;j<src.c;j++) 
            dst(r0+i,c0+j)=src(i,j);
}

Matrix Matrix::P_i(int i, int n1, int N_){
    Matrix f(n1, N_*n1, 0.0);
    for(int r=0;r<n1;r++) 
        f(r,(i-1)*n1 + r) = 1.0;
    return f;
}