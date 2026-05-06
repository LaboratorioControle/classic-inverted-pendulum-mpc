#include <vector>
#include <Arduino.h>

/**
 * @class Matrix
 * @brief Lightweight dynamic matrix class for embedded systems.
 * 
 * This class implements a simple matrix structure using a contiguous
 * std::vector<float> storage in row-major format. It provides basic
 * linear algebra operations required for control algorithms such as
 * Model Predictive Control (MPC).
 * 
 * The class is designed to be lightweight and suitable for microcontrollers,
 * avoiding heavy dependencies from full linear algebra libraries.
 */
class Matrix {
public:

    /**
     * @brief Number of rows of the matrix.
     */
    size_t r;

    /**
     * @brief Number of columns of the matrix.
     */
    size_t c;

    /**
     * @brief Matrix data stored in row-major order.
     * 
     * Element (i,j) is stored at index:
     * 
     * \f[
     * index = i \cdot c + j
     * \f]
     */
    std::vector<float> d;

    /**
     * @brief Default constructor.
     * 
     * Creates an empty matrix with zero rows and columns.
     */
    Matrix(): r(0), c(0) {}

    /**
     * @brief Matrix constructor.
     * 
     * Creates a matrix of size r × c initialized with value v.
     * 
     * @param r_ Number of rows
     * @param c_ Number of columns
     * @param v Initial value for all elements (default = 0.0)
     */
    Matrix(size_t r_, size_t c_, float v=0.0): r(r_), c(c_), d(r_*c_, v) {}

    /**
     * @brief Resize the matrix and fill with a given value.
     * 
     * Existing data is discarded.
     * 
     * @param r_ New number of rows
     * @param c_ New number of columns
     * @param v Value used to initialize all elements
     */
    void resize(size_t r_, size_t c_, float v=0.0){ 
        r=r_; 
        c=c_; 
        d.assign(r*c, v); 
    }

    /**
     * @brief Access matrix element (modifiable).
     * 
     * Provides write access to element (i,j).
     * 
     * @param i Row index
     * @param j Column index
     * @return Reference to the matrix element
     */
    float& operator()(size_t i, size_t j){ 
        return d[i*c + j]; 
    }

    /**
     * @brief Access matrix element (read-only).
     * 
     * Provides read-only access to element (i,j).
     * 
     * @param i Row index
     * @param j Column index
     * @return Value of the matrix element
     */
    float operator()(size_t i, size_t j) const { 
        return d[i*c + j]; 
    }

    /**
     * @brief Generate an identity matrix.
     * 
     * Creates a square matrix where diagonal elements are 1 and
     * all other elements are 0.
     * 
     * \f[
     * I =
     * \begin{bmatrix}
     * 1 & 0 & \dots & 0 \\
     * 0 & 1 & \dots & 0 \\
     * \vdots & \vdots & \ddots & \vdots \\
     * 0 & 0 & \dots & 1
     * \end{bmatrix}
     * \f]
     * 
     * @param n_ Size of the identity matrix
     * @return Identity matrix
     */
    static Matrix eye(int n_);

    /**
     * @brief Create a matrix filled with zeros.
     * 
     * @param r Number of rows
     * @param c Number of columns
     * @return Zero matrix
     */
    static Matrix zeros(int r, int c);

    /**
     * @brief Compute the transpose of a matrix.
     * 
     * The transpose swaps rows and columns:
     * 
     * \f[
     * A^T_{ij} = A_{ji}
     * \f]
     * 
     * @param A Input matrix
     * @return Transposed matrix
     */
    static Matrix transpose(const Matrix& A);

    /**
     * @brief Matrix multiplication.
     * 
     * Computes the product:
     * 
     * \f[
     * C = A \cdot B
     * \f]
     * 
     * Dimensions must satisfy:
     * 
     * \f[
     * A_{r \times c} \cdot B_{c \times k}
     * \f]
     * 
     * @param A Left matrix
     * @param B Right matrix
     * @return Resulting matrix
     */
    static Matrix mul(const Matrix& A, const Matrix& B);

    /**
     * @brief Scalar-matrix multiplication.
     * 
     * Computes:
     * 
     * \f[
     * B = sA
     * \f]
     * 
     * @param s Scalar value
     * @param A Input matrix
     * @return Scaled matrix
     */
    static Matrix smul(float s, const Matrix& A);

    /**
     * @brief Insert a matrix block into another matrix.
     * 
     * Copies matrix `src` into matrix `dst` starting at position (r0, c0).
     * 
     * @param dst Destination matrix
     * @param r0 Initial row index
     * @param c0 Initial column index
     * @param src Source matrix to insert
     */
    static void insertBlock(Matrix& dst, int r0, int c0, const Matrix& src);

    /**
     * @brief Generate the selection matrix \(P_i\).
     * 
     * This matrix is commonly used in MPC formulations to select
     * specific prediction steps from a stacked vector.
     * 
     * @param i Prediction step
     * @param n1 Output dimension
     * @param N_ Prediction horizon
     * @return Selection matrix
     */
    static Matrix P_i(int i, int n1, int N_);

};

/**
 * @brief Matrix addition operator.
 * 
 * Computes:
 * 
 * \f[
 * C = A + B
 * \f]
 * 
 * Both matrices must have the same dimensions.
 * 
 * @param A First matrix
 * @param B Second matrix
 * @return Resulting matrix
 */
Matrix operator+(const Matrix& A, const Matrix& B);

/**
 * @brief Matrix subtraction operator.
 * 
 * Computes:
 * 
 * \f[
 * C = A - B
 * \f]
 * 
 * Both matrices must have the same dimensions.
 * 
 * @param A First matrix
 * @param B Second matrix
 * @return Resulting matrix
 */
Matrix operator-(const Matrix& A, const Matrix& B);