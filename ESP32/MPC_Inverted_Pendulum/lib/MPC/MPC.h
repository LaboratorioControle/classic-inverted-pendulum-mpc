#ifndef MPC_H
#define MPC_H

#include <qpOASES.hpp>
#include <Matrix.h>
#include <MessageHandling.hpp>

/**
 * @file MPC.h
 * @brief Implementation of a Model Predictive Control (MPC) controller
 * based on Quadratic Programming using the qpOASES solver.
 *
 * This library allows the use of different MPC formulations:
 * - Classic MPC
 * - MPC with linear parameterization (trivial)
 * - MPC with exponential parameterization
 */


/// MPC prediction horizon
#define N 35

/// Number of states in the system
#define n 4 

/// Number of restricted exits
#define nc 1

/// Number of regulated outputs
#define ny 2 

/// Number of inputs (control signals)
#define nu 1

/// Number of parameters for exponential and trivial parameterization
// O adequado para o Linear é 5
#define nre 5

/// Total number of decision variables in the parameterized system.
#define np (nu*nre) 

/// Total number of decision variables
#define nU (N*nu)

/// Total number of constraints in the original problem
#define nA (2*N*nc + 2*N*nu)

/// Total number of constraints in the parameterized problem
#define nAr (2*N*nc + 4*N*nu)

/**
 * @enum MPCForm
 * @brief Types of formulations available for the MPC
 */
enum class MPCForm {
    CLASSIC,
    LINEAR,
    EXPONENCIAL
};

/**
 * @class MPC
 * @brief Class responsible for implementing the MPC controller
 *
 * This class performs:
 *  - Cost matrix calculation
 *  - Constraint matrix calculation
 *  - QP problem formulation
 *  - Optimization problem resolution using qpOASES
 *  - Optimal control signal calculation
 */
class MPC {
public:
    /**
     * @brief State-space model matrices
     *
     * The matrix A represents the system dynamics:
     * 
     * x(k+1) = A x(k) + B u(k)
     */
    Matrix A;
    
    /**
     * @brief 
     *
     * The matrix B represents the input matrix of the system:
     * 
     * x(k+1) = A x(k) + B u(k)
     */
    Matrix B;

    /// Matrices that define restricted outputs
    Matrix Cc;
    
    /// Matrices that define restricted commands
    Matrix Dc;

    /// Matrix of regulated outputs
    Matrix Cr;

    /// Penalizes control effort
    Matrix Qu;
    
    /// Penalizes tracking error
    Matrix Qy;

    /// Maximum limits of the restricted outputs
    Matrix ycmax;
    
    /// Minimum limits of the restricted outputs
    Matrix ycmin;

    /// Maximum limits of the control signal variation
    Matrix deltamax;
    
    /// Minimum limits of the control signal variation
    Matrix deltamin;

    /// Maximum limits of the control signal
    Matrix umax;

    /// Minimum limits of the control signal
    Matrix umin;

    /**
     * @brief Constructor of the MPC class
     *
     * Initializes the MPC controller and the QP solver.
     *
     * @param form Type of MPC formulation
     * @param nWSR Maximum number of iterations for the QP solver
     */
    MPC(MPCForm form = MPCForm::CLASSIC, int nWSR = 30);

    /**
     * @brief Calculates all the matrices of the MPC (classic mode)
     */
    void compute_MPC_Matrices();

    /**
     * @brief Calculates the matrices of the MPC using linear parameterization
     *
     * @param pontos Points of control parameterization
     */
    void compute_MPC_Matrices(float* pontos);

    /**
     * @brief Calculates the matrices of the MPC using exponential parameterization
     *
     * @param lambda Vector of exponents
     * @param alpha damping factor
     * @param tau time constant
     */
    void compute_MPC_Matrices(float* lambda, float alpha, float tau);


    /**
     * @brief Calculates the optimal control signal
     *
     * Solves the MPC QP problem and returns the first command
     * of the calculated optimal sequence.
     *
     * @param ulast last control signal value applied
     * @param spt vector of desired setpoints
     * @param err vector of system errors
     * @return pointer to the calculated control signal vector
     */
    float* compute_MPC_Command(float ulast, float* spt, float* err);

    /**
     * @brief Getter for the solver result code
     *
     * This function allows external access to the result code of the
     * QP solver, which can be used for debugging and error handling.
     *
     * @return Integer representing the solver result code
     */
    int get_solver_result_code() const { return solver_result_code; }

    private:
    
    /// qpOASES optimizer pointer
    qpOASES::QProblem *qp = nullptr;

    /// Flag to save the initialization of the QP solver
    bool qp_initialized = false;

    /// Solver result Code
    int solver_result_code;

    /// Quantity of iterations of the optimizer
    int nWSR;
    
    /// Stores which was the chosen MPC option
    MPCForm form_;

    /// Cost function matrices (calculated offline)
    qpOASES::real_t H[nU * nU];
    qpOASES::real_t F1[nU * n];
    qpOASES::real_t F2[nU * N*ny];
    qpOASES::real_t F3[nU * nu];
    
    /// Constraint matrices (calculated offline)
    qpOASES::real_t Aineq[nA * nU];
    qpOASES::real_t G1[nA * n];
    qpOASES::real_t G2[nA * nu];
    qpOASES::real_t G3[nA];
    
    /// Vector of reference along the horizon
    qpOASES::real_t yref[N * ny];

    /// Vector of the cost function
    qpOASES::real_t F[nU];

    /// Vector of constraints
    qpOASES::real_t Bineq[nA];

    /// Vector solution of the QP problem
    qpOASES::real_t qp_opt[nU];
    
    /// Selection matrices for the trivial parametrized cases
    qpOASES::real_t Pi_r[nU * np];

    /// Selection matrices for the exponencial parametrized cases
    qpOASES::real_t Pi_e[nU * nre];

    /// Matrices for trivial and exponential parameterization
    qpOASES::real_t H_p[np * np];
    qpOASES::real_t Aineq_p[nAr * np];
    qpOASES::real_t F_p[np];
    qpOASES::real_t Bineq_p[nAr];
    
    /// Cost function matrices (calculated online)
    qpOASES::real_t utildemax[nU];
    qpOASES::real_t utildemin[nU]; 

    qpOASES::real_t lb_p[np];
    qpOASES::real_t ub_p[np];

    /// Auxiliary matrices for storing the first command of the calculated command signal sequence and the sequence itself
    qpOASES::real_t u_[nu];
    qpOASES::real_t u_full[nU];

    /**
     * @brief Generates the reference trajectory over the prediction horizon.
     *
     * This function builds the reference vector used by the MPC controller
     * along the prediction horizon. The reference trajectory is constructed
     * from the setpoint vector provided by the user and stored in the
     * internal vector yref.
     *
     * @param spt Pointer to the setpoint vector.
     */
    void generate_yref(const float* spt);

    /**
     * @brief Converts a Matrix object to a row-major vector of type real_t.
     *
     * The qpOASES solver requires matrices to be provided as contiguous
     * row-major arrays. This function converts the custom Matrix structure
     * used in the library into a row-major vector compatible with qpOASES.
     *
     * @param M Matrix to be converted.
     * @param result Pointer to the output vector where the matrix elements
     * will be stored in row-major format.
     */
    void matrix_to_realt(const Matrix& M, qpOASES::real_t* result);

    /**
     * @brief Computes the MPC cost matrices H, F1, F2, and F3.
     *
     * These matrices define the quadratic cost function of the MPC problem
     * and are computed offline since they depend only on the system model
     * and weighting matrices.
     *
     * The quadratic cost function has the form:
     *
     * J = (1/2) u^T H u + F^T u
     *
     * where the vector F is composed of terms involving F1, F2 and F3.
     */
    void compute_Cost_Matrices();
    
    /**
     * @brief Computes the constraint matrices G1, G2, and G3.
     *
     * These matrices represent the linear inequality constraints imposed
     * on the system outputs and control inputs along the prediction horizon.
     *
     * The constraints are written in the form:
     *
     * Aineq * u <= Bineq
     *
     * where the matrices G1, G2 and G3 are used to construct Bineq
     * depending on the current system state.
     */
    void compute_Constraints_Matrices();

    /**
     * @brief Builds the linear cost vector F.
     *
     * This function computes the linear term of the quadratic programming
     * cost function based on the current tracking error and reference
     * trajectory.
     *
     * The vector F is updated online since it depends on the current
     * system state and the tracking error.
     *
     * @param err Pointer to the error vector between reference and output.
     */
    void build_cost_vector(float* err);


    
    /**
     * @brief Builds the constraint vector Bineq.
     *
     * This function calculates the right-hand side of the inequality
     * constraint vector based on the current system state and the
     * previously applied control input.
     *
     * @param err Pointer to the system error vector.
     * @param ulast Last control input applied to the system.
     */
    void build_constraints(float* err, float ulast);

    /**
     * @brief Extracts the optimal control signal from the QP solution.
     *
     * After the quadratic program is solved, the optimal sequence of
     * control increments is stored in qp_opt. This function computes
     * the actual control signal to be applied and stores the first
     * control action in the vector u_.
     */
    void compute_util_opt();

    /**
     * @brief Initializes the QP solver parameters.
     *
     * This function allocates and configures the qpOASES solver instance
     * according to the size of the quadratic programming problem.
     *
     * @param nv_decision Number of decision variables in the QP problem.
     * @param nc_constraints Number of constraints in the QP problem.
     */
    void init_solver_qp(int nv_decision, int nc_constraints);

    /**
     * @brief Initializes the QP solver parameters.
     *
     * This function allocates and configures the qpOASES solver instance
     * according to the size of the quadratic programming problem.
     *
     * @param size_qp Number of optimization variables in the QP problem.
     */
    void solver_qp();

    /**
     * @brief Computes the reduced constraint vector Bineq for parameterized MPC.
     *
     * When control parametrization is used, the original optimization
     * variables are replaced by a reduced set of parameters. This function
     * transforms the original constraint vector accordingly.
     *
     * @param Pi_ref Pointer to the selection matrix used in the
     * control parametrization.
     */
    void compute_Bineq_reduced(qpOASES::real_t* Pi_ref);

    /**
     * @brief Computes the reduced constraint matrix Aineq for parameterized MPC.
     *
     * This function applies the control parametrization transformation
     * to the original constraint matrix Aineq.
     *
     * @param Pi_ref Pointer to the selection matrix used in the
     * control parametrization.
     */
    void compute_Aineq_reduced(qpOASES::real_t* Pi_ref);

    /**
     * @brief Computes the reduced constraint matrix Aineq for parameterized MPC.
     *
     * This function applies the control parametrization transformation
     * to the original constraint matrix Aineq.
     *
     * @param Pi_ref Pointer to the selection matrix used in the
     * control parametrization.
     */
    void compute_F_reduced(qpOASES::real_t* Pi_ref);

    /**
     * @brief Computes the reduced cost vector for parameterized MPC.
     *
     * The cost vector F is transformed according to the chosen
     * control parametrization to reduce the optimization problem
     * dimension.
     *
     * @param Pi_ref Pointer to the selection matrix used in the
     * control parametrization.
     */
    void compute_H_reduced(qpOASES::real_t* Pi_ref);
    
    /**
     * @brief Computes the exponential parametrization selection matrix.
     *
     * This function constructs the matrix Pi_e used to represent the
     * control sequence using exponential basis functions.
     *
     * @param lambda Vector of exponential parameters.
     * @param alpha Scaling parameter.
     * @param tau Time constant parameter.
     */
    void compute_Pi_e(float* lambda, float alpha, float tau);

    /**
     * @brief Computes the linear (trivial) parametrization selection matrix.
     *
     * This function builds the matrix Pi_r that maps the reduced
     * parametrization points to the full control sequence over the
     * prediction horizon.
     *
     * @param pontos Vector containing the parametrization points.
     */
    void compute_Pi_r(float* pontos);
};

#endif
