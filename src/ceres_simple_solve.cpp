//
// Created by waxz on 8/27/22.
//
#include <deque>
#include "transform/transform.h"

#if 1

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::NumericDiffCostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

using Grid = ceres::Grid2D<double>;
using Interpolator = ceres::BiCubicInterpolator<Grid>;

// Cost-function using autodiff interface of BiCubicInterpolator
struct AutoDiffBiCubicCost {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    template<typename T>
    bool operator()(const T *s, T *residual) const {

        using Vector2T = Eigen::Matrix<T, 2, 1>;
        Eigen::Map<const Vector2T> shift(s);

        const Vector2T point = point_ + shift;

        T v;
        interpolator_.Evaluate(point.y(), point.x(), &v);
        *residual = T(0.0);


        *residual = T(v);;//- value_;
        return true;
    }

    AutoDiffBiCubicCost(const Interpolator &interpolator,
                        Eigen::Vector2d point,
                        double value)
            : point_(std::move(point)), value_(value), interpolator_(interpolator) {}

    static ceres::CostFunction *Create(const Interpolator &interpolator,
                                       const Eigen::Vector2d &point,
                                       double value) {
        return new ceres::AutoDiffCostFunction<AutoDiffBiCubicCost, 1, 2>(
                new AutoDiffBiCubicCost(interpolator, point, value));
    }

    const Eigen::Vector2d point_;
    const double value_;
    const Interpolator &interpolator_;
};

// Function for input data generation
static double f(const double &x, const double &y) {
    return x * x - y * x + y * y;
}


#endif
// A cost functor that implements the residual r = 10 - x.


//#include "tinyceres/tiny_solver_autodiff_function.hpp"
//#include "tinyceres/tiny_solver.hpp"

using FloatType = double;

struct CostFunctor {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 * std::abs(10.0 - x[0]);
        return true;
    }
};

struct CostFunctor2 {
    template<typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = 10.0 * std::abs(20.0 - x[1]);
        return true;
    }
};

struct CostFunctor3 {
    template<typename T>
    bool operator()(const T *const x, const T *y, T *residual) const {
        residual[0] = std::abs(x[0] - y[0]) + std::abs(x[1] - y[1]);
        return true;
    }
};

template<typename T1, typename T2,
        typename std::enable_if<std::is_constructible<T1, T2>{},
                bool>::type = true>
void simple_cast(T1 &value1, const T2 &value2) {

    value1 = static_cast<T1>(value2);
}

template<typename T1, typename T2, long unsigned int N>
void simple_cast(std::array<T1, N> &value1, const std::array<T2, N> &value2) {
    for (int i = 0; i < value2.size(); i++) {
        simple_cast(value1[i], value2[i]);
    }
}


template<typename T1, typename T2>
void simple_cast(std::vector<T1> &value1, const std::vector<T2> &value2) {
    value1.resize(value2.size());
    for (int i = 0; i < value2.size(); i++) {
        simple_cast(value1[i], value2[i]);
    }
}


struct PoseConstrains {

    Eigen::Matrix<double, 3, 3> a_Tcap_b;

    Eigen::Matrix<double, 3, 3> base_laser_inv;

    double weight_x = 1.0;
    double weight_y = 1.0;
    double weight_yaw = 1.0;


    template<typename ...ARGS>
    static CostFunction *create(ARGS ...args) {
        return new NumericDiffCostFunction<PoseConstrains, CENTRAL, 3, 3, 3>(new PoseConstrains(args...));
    }


    PoseConstrains(double x, double y, double yaw, double weight_x_ = 1.0, double weight_y_ = 1.0,
                   double weight_yaw_ = 1.0) {
        using T = double;
        weight_x = weight_x_;
        weight_y = weight_y_;
        weight_yaw = weight_yaw_;

        T cos_t = T(cos(yaw));
        T sin_t = T(sin(yaw));
        a_Tcap_b(0, 0) = cos_t;
        a_Tcap_b(0, 1) = -sin_t;
        a_Tcap_b(1, 0) = sin_t;
        a_Tcap_b(1, 1) = cos_t;
        a_Tcap_b(0, 2) = x;
        a_Tcap_b(1, 2) = y;

        a_Tcap_b(2, 0) = T(0.0);
        a_Tcap_b(2, 1) = T(0.0);
        a_Tcap_b(2, 2) = T(1.0);
    }

    template<typename T>
    bool operator()(const T *const P1, const T *const P2, T *residual) const {


        // Convert P1 to T1 ^w_T_a
        Eigen::Matrix<T, 3, 3> w_T_a;
        {
            T cos_t = T(cos(P1[2]));
            T sin_t = T(sin(P1[2]));
            w_T_a(0, 0) = cos_t;
            w_T_a(0, 1) = -sin_t;
            w_T_a(1, 0) = sin_t;
            w_T_a(1, 1) = cos_t;
            w_T_a(0, 2) = P1[0];
            w_T_a(1, 2) = P1[1];

            w_T_a(2, 0) = T(0.0);
            w_T_a(2, 1) = T(0.0);
            w_T_a(2, 2) = T(1.0);
        }


        // Convert P2 to T2 ^w_T_a
        Eigen::Matrix<T, 3, 3> w_T_b;
        {
            T cos_t = cos(P2[2]);
            T sin_t = sin(P2[2]);
            w_T_b(0, 0) = cos_t;
            w_T_b(0, 1) = -sin_t;
            w_T_b(1, 0) = sin_t;
            w_T_b(1, 1) = cos_t;
            w_T_b(0, 2) = P2[0];
            w_T_b(1, 2) = P2[1];

            w_T_b(2, 0) = T(0.0);
            w_T_b(2, 1) = T(0.0);
            w_T_b(2, 2) = T(1.0);
        }

        // cast from double to T
        Eigen::Matrix<T, 3, 3> T_a_Tcap_b;
#if 0
        T_a_Tcap_b <<   T(a_Tcap_b(0,0)), T(a_Tcap_b(0,1)),T(a_Tcap_b(0,2)),
                T(a_Tcap_b(1,0)), T(a_Tcap_b(1,1)),T(a_Tcap_b(1,2)),
                T(a_Tcap_b(2,0)), T(a_Tcap_b(2,1)),T(a_Tcap_b(2,2));
#endif


        T_a_Tcap_b = a_Tcap_b.template cast<T>();

        // now we have :: w_T_a, w_T_b and a_Tcap_b
        // compute pose difference
        Eigen::Matrix<T, 3, 3> diff = T_a_Tcap_b.inverse() * (w_T_a.inverse() * w_T_b);


        residual[0] = diff(0, 2) * weight_x;
        residual[1] = diff(1, 2) * weight_y;
        residual[2] = ceres::asin(diff(1, 0)) * weight_yaw;

//        std::cout << "a_Tcap_b:\n" << a_Tcap_b << "\n" ;
//        std::cout << "w_T_a:\n" << w_T_a << "\n" ;
//        std::cout << "w_T_b:\n" << w_T_b<< "\n" ;
//        std::cout << "diff:\n" << diff<< "\n" ;
//        std::cout << " residual[0]:\n" <<  residual[0] << "\n" ;
//        std::cout << " residual[1]:\n" <<  residual[1] << "\n" ;
//        std::cout << " residual[2]:\n" <<  residual[2] << "\n" ;

        return true;

    }

};

void test() {


    int a;
    simple_cast(a, 3);
    simple_cast(a, 3.4);

    std::vector<double> v0;

    std::vector<float> v1;
    simple_cast(v0, v1);


    std::vector<std::array<float, 3>> markers2;
    std::vector<std::vector<std::array<float, 3>>> clusters2;

    std::vector<std::array<double, 3>> markers;
    std::vector<std::vector<std::array<double, 3>>> clusters;

    for (int i = 0; i < 10; i++) {
        markers2.push_back(std::array<float, 3>{static_cast<float>(i), 1.0, 2.0});
        markers2.push_back(std::array<float, 3>{10.0f - static_cast<float>(i), 1.0, 2.0});

    }
    int k = markers2.size() > 5 ? 5 : markers2.size();


    std::nth_element(markers2.begin(), markers2.begin() + k, markers2.end(), [](auto &v1, auto &v2) {
        return v1[0] > v2[0];
    });


    clusters2.push_back(markers2);


    simple_cast(markers, markers2);
    simple_cast(clusters, clusters2);

    std::cout << "check markers\n";
    for (auto &i: markers) {

        std::cout << i[0] << ", " << i[1] << ", " << i[2] << std::endl;
    }
    std::cout << "done check markers\n";

    std::cout << "check clusters\n";
    for (auto &m: clusters) {
        for (auto &i: m) {

            std::cout << i[0] << ", " << i[1] << ", " << i[2] << std::endl;
        }
    }

    std::cout << "done check clusters\n";
}


void test_ceres() {
    // The variable to solve for with its initial value. It will be
    // mutated in place by the solver.
    FloatType x = 0.5;
    FloatType initial_x[2] = {x, x};
    FloatType initial_x2[2] = {x, x};

    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).
    CostFunction *cost_function = new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 2>(new CostFunctor);
    CostFunction *cost_function2 = new NumericDiffCostFunction<CostFunctor2, CENTRAL, 1, 2>(new CostFunctor2);
    CostFunction *cost_function3 = new NumericDiffCostFunction<CostFunctor3, CENTRAL, 1, 2, 2>(new CostFunctor3);


    problem.AddResidualBlock(cost_function, nullptr, initial_x);
    problem.AddResidualBlock(cost_function2, nullptr, initial_x2);
    problem.AddResidualBlock(cost_function3, nullptr, initial_x, initial_x2);

    // Run the solver!

    ceres::Solver::Options ceres_solver_options;
//    ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres_solver_options.minimizer_progress_to_stdout = true;
    ceres_solver_options.num_threads = 1;
    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
    ceres_solver_options.max_num_iterations = 100;
    ceres_solver_options.gradient_tolerance = 1e-3;
    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres_solver_options.parameter_tolerance = 1e-6;


    Solver::Summary summary;
    Solve(ceres_solver_options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "final    summary.initial_cost : " << summary.initial_cost << ",    summary.final_cost : "
              << summary.final_cost << std::endl;
    std::cout << "final initial_x : " << initial_x[0] << ", " << initial_x[1] << std::endl;
    std::cout << "final initial_x2 : " << initial_x2[0] << ", " << initial_x2[1] << std::endl;

}


void test_ceres_pose_graph() {

    FloatType initial_x1[3] = {0.0, 0.0, 0.0};
    FloatType initial_x2[3] = {0.45, 0.1, 0.4};
    FloatType initial_x3[3] = {0.45, 0.1, 0.4};

    // Build the problem.
    Problem problem;
    // Set up the only cost function (also known as residual). This uses
    // numeric differentiation to obtain the derivative (jacobian).
//    CostFunction* cost_function =  new NumericDiffCostFunction<PoseConstrains, CENTRAL, 3, 3,3>(new PoseConstrains(0.5,0.0,0.5));

    CostFunction *cost_function = PoseConstrains::create(0.5, 0.1, 0.0, 0.5, 0.5, 0.9);
    CostFunction *cost_function2 = PoseConstrains::create(0.1, 0.3, 0.2, 0.5, 0.5, 0.9);

    problem.AddResidualBlock(cost_function, nullptr, initial_x1, initial_x2);
    problem.AddResidualBlock(cost_function2, nullptr, initial_x2, initial_x3);

    // Run the solver!

    ceres::Solver::Options ceres_solver_options;
//    ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres_solver_options.minimizer_progress_to_stdout = true;
    ceres_solver_options.num_threads = 1;
    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
    ceres_solver_options.max_num_iterations = 100;
    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres_solver_options.parameter_tolerance = 1e-12;

    //===
    ceres_solver_options.minimizer_progress_to_stdout = true;
    ceres_solver_options.num_threads = 1;
    ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
    ceres_solver_options.max_num_iterations = 100;
    ceres_solver_options.gradient_tolerance = 1e-6;
    ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    ceres_solver_options.parameter_tolerance = 1e-9;


    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;


    problem.SetParameterBlockConstant(initial_x1);


    Solver::Summary summary;
    Solve(ceres_solver_options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "final    summary.initial_cost : " << summary.initial_cost << ",    summary.final_cost : "
              << summary.final_cost << std::endl;

    std::cout << "final initial_x : " << initial_x1[0] << ", " << initial_x1[1] << ", " << initial_x1[2] << std::endl;
    std::cout << "final initial_x2 : " << initial_x2[0] << ", " << initial_x2[1] << ", " << initial_x2[2] << std::endl;
    std::cout << "final initial_x3 : " << initial_x3[0] << ", " << initial_x3[1] << ", " << initial_x3[2] << std::endl;


}

void test_grid() {

    FloatType initial_x1[3] = {0.0, 0.0, 0.0};
    FloatType initial_x2[3] = {0.45, 0.1, 0.4};
    FloatType initial_x3[3] = {0.45, 0.1, 0.4};



    // Problem sizes
    const int kGridRowsHalf = 9;
    const int kGridColsHalf = 11;
    const int kGridRows = 2 * kGridRowsHalf + 1;
    const int kGridCols = 2 * kGridColsHalf + 1;
    const int kPoints = 4;

    const Eigen::Vector2d shift(0.234, 0.345);
    const std::array<Eigen::Vector2d, kPoints> points = {
            Eigen::Vector2d{-2., -3.},
            Eigen::Vector2d{-2., 3.},
            Eigen::Vector2d{2., 3.},
            Eigen::Vector2d{2., -3.}};

    // Data is a row-major array of kGridRows x kGridCols values of function
    // f(x, y) on the grid, with x in {-kGridColsHalf, ..., +kGridColsHalf},
    // and y in {-kGridRowsHalf, ..., +kGridRowsHalf}
    double data[kGridRows * kGridCols];
    for (int i = 0; i < kGridRows; ++i) {
        for (int j = 0; j < kGridCols; ++j) {
            // Using row-major order
            int index = i * kGridCols + j;
            double y = i - kGridRowsHalf;
            double x = j - kGridColsHalf;

            data[index] = f(x, y);
        }
    }
    const Grid grid(data,
                    -kGridRowsHalf,
                    kGridRowsHalf + 1,
                    -kGridColsHalf,
                    kGridColsHalf + 1);
    const Interpolator interpolator(grid);

    Eigen::Vector2d shift_estimate(3.1415, 1.337);

    ceres::Problem problem;
//    problem.AddParameterBlock(shift_estimate.data(), 2);


    for (const auto &p: points) {
        const Eigen::Vector2d shifted = p + shift;

        const double v = f(shifted.x(), shifted.y());
        CostFunction *cost_function = AutoDiffBiCubicCost::Create(interpolator, p, v);

        problem.AddResidualBlock(cost_function, nullptr, shift_estimate.data());
    }


    {
//        CostFunction* cost_function = PoseConstrains::create(0.5,0.1,0.0,0.5,0.5,0.9);
//        CostFunction* cost_function2 = PoseConstrains::create(0.1,0.3,0.2,0.5,0.5,0.9);
//
//        problem.AddResidualBlock(cost_function, nullptr, initial_x1,initial_x2);
//        problem.AddResidualBlock(cost_function2, nullptr, initial_x2,initial_x3);
    }

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << '\n';

    std::cout << "Bicubic interpolation with automatic derivatives:\n";
    std::cout << "Estimated shift: " << shift_estimate.transpose()
              << ", ground-truth: " << shift.transpose()
              << " (error: " << (shift_estimate - shift).transpose() << ")"
              << std::endl;

//    CHECK_LT((shift_estimate - shift).norm(), 1e-9);
}


struct PointsToGrid {
    enum {
        DATA_DIMENSION = 1
    };

    static const int GRID_SIZE = 100;
    std::array<std::array<double, GRID_SIZE + GRID_SIZE>,GRID_SIZE + GRID_SIZE> data ;

    double mean_x = 0.0;
    double mean_y = 0.0;
    double occupied_value = 0.0;
    double free_value = 1.0;
    double resolution_inv ;

    PointsToGrid(PointsToGrid && rhv){
        this->data =  rhv.data;
        this->mean_x = rhv.mean_x;
        this->mean_y = rhv.mean_y;
        this->occupied_value = rhv.occupied_value;
        this->free_value = rhv.free_value;
        this->resolution_inv = rhv.resolution_inv;
    }
    PointsToGrid(const PointsToGrid & rhv){
        this->data =  rhv.data;
        this->mean_x = rhv.mean_x;
        this->mean_y = rhv.mean_y;
        this->occupied_value = rhv.occupied_value;
        this->free_value = rhv.free_value;
        this->resolution_inv = rhv.resolution_inv;
    }

    PointsToGrid(const std::vector<float> &points, double resolution = 0.01, double filter_radius = 0.5 , double filter_step = 0.1,double smooth_radius = 3, double smooth_weight = 0.4) {

        resolution_inv = 1.0/resolution;

        int K = filter_radius * resolution_inv;


        free_value = (smooth_radius + smooth_weight * (K - smooth_radius)) * filter_step;

        /*LET'S INITIALIZE CELL VALUES TO= 0*/
        for (int i = 0; i < GRID_SIZE + GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE + GRID_SIZE; j++) {
                data[i][j]=free_value;
            }
        }

        // get mean x,y
        mean_x = 0.0, mean_y = 0.0;
        int N = 0.5 * points.size();
        if (N == 0) return;

        for (int i = 0; i < N; i++) {
            mean_x += points[i + i];
            mean_y += points[i + i + 1];

        }
        mean_x /= N;
        mean_y /= N;

        // points to index
        int row, col;
        int index[1000];

        for (int i = 0; i < N; i++) {
            row = int(std::round((points[i + i] - mean_x) * resolution_inv )   ) + GRID_SIZE;
            col = int(std::round((points[i + i + 1] - mean_y) * resolution_inv )  )  + GRID_SIZE;

            data[row][col] = occupied_value;
            data[row][col] = occupied_value;
#if 0
            data[row+1][col] = occupied_value;
            data[row-1][col] = occupied_value;
            data[row][col+1] = occupied_value;
            data[row][col-1] = occupied_value;
#endif

            index[i+i] = row;
            index[i+i + 1 ] = col;

        }


        int index_i, index_j;

        for (int i = 0; i < N; i++) {
            index_i = index[i+i];
            index_j = index[i+i + 1 ];

            int m1 = std::max(index_i-K, 0);
            int m2 = std::max(index_i+K, GRID_SIZE + GRID_SIZE-1);
            int n1 = std::max(index_j-K, 0);
            int n2 = std::max(index_j+K, GRID_SIZE + GRID_SIZE-1);
            for(int m = m1 ; m < m2; m++){
                for(int n = n1; n < n2; n++){
                    double dist =  std::sqrt((m-index_i)*(m-index_i) + (n -index_j)*(n -index_j));

                    dist = (dist > smooth_radius) ?  smooth_radius + smooth_weight * (dist - smooth_radius): dist;

                    data[m][n]  = std::min(dist* filter_step ,  data[m][n] ) ;

                }
            }

        }
    }

     void GetValue(const int r, const int c, double *f) const {
        const int row_idx =
                (std::min)((std::max)(0, r), GRID_SIZE + GRID_SIZE - 1);
        const int col_idx =
                (std::min)((std::max)(0, c), GRID_SIZE + GRID_SIZE - 1);
        f[0] = static_cast<double>(data[row_idx][col_idx]);
    }

    template<typename T>
     void GetIndex(const T &r, const T &c, T &r_index, T &c_index) const {
        r_index = ((r - mean_x) * resolution_inv  )+ T(GRID_SIZE);
        c_index = ((c - mean_y) * resolution_inv  )+ T(GRID_SIZE);
    }

};


// Cost-function using autodiff interface of BiCubicInterpolator
struct SimpleAutoDiffBiCubicCost {

    template<typename T>
    bool operator()(const T *P, T *residual) const {

        // transform points from laser frame to map frame

        T tx = P[0];
        T ty = P[1];

        T cos_t = cos(P[2]);
        T sin_t = sin(P[2]);

        T r00 = cos_t;
        T r01 = -sin_t;
        T r10 = sin_t;
        T r11 = cos_t;

        T dst_point[2];
        T dst_index[2];


        int N = 0.5*points_.size();
        T value = T(0.0);
        for(int i = 0; i < N;i++){
            dst_point[0] = r00 * points_[i+i] + r01 * points_[i+i+1] + tx;
            dst_point[1] = r10 * points_[i+i] + r11 * points_[i+i+1] + ty;



            grid_->template GetIndex(dst_point[0],dst_point[1],dst_index[0],dst_index[1]);
            interpolator_->Evaluate(dst_index[0],dst_index[1], &(residual[i] ));


//            std::cout << "points_   = " << points_[i+i] << ", " <<  points_[i+i+1] << std::endl;
//            std::cout << "dst_point = " <<dst_point[0]  << ", " <<  dst_point[1]  << std::endl;
//            std::cout << "dst_index = " <<dst_index[0]  << ", " <<  dst_index[1]  << std::endl;
//            std::cout << "residual[i] = " <<residual[i]   << std::endl;


            residual[i] = scaling_factor_ * residual[i];
        }


        return true;
    }

    SimpleAutoDiffBiCubicCost(PointsToGrid *grid ,ceres::BiCubicInterpolator<PointsToGrid>* interpolator , double scaling_factor,
                              const std::vector<float> &points )
            :   grid_(grid), interpolator_(interpolator) ,scaling_factor_(scaling_factor){

        simple_cast(points_, points);

    }

    static ceres::CostFunction *Create(PointsToGrid * grid ,ceres::BiCubicInterpolator<PointsToGrid>* interpolator , double scaling_factor,
                                       const std::vector<float> &points) {
        return new ceres::AutoDiffCostFunction<SimpleAutoDiffBiCubicCost, ceres::DYNAMIC, 3>(
                new SimpleAutoDiffBiCubicCost(grid,interpolator, scaling_factor, points),  static_cast<int>(0.5*points.size()));
    }

    std::vector<double> points_;
     PointsToGrid * grid_;
    ceres::BiCubicInterpolator<PointsToGrid>* interpolator_;
    double scaling_factor_ = 1.0;

};

// Cost-function using autodiff interface of BiCubicInterpolator
struct SimpleAutoDiffBiCubicCost_V2 {

    template<typename T>
    bool operator()(const T *P, T *residual) const {

        // transform points from laser frame to map frame

        T tx = P[0];
        T ty = P[1];

        T cos_t = cos(P[2]);
        T sin_t = sin(P[2]);

        T r00 = cos_t;
        T r01 = -sin_t;
        T r10 = sin_t;
        T r11 = cos_t;

        T dst_point[2];
        T dst_index[2];


        int N = 0.5*points_.size();
        T value = T(0.0);
        for(int i = 0; i < N;i++){
            dst_point[0] = r00 * points_[i+i] + r01 * points_[i+i+1] + tx;
            dst_point[1] = r10 * points_[i+i] + r11 * points_[i+i+1] + ty;



            grid_->template GetIndex(dst_point[0],dst_point[1],dst_index[0],dst_index[1]);
            interpolator_.Evaluate(dst_index[0],dst_index[1], &(residual[i] ));


//            std::cout << "points_   = " << points_[i+i] << ", " <<  points_[i+i+1] << std::endl;
//            std::cout << "dst_point = " <<dst_point[0]  << ", " <<  dst_point[1]  << std::endl;
//            std::cout << "dst_index = " <<dst_index[0]  << ", " <<  dst_index[1]  << std::endl;
//            std::cout << "residual[i] = " <<residual[i]   << std::endl;


            residual[i] = scaling_factor_ * residual[i];
        }


        return true;
    }

    SimpleAutoDiffBiCubicCost_V2(PointsToGrid *grid ,double scaling_factor,
                              const std::vector<float> &points )
            :   grid_(grid), interpolator_(*grid) ,scaling_factor_(scaling_factor){

        simple_cast(points_, points);

    }

    static ceres::CostFunction *Create(PointsToGrid * grid , double scaling_factor,
                                       const std::vector<float> &points) {
        return new ceres::AutoDiffCostFunction<SimpleAutoDiffBiCubicCost_V2, ceres::DYNAMIC, 3>(
                new SimpleAutoDiffBiCubicCost_V2(grid, scaling_factor, points),  static_cast<int>(0.5*points.size()));
    }

    std::vector<double> points_;
    PointsToGrid * grid_;
    const ceres::BiCubicInterpolator<PointsToGrid> interpolator_;
    double scaling_factor_ = 1.0;

};


template <typename Grid = PointsToGrid>
class SimpleBiCubicInterpolator{
public:
     Grid grid;
    const ceres::BiCubicInterpolator<Grid> interpolator;

    explicit SimpleBiCubicInterpolator(const std::vector<float>& points) : grid(points), interpolator(grid)  {
        // The + casts the enum into an int before doing the
        // comparison. It is needed to prevent
        // "-Wunnamed-type-template-args" related errors.
    }


};

struct SimpleAutoDiffBiCubicCost_V3 {

    template<typename T>
    bool operator()(const T *P, T *residual) const {

        // transform points from laser frame to map frame

        T tx = P[0];
        T ty = P[1];

        T cos_t = cos(P[2]);
        T sin_t = sin(P[2]);

        T r00 = cos_t;
        T r01 = -sin_t;
        T r10 = sin_t;
        T r11 = cos_t;

        T dst_point[2] = {T(0.0),T(0.0)};
        T dst_index[2]= {T(0.0),T(0.0)};


        int N = 0.5*points_.size();
        T value = T(0.0);
        for(int i = 0; i < N;i++){
            dst_point[0] = r00 * points_[i+i] + r01 * points_[i+i+1] + tx;
            dst_point[1] = r10 * points_[i+i] + r11 * points_[i+i+1] + ty;


            residual[i] = T(0.0);

//            std::cout << "points_   = " << points_[i+i] << ", " <<  points_[i+i+1] << std::endl;
//            std::cout << "dst_point = " <<dst_point[0]  << ", " <<  dst_point[1]  << std::endl;

            interpolator_.grid.GetIndex(dst_point[0],dst_point[1],dst_index[0],dst_index[1]); //ok
            interpolator_.interpolator.Evaluate(dst_index[0],dst_index[1], &(residual[i] ));


//            std::cout << "dst_index = " <<dst_index[0]  << ", " <<  dst_index[1]  << std::endl;
//            std::cout << "residual[i] = " <<residual[i]   << std::endl;


            residual[i] = scaling_factor_ * residual[i];
        }


        return true;
    }

    SimpleAutoDiffBiCubicCost_V3(const SimpleBiCubicInterpolator<PointsToGrid>& interpolator ,double scaling_factor,
                                 const std::vector<float> &points )
            :  interpolator_(interpolator) ,scaling_factor_(scaling_factor){

        simple_cast(points_, points);

    }

    static ceres::CostFunction *Create(const SimpleBiCubicInterpolator<PointsToGrid>& interpolator , double scaling_factor,
                                       const std::vector<float> &points) {
        return new ceres::AutoDiffCostFunction<SimpleAutoDiffBiCubicCost_V3, ceres::DYNAMIC, 3>(
                new SimpleAutoDiffBiCubicCost_V3( std::forward<const SimpleBiCubicInterpolator<PointsToGrid>&>(interpolator) , scaling_factor, points),
                static_cast<int>(0.5*points.size()));
    }

    std::vector<double> points_;
    const SimpleBiCubicInterpolator<PointsToGrid>& interpolator_;
    double scaling_factor_ = 1.0;

};

// An object that implements an infinite two dimensional grid needed
// by the BiCubicInterpolator where the source of the function values
// is an grid of type T on the grid
//
//   [(row_start,   col_start), ..., (row_start,   col_end - 1)]
//   [                          ...                            ]
//   [(row_end - 1, col_start), ..., (row_end - 1, col_end - 1)]
//
// Since the input grid is finite and the grid is infinite, values
// outside this interval needs to be computed. Grid2D uses the value
// from the nearest edge.
//
// The function being provided can be vector valued, in which case
// kDataDimension > 1. The data maybe stored in row or column major
// format and the various dimensional slices of the function maybe
// interleaved, or they maybe stacked, i.e, if the function has
// kDataDimension = 2, is stored in row-major format and if
// kInterleaved = true, then it is stored as
//
//   f001, f002, f011, f012, ...
//
// A commonly occuring example are color images (RGB) where the three
// channels are stored interleaved.
//
// If kInterleaved = false, then it is stored as
//
//  f001, f011, ..., fnm1, f002, f012, ...
template<typename T,
        int kDataDimension = 1,
        bool kRowMajor = true,
        bool kInterleaved = true>
struct SimpleGrid2D {
public:
    enum {
        DATA_DIMENSION = kDataDimension
    };

    SimpleGrid2D(const T *data,
                 const int row_begin,
                 const int row_end,
                 const int col_begin,
                 const int col_end)
            : data_(data),
              row_begin_(row_begin),
              row_end_(row_end),
              col_begin_(col_begin),
              col_end_(col_end),
              num_rows_(row_end - row_begin),
              num_cols_(col_end - col_begin),
              num_values_(num_rows_ * num_cols_) {
        CHECK_GE(kDataDimension, 1);
        CHECK_LT(row_begin, row_end);
        CHECK_LT(col_begin, col_end);
    }

    EIGEN_STRONG_INLINE void GetValue(const int r, const int c, double *f) const {
        const int row_idx =
                (std::min)((std::max)(row_begin_, r), row_end_ - 1) - row_begin_;
        const int col_idx =
                (std::min)((std::max)(col_begin_, c), col_end_ - 1) - col_begin_;

        const int n = (kRowMajor) ? num_cols_ * row_idx + col_idx
                                  : num_rows_ * col_idx + row_idx;

        if (kInterleaved) {
            for (int i = 0; i < kDataDimension; ++i) {
                f[i] = static_cast<double>(data_[kDataDimension * n + i]);
            }
        } else {
            for (int i = 0; i < kDataDimension; ++i) {
                f[i] = static_cast<double>(data_[i * num_values_ + n]);
            }
        }
    }

private:
    const T *data_;
    const int row_begin_;
    const int row_end_;
    const int col_begin_;
    const int col_end_;
    const int num_rows_;
    const int num_cols_;
    const int num_values_;
};

struct TestClass{

    std::vector<ceres::CostFunction*> cost_function_vec;


    void add(ceres::CostFunction* f){

        cost_function_vec.emplace_back(f);
    }

    ~TestClass(){

        const std::array<const double,3 > a{0.0,0.0,0.0};
        std::array<double,3 > b{0.0,0.0,0.0};

        for(auto& p: cost_function_vec){
            double r[3] = {0.0,0.0,0.0};


            delete p;
        }
        std::cout << "destroy tester" << std::endl;
    }

};

//todo: program crash when using vector, but ok with dequeue
// std::deque<SimpleBiCubicInterpolator<PointsToGrid>> II; //ok
// std::vector<SimpleBiCubicInterpolator<PointsToGrid>> II; // crash

//https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.cc
void test_interplator() {
#if 0
    {
        // clang-format off
        int x[] = {1, 2, 3,
                   2, 3, 4};
        // clang-format on
        ceres::Grid2D<int, 1, true, true> grid(x, 0, 2, 0, 3);
        for (int r = -1; r < 2; r ++) {
            for (int c = 0; c < 2; c ++) {
                double value;
                grid.GetValue(r, c, &value);
                std::cout << "r: " << r << ", c : " << c << ", value : " << value << std::endl;
            }
        }
    }
    {
        // clang-format off
        const double values[] = {1.0, 5.0, 2.0, 10.0, 2.0, 6.0, 3.0, 5.0,
                                 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 3.0, 1.0};
        // clang-format on

        SimpleGrid2D<double, 2> grid(values, 0, 2, 0, 4);
        ceres::BiCubicInterpolator<SimpleGrid2D<double, 2>> interpolator(grid);

        double f[2], dfdr[2], dfdc[2];
        const double r = 0.5;
        const double c = 2.5;
        interpolator.Evaluate(r, c, f, dfdr, dfdc);
        std::cout << "r: " << r << ", c : " << c << std::endl;
        std::cout << "f : " << f[0] << ", " << f[1] << std::endl
                  << "dfdr: " << dfdr[0] << ", " << dfdr[1] << std::endl
                  << "dfdc : " << dfdc[0] << ", " << dfdc[1]
                  << std::endl;

    }
#endif


    {

        std::cout << "==== BiCubicInterpolator\n" ;


        std::vector<float> points{0.1,0.1, 0.2,0.5};
//        - [6.930 , 0.878, 600.0]
//        - [8.261  , 0.101 , 600.0]
//        - [9.618  , 0.106 , 600.0]


        std::vector<float> points_2{0.1,0.1, 0.1,0.5};

        transform::Transform2d mt(0.1,0.2,0.4);
        mt.inverse().mul(points,points_2);

        PointsToGrid grid_(points);
        std::vector<PointsToGrid> V;

        std::deque<SimpleBiCubicInterpolator<PointsToGrid>> II; //ok
//        std::vector<SimpleBiCubicInterpolator<PointsToGrid>> II; // crash


        int N= 2;
        std::vector<float> tmp_points{0.1,0.1, 0.2,0.5};

        PointsToGrid grid(tmp_points);



        for(int i = 0; i < N;i++){
            std::vector<float> tmp_points{0.1,0.1, 0.2,0.5};
            II.emplace_back(tmp_points);
        }

        for(int i = 0; i < N;i++){

            bool ok = true;

            for(int m = 0 ; m < grid.data.size(); m++){
                for(int n = 0 ; n < grid.data[n].size();n++){
                    if(II[i].grid.data[m][n] != grid.data[m][n]){
                        ok = false;
                        std::cout << "== 1 grid ok = " << ok  << ", m = " << m << ", n = " << n << std::endl;

                        break;
                    }

                }
            }
            std::cout << "== 1 grid ok = " << ok << std::endl;
            ok= II[i].grid.data == grid.data;
            std::cout << "== 2 grid ok = " << ok << std::endl;

        }


        ceres::Problem problem;
        double init_x[3]= {0.05,0.15,0.30};

        TestClass Tester;


        for(int z = 0; z < N;z++){
            ceres::CostFunction * cost_function = SimpleAutoDiffBiCubicCost_V3::Create( II[z] ,1.0,points_2); //crash

            Tester.add(cost_function);
        }

        std::cout << "done tester: " << std::endl;



        for(int z = 0; z < N;z++){


//            ceres::CostFunction* cost_function =   SimpleAutoDiffBiCubicCost::Create(&V[z], &II[z],1.0, points_2);
//            ceres::CostFunction* cost_function =   SimpleAutoDiffBiCubicCost::Create(&V[z], &interpolator,1.0, points_2); //ok

//            ceres::CostFunction* cost_function =   SimpleAutoDiffBiCubicCost::Create(&V[z], &interpolator,1.0, points_2); //ok

//            ceres::CostFunction* cost_function =   SimpleAutoDiffBiCubicCost_V2::Create(&V[z],1.0, points_2);//ok

            ceres::CostFunction * cost_function = SimpleAutoDiffBiCubicCost_V3::Create( II[z] ,1.0,points_2); //crash

            problem.AddResidualBlock(cost_function, nullptr, init_x);


        }

        ceres::Solver::Options ceres_solver_options;
//
        ceres_solver_options.minimizer_progress_to_stdout = true;
        ceres_solver_options.num_threads = 1;
        ceres_solver_options.function_tolerance = 1e-3;  // Enough for denoising.
        ceres_solver_options.max_num_iterations = 100;
        ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        ceres_solver_options.parameter_tolerance = 1e-12;

        //===
        ceres_solver_options.minimizer_progress_to_stdout = false;
        ceres_solver_options.num_threads = 1;
        ceres_solver_options.function_tolerance = 1e-4;  // Enough for denoising.
        ceres_solver_options.max_num_iterations = 200;
        ceres_solver_options.gradient_tolerance = 1e-4;
        ceres_solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        ceres_solver_options.parameter_tolerance = 1e-4;

//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
//        ceres_solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; // crash
        ceres_solver_options.linear_solver_type = ceres::DENSE_QR;

        ceres_solver_options.linear_solver_type = ceres::DENSE_SCHUR; //ok

        std::cout << "=== start Solver" << std::endl;

        ceres::Solver::Summary summary;
        ceres::Solve(ceres_solver_options, &problem, &summary);
        std::cout << "=== end Solver" << std::endl;

        std::cout << summary.BriefReport() << '\n';
        std::cout << "init_x : " << init_x[0] << ", " << init_x[1] << ", " << init_x[2] << std::endl;
        std::cout << "=== finish" << std::endl;


    }
}

int main(int argc, char **argv) {


    //test_ceres();

//    test_ceres_pose_graph();
    test_interplator();

    return 0;
}