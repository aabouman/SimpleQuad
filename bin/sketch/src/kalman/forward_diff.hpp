#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/imu_vicon_feather/src/kalman/forward_diff.hpp"
#include <ArduinoEigenDense.h>
#include <ArduinoEigen/unsupported/Eigen/AutoDiff>

#include "eigen_utils.h"

// EKF::process(Ref<State> curr_state, Ref<Input> curr_input, float dt)

using namespace Eigen;

template <typename Scalar, int NX = Dynamic, int NY = Dynamic>
struct TestFunc1
{
    enum
    {
        InputsAtCompileTime = NX,
        OutputsAtCompileTime = NY
    };
    typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Matrix<Scalar, OutputsAtCompileTime, 1> ValueType;
    typedef Matrix<Scalar, OutputsAtCompileTime, InputsAtCompileTime> JacobianType;

    template <typename T>
    void operator()(const Matrix<T, InputsAtCompileTime, 1> &x,
                    Matrix<T, OutputsAtCompileTime, 1> *_v) const
    {
        Matrix<T, OutputsAtCompileTime, 1> &v = *_v;
        v[0] = x[0];
        v[1] = x[1];
    }
};

template <typename Func>
void forward_jacobian(const Func &f)
{
    typename Func::InputType x = Func::InputType::Random(2);
    typename Func::ValueType y(2);
    typename Func::JacobianType j(2, 2);

    j.setZero();
    y.setZero();
    AutoDiffJacobian<Func> auto_jac(f);
    auto_jac(x, &y, &j);

    // Serial.printf("|%f, %f|\n", j(0, 0), j(0, 1));
    // Serial.printf("|%f, %f|", j(1, 0), j(1, 1));
    // Serial.println();

    print_mtxf(j);
}


void test_autodiff_jacobian()
{
    forward_jacobian(TestFunc1<float, 2, 2>());
}
