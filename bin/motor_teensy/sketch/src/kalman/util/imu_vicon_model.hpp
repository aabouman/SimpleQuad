#line 1 "/Users/AlexanderBouman/Desktop/GradSchool/RExLab/SimpleQuad/src/motor_teensy/src/kalman/util/imu_vicon_model.hpp"
#ifndef _IMU_VICON_MODEL_HPP
#define _IMU_VICON_MODEL_HPP

// Make sure everything is statically alloced
#define EIGEN_NO_MALLOC

#include <ArduinoEigenDense.h>
#include <ArduinoEigen/unsupported/Eigen/AutoDiff>

// #include "eigen_utils.h"
#include "quaternion_diff.hpp"
#include "imu_vicon_types.hpp"

using namespace Eigen;

namespace Filter
{
    /********************************************************************
     *                    Process/Dynamics functions                    *
     ********************************************************************/
    template <typename T>
    static state_t<T> dynamics(const state_t<T> &curr_state,
                               const input_t<float> &curr_input)
    {
        // Extracting components of the state
        // Vector3f pos = curr_state.segment(0, 3);
        Quaternion<T> quat(curr_state(3), curr_state(4), curr_state(5), curr_state(6));
        Vector3<T> vel = curr_state.segment(7, 3);
        Vector3<T> alpha = curr_state.segment(10, 3);
        Vector3<T> beta = curr_state.segment(13, 3);

        // Extracting components of the input
        Vector3<T> vel_dot = curr_input.segment(0, 3);
        Vector3<T> omega = curr_input.segment(3, 3);

        // Derivative of position
        state_t<T> ret;
        ret.segment(0, 3) = quat * vel;

        // Derivative of quaternion
        Quaternion<T> omegaQuat(0.0, omega(0) - beta(0), omega(1) - beta(1), omega(2) - beta(2));
        Quaternion<T> kin = quat * omegaQuat;
        Vector4<T> kin_vec(kin.w(), kin.x(), kin.y(), kin.z());
        ret.segment(3, 4) = 1 / 2 * kin_vec;

        // Derivative of linear velocity
        Vector3<T> gravity(0, 0, 9.81);
        ret.segment(7, 3) = vel_dot - alpha - (quat.inverse() * gravity);
        // Derivative of imu biases
        ret.segment(10, 6).setZero();

        return ret;
    }

    template <typename T>
    state_t<T> process(const state_t<T> &curr_state,
                            const input_t<float> &curr_input,
                            float dt)
    {
        // RK4 Integrator
        state_t<T> tmp;
        state_t<T> k1 = dynamics(curr_state, curr_input);
        tmp = curr_state + 0.5 * dt * k1;
        state_t<T> k2 = dynamics(tmp, curr_input);
        tmp = curr_state + 0.5 * dt * k2;
        state_t<T> k3 = dynamics(tmp, curr_input);
        tmp = curr_state + dt * k3;
        state_t<T> k4 = dynamics(tmp, curr_input);
        // Set the state to the RK4 integrator output
        state_t<T> next_state = curr_state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
        // Normalize Quaternion
        next_state.segment(3, 4).normalize();
        return next_state;
    }

    struct ProcessFunc
    {
        enum
        {
            InputsAtCompileTime = EKF_NUM_STATES,
            ValuesAtCompileTime = EKF_NUM_STATES
        };
        typedef Matrix<float, InputsAtCompileTime, 1> InputType;
        typedef Matrix<float, ValuesAtCompileTime, 1> ValueType;
        typedef Matrix<float, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    private:
        input_t<float> _u;
        float _dt;

    public:
        ProcessFunc(input_t<float> u, float dt)
        {
            _u = u;
            _dt = dt;
        }

        template <typename T>
        void operator()(const Matrix<T, InputsAtCompileTime, 1> &ins,
                        Matrix<T, ValuesAtCompileTime, 1> *outputs) const
        {
            Matrix<T, ValuesAtCompileTime, 1> &v = *outputs;
            state_t<T> xs = ins;
            v = process(xs, _u, _dt);
        }
    };

    process_jac_t<float> process_jacobian(const state_t<float> &curr_state,
                                                const input_t<float> &curr_input,
                                                float dt)
    {
        ProcessFunc proc(curr_input, dt);
        state_t<float> tmp_state = state_t<float>::Zero();
        process_jac_t<float> jac = process_jac_t<float>::Zero();

        AutoDiffJacobian<ProcessFunc> auto_jac(proc);
        auto_jac(curr_state, &tmp_state, &jac);

        return jac;
    }

    /********************************************************************
     *                         Measure functions                        *
     ********************************************************************/
    template <typename T>
    measurement_t<T> measure(const state_t<T> &curr_state)
    {
        measurement_t<T> measurement1(curr_state.segment(0, 7));
        return measurement1;
    }

    struct MeasureFunc
    {
        enum
        {
            InputsAtCompileTime = EKF_NUM_STATES,
            ValuesAtCompileTime = EKF_NUM_MEASURES
        };
        typedef Matrix<float, InputsAtCompileTime, 1> InputType;
        typedef Matrix<float, ValuesAtCompileTime, 1> ValueType;
        typedef Matrix<float, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    public:
        MeasureFunc(){}

        template <typename T>
        void operator()(const Matrix<T, InputsAtCompileTime, 1> &ins,
                        Matrix<T, ValuesAtCompileTime, 1> *outputs) const
        {
            Matrix<T, ValuesAtCompileTime, 1> &v = *outputs;
            state_t<T> xs = ins;
            v = measure(xs);
        }
    };

    measure_jac_t<float> measure_jacobian(const state_t<float> &curr_state)
    {
        const int Nx = EKF_NUM_STATES;
        const int Ny = EKF_NUM_MEASURES;

        MeasureFunc meas = MeasureFunc();
        MeasureFunc::InputType x = curr_state;
        MeasureFunc::ValueType y = MeasureFunc::ValueType::Zero(Ny);
        MeasureFunc::JacobianType jac = MeasureFunc::JacobianType::Zero(Ny, Nx);

        AutoDiffJacobian<MeasureFunc> auto_jac(meas);
        auto_jac(x, &y, &jac);

        return jac;
    }

    /********************************************************************
     *                 State/Measure Error Functions                    *
     ********************************************************************/
    // template <typename T>
    state_t<float> state_composition(state_t<float> state, err_state_t<float> state_diff)
    {
        Vector3<float> pos = state.segment(0, 3);
        Quaternion<float> quat(state(3), state(4), state(5), state(6));
        Vector3<float> vel = state.segment(7, 3);
        Vector3<float> alpha = state.segment(10, 3);
        Vector3<float> beta = state.segment(13, 3);

        Vector3<float> d_pos = state_diff.segment(0, 3);
        Vector3<float> d_quat = state_diff.segment(3, 3);
        Vector3<float> d_vel = state_diff.segment(6, 3);
        Vector3<float> d_alpha = state_diff.segment(9, 3);
        Vector3<float> d_beta = state_diff.segment(12, 3);

        Vector3<float> pos2 = pos + d_pos;
        Quaternion<float> quat2 = add_rotation_error(quat, d_quat);
        Vector3<float> vel2 = vel + d_vel;
        Vector3<float> alpha2 = alpha + d_alpha;
        Vector3<float> beta2 = beta + d_beta;

        state_t<float> ret_state;
        ret_state.segment(0, 3) = pos2;
        ret_state(3) = quat2.w();
        ret_state(4) = quat2.x();
        ret_state(5) = quat2.y();
        ret_state(6) = quat2.z();
        ret_state.segment(7, 3) = vel2;
        ret_state.segment(10, 3) = alpha2;
        ret_state.segment(13, 3) = beta2;
        return ret_state;
    }

    // Compute the error state between measurements
    err_measurement_t<float> measurement_error(measurement_t<float> m2, measurement_t<float> m1)
    {
        Vector3<float> pos1 = m1.segment(0, 3);
        Quaternion<float> quat1(m1(3), m1(4), m1(5), m1(6));

        Vector3<float> pos2 = m2.segment(0, 3);
        Quaternion<float> quat2(m2(3), m2(4), m2(5), m2(6));

        err_measurement_t<float> ret_measure;
        ret_measure.segment(0, 3) = pos2 - pos1;
        ret_measure.segment(3, 3) = rotation_error(quat2, quat1);

        return ret_measure;
    }

    // Matrix to map states into error states
    err_state_state_jac_t<float> err_state_state_jacobian(Quaternion<float> quat)
    {
        Matrix<float, EKF_NUM_STATES, EKF_NUM_ERR_STATES> J;
        J.setIdentity();
        J.block(3, 3, 4, 3) = differential_quat(quat);
        return J.transpose();
    }

    // Matrix to map states into error states
    err_measure_measure_jac_t<float> err_measure_measure_jacobian(Quaternion<float> quat)
    {
        Matrix<float, EKF_NUM_MEASURES, EKF_NUM_ERR_MEASURES> G;
        G.setIdentity();
        G.block(3, 3, 4, 3) = differential_quat(quat);
        return G.transpose();
    }

}
#endif