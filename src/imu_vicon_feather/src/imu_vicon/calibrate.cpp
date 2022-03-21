// #include "imu_vicon.hpp"

// // Make sure everything is statically alloced
// #define EIGEN_NO_MALLOC
// #include <ArduinoEigenDense.h>
// #include <ArduinoEigen/Eigen/Geometry>

// using namespace Eigen;

// // Default to Identity quaternion
// Quaternionf offset_quat(1, 0, 0, 0);

// void calibrate_imu(imu_vicon_t *data)
// {
//     updateIMU(data);

//     float ave_acc_x = data->acc_x;
//     float ave_acc_y = data->acc_y;
//     float ave_acc_z = data->acc_z;

//     for (int i = 0; i < 1000; i++)
//     {
//         // Fetch new data
//         updateIMU(data);
//         // Rolling average
//         ave_acc_x = (ave_acc_x + data->acc_x) / 2;
//         ave_acc_y = (ave_acc_y + data->acc_y) / 2;
//         ave_acc_z = (ave_acc_z + data->acc_z) / 2;
//     }

//     Vector3f true_grav(0, 0, -9.81);
//     Vector3f meas_grav(ave_acc_x, ave_acc_y, ave_acc_z);

//     offset_quat = Quaternionf::FromTwoVectors(meas_grav, true_grav);
// }