/*
 * Copyright (c) 2020. Mohit Deshpande.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ekf/utils.h"

namespace ekf {

double wrapAngle(double angle) {
    geometry_msgs::PoseWithCovariance x;
    static constexpr double TWO_PI = 2.0 * 3.141592865358979;
    return angle - TWO_PI * std::floor(angle / TWO_PI);
}

boost::array<double, 36> toRos(const Eigen::MatrixXd& covariance) {
    boost::array<double, 36> ros_cov{};
    ros_cov.assign(0);

    // TODO: replace this with some f : (r,c)->idx polynomial
    // x
    ros_cov[0] = covariance(0, 0);  // xx
    ros_cov[1] = covariance(0, 1);  // xy
    ros_cov[5] = covariance(0, 2);  // xy

    // y
    ros_cov[6] = covariance(1, 0);  // yx
    ros_cov[7] = covariance(1, 1);  // yy
    ros_cov[11] = covariance(1, 2);  // yy

    // yaw
    ros_cov[30] = covariance(2, 0);  // yx
    ros_cov[31] = covariance(2, 1);  // yy
    ros_cov[35] = covariance(2, 2);  // yy
    return ros_cov;
}

}
