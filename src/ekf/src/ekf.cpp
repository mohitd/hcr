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

#include "ekf/ekf.h"
#include "ekf/utils.h"

#include <Eigen/Core>
#include <Eigen/LU>

namespace ekf {

void Predict(Ekf& ekf, double dt) {
    auto new_state = ekf.state;
    new_state(0) = ekf.state(3) * dt * std::cos(ekf.state(2)) + ekf.state(0);
    new_state(1) = ekf.state(3) * dt * std::sin(ekf.state(2)) + ekf.state(1);
    new_state(2) = wrapAngle(ekf.state(4) * dt + ekf.state(2));

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(5, 5);
    // x
    jacobian(0,0) = 1;
    jacobian(0,2) = -ekf.state(3) * dt * std::sin(ekf.state(2));
    jacobian(0,3) = dt * std::cos(ekf.state(2));

    // y
    jacobian(1,1) = 1;
    jacobian(1,2) = ekf.state(3) * dt * std::cos(ekf.state(2));
    jacobian(1,3) = dt * std::sin(ekf.state(2));

    // theta
    jacobian(2,2) = 1;
    jacobian(2,4) = dt;

    // v
    jacobian(3,3) = 1;

    // w
    jacobian(4,4) = 1;

    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(5, 5);
    process_noise = process_noise * 0.1;

    ekf.state = new_state;
    ekf.covariance = jacobian * ekf.covariance * jacobian.transpose() + process_noise;
}

bool Update(Ekf& ekf,
        const Eigen::VectorXd& z,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R) {
    Eigen::VectorXd y = z - H * ekf.state;
    Eigen::MatrixXd S = H * ekf.covariance * H.transpose() + R;

    Eigen::FullPivLU<Eigen::MatrixXd> lu(S);
    if (!lu.isInvertible()) {
        return false;
    }

    Eigen::MatrixXd K = ekf.covariance * H.transpose() * S.inverse();
    ekf.state = ekf.state + K * y;
    Eigen::MatrixXd KH = K * H;
    ekf.covariance = (Eigen::MatrixXd::Identity(KH.rows(), KH.cols()) - KH) * ekf.covariance;
    return true;
}
}

