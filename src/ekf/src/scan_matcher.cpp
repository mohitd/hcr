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

#include "ekf/scan_matcher.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

namespace ekf {

Eigen::Matrix4d ScanMatch(
        const ScanMatcherParams& params,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& prev_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& current_cloud) {

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    if (params.nonlinear_icp) {
        icp = pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>();
    }

    icp.setRANSACIterations(500);
    icp.setMaximumIterations(params.max_iterations);
    icp.setTransformationEpsilon(1e-9);
    icp.setMaxCorrespondenceDistance(0.10);
    icp.setEuclideanFitnessEpsilon(0.50);
    icp.setRANSACOutlierRejectionThreshold(0.05);

    // find the forward transform from the previous frame to the current one
    icp.setInputTarget(current_cloud);
    icp.setInputSource(prev_cloud);

    pcl::PointCloud<pcl::PointXYZ> output;

    icp.align(output);
    return icp.getFinalTransformation().cast<double>();
}

}

