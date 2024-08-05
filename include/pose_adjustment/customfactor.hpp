#ifndef CUSTOMFACTOR_HPP
#define CUSTOMFACTOR_HPP

#include <iostream>
#include <memory>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <cmath>
#include <gtsam/slam/dataset.h>

using namespace gtsam;
using namespace std;

// This is a custom between factor which is implemented by inheritancing gtsam::NoiseModelFactor2
class CustomBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
        private:
            gtsam::Pose3 measured_; // measurement relative pose

        public:
            CustomBetweenFactor(
                gtsam::Key key1,                           // key of previous pose (optimization value)
                gtsam::Key key2,                           // key of current pose (optimization value)
                const gtsam::Pose3& measured,              // measurement relative pose
                const gtsam::SharedNoiseModel& model);     // noise model of measurement relative pose (i.e., Covariance matrix, Information matrix)

            virtual ~CustomBetweenFactor() {}
            // You must calculate the error (Return value) and the jacobian matrix (H1 and H2 that is call by reference) 
            gtsam::Vector evaluateError(
            const gtsam::Pose3 &pp, const gtsam::Pose3 &cp, 
            boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none) const override;
};
#endif // CUSTOMFACTOR_HPP
