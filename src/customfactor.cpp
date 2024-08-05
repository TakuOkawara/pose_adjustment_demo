#include "pose_adjustment/customfactor.hpp"
// *** NOTATION *** ///
// pp:    Previous pose
// cp:    Current pose
// dp:    Delta pose
// E :    Error
// H_A_B: Jacobian matrix d(A) / d(B)

CustomBetweenFactor::CustomBetweenFactor(
    gtsam::Key key1,                        // key of previous pose (optimization value)
    gtsam::Key key2,                        // key of current pose (optimization value)
    const gtsam::Pose3& measured,           // measurement relative pose
    const gtsam::SharedNoiseModel& model)   // noise model of measurement relative pose (i.e., Covariance matrix, Information matrix)
:gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1, key2), measured_(measured) {}

// You must calculate the error (Return value) and the jacobian matrix (H1 and H2 that is call by reference) 
gtsam::Vector CustomBetweenFactor::evaluateError(
    const gtsam::Pose3 &pp, const gtsam::Pose3 &cp, 
    boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2) const {
        
        // d(T_pp_cp) / d(previous_pose)
        gtsam::Matrix66 H_dp_pp;
        // d(T_pp_cp) / d(current_pose)
        gtsam::Matrix66 H_dp_cp;
        
        // delta pose between previous pose and current pose
        gtsam::Pose3 dp = pp.between(cp, H_dp_pp, H_dp_cp);

        // Calculate error

        // d(Error) / d(dp)
        gtsam::Matrix66 H_E_dp;
        gtsam::Pose3 Error = dp.between(measured_, H_E_dp);

        // d(log_Error) / d(Error)
        gtsam::Matrix6 H_logE_E;

        // Error with logmap
        gtsam::Vector6 log_Error = gtsam::Pose3::Logmap(Error, H_logE_E);

        // Calculate the jacobian matrix for the previous pose
        if(H1) {
            // d(log Error) / d(previous pose)
            *H1 = H_logE_E * H_E_dp * H_dp_pp;
        }
        // Calculate the jacobian matrix for the current pose
        if(H2) {
            // d(log Error) / d(current pose)
            *H2 = H_logE_E * H_E_dp * H_dp_cp;
        }
        
        return log_Error;
}
