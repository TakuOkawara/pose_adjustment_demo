
/**
 * @file Pose2SLAMExample.cpp
 * @brief A 2D Pose SLAM example based on https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExample.cpp
 */

/**
 * A simple 2D pose slam example
 *  - The robot moves in a 2 meter square
 *  - The robot moves 2 meters each step, turning 90 degrees after each step
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have a loop constraint when the robot returns to the first position
 */

// In planar SLAM example we use Pose2 variables (x, y, theta) to represent the robot poses
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// We will also use a Between Factor to encode the loop closure constraint
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/BetweenFactor.h>

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// a Gauss-Newton solver
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>


using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  auto priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 0.1));
  graph.addPrior(1, Pose2(0, 0, 0), priorNoise);

  // For simplicity, we will use the same noise model for odometry and loop closures
  auto noise_model = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

  // 2b. Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2, 0, 0), noise_model);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2, 0, M_PI_2), noise_model);
  graph.emplace_shared<BetweenFactor<Pose2> >(3, 4, Pose2(2, 0, M_PI_2), noise_model);
  graph.emplace_shared<BetweenFactor<Pose2> >(4, 5, Pose2(2, 0, M_PI_2), noise_model);

  // 2c. Add the loop closure constraint
  // This factor encodes the fact that we have returned to the same pose. In real systems,
  // these constraints may be identified in many ways, such as appearance-based techniques
  // with camera images. We will use another Between Factor to enforce this constraint:
  graph.emplace_shared<BetweenFactor<Pose2> >(5, 2, Pose2(2, 0, M_PI_2), noise_model);
  graph.print("\nFactor Graph:\n");  // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, M_PI_2));
  initialEstimate.insert(4, Pose2(4.0, 2.0, M_PI));
  initialEstimate.insert(5, Pose2(2.1, 2.1, -M_PI_2));
  initialEstimate.print("\n***** Initial Estimate *****\n");  // print

  // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer
  // Create the optimizer ...
  GaussNewtonOptimizer optimizer(graph, initialEstimate);
  // ... and optimize
  Values result = optimizer.optimize();
  result.print("***** Final Result *****\n");

  return 0;
}
