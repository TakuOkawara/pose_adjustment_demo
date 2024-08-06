#ifndef DEMO_IRIDESCENCE_HPP
#define DEMO_IRIDESCENCE_HPP

#include <iostream>
#include <stdio.h>
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
#include <glk/lines.hpp>
#include <glk/thin_lines.hpp>
#include <glk/colormap.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/hovered_drawings.hpp>
#include <guik/viewer/light_viewer.hpp>
#include "pose_adjustment/customfactor.hpp"
using namespace gtsam;
using namespace std;


void draw_3d_trajectory_and_all_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id = 0);

void draw_2d_trajectory_and_all_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id = 0);

void draw_3d_trajectory_and_only_3_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id = 0, const bool is_custom_factor = false);

void draw_2d_trajectory_and_only_3_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id = 0);

#endif // DEMO_IRIDESCENCE_HPP