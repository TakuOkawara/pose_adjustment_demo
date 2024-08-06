#include "pose_adjustment/pose_adjustment.hpp"
#include "pose_adjustment/demo_iridescence.hpp"

void readG2OFile(const std::string& filename, NonlinearFactorGraph& graph, Values& initialEstimate) {
    // Opne the file
    ifstream infile(filename);
    if (!infile) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }

    string line;
    while (getline(infile, line)) {
        stringstream ss(line);
        string tag;
        ss >> tag;

        if (tag == "VERTEX_SE2") {
            // Parsing a vertex line
            size_t id;
            double x, y, theta;
            ss >> id >> x >> y >> theta;

            // Create a Pose2 object
            Pose2 pose(x, y, theta);

            // Insert into initial estimates
            initialEstimate.insert(Symbol('x', id), pose);
        } else if (tag == "EDGE_SE2") {
            // Parsing an edge line
            size_t id1, id2;
            double x, y, theta;
            double info[6];
            ss >> id1 >> id2 >> x >> y >> theta;

            // Read information matrix
            for (int i = 0; i < 6; ++i) {
                ss >> info[i];
            }

            // Create a Pose2 object for the transformation
            Pose2 pose(x, y, theta);

            // Convert the information matrix to Eigen format
            Matrix3 information;
            information << info[0], info[1], info[2],
                           info[1], info[3], info[4],
                           info[2], info[4], info[5];

            // Create noise model from information matrix
            SharedNoiseModel model = noiseModel::Gaussian::Information(information);

            // Insert into the factor graph
            graph.emplace_shared<BetweenFactor<Pose2>>(Symbol('x', id1), Symbol('x', id2), pose, model);
        }
    }

    infile.close();
}

int main(int argc, char** argv) {

    // Read graph and values from the file
    NonlinearFactorGraph graph;
    Values initial;
    string filename = "../Data/manhattanOlson3500.g2o";

    // Read g2o file that a trajectory and relative poses (i.e., odometry constraint and loop constraint) defined.
    // Then, initial poses (initial) and a pose graph (graph) are defined by the file.
    readG2OFile(filename, graph, initial);

    // iridescence viewer
    guik::LightViewer* raw_ptr = guik::LightViewer::instance();
    std::shared_ptr<guik::LightViewer> viewer(raw_ptr);
    // Draw lines for initial poses
    draw_2d_trajectory_and_only_3_constraints(graph, initial, viewer, 0);

    if (!initial.empty()) {
    // add prior on the first pose
      auto priorModel = noiseModel::Diagonal::Variances(Vector3(1e-6, 1e-6, 1e-8));
      Key firstkey = initial.keys().front();
      graph.addPrior(firstkey, Pose2(), priorModel);
      std::cout << "Adding prior to pose 0" << std::endl;
    } else {
    std::cerr << "Initial estimate is empty, cannot add prior." << std::endl;
    return -1;
    }

    // Optimize the graph
    std::cout << "Optimizing the factor graph" << std::endl;
    GaussNewtonOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();
    std::cout << "Optimization complete" << std::endl;

    // Draw the optimized line
    draw_2d_trajectory_and_only_3_constraints(graph, result, viewer, 1);

    std::cout << "initial error=" << graph.error(initial) << std::endl;
    std::cout << "final error=" << graph.error(result) << std::endl;
   // result.print("result");
   
    // Register a callback for UI rendering
    viewer->register_ui_callback("ui", [&]() {
    // In the callback, you can call ImGui commands to create your UI.
    // Here, we use "DragFloat" and "Button" to create a simple UI.
      if (ImGui::Button("Close")) {
        viewer->close();
      }
    });
    viewer->spin();

    return 0;
}
