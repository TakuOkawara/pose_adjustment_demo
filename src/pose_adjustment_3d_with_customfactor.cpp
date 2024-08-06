#include "pose_adjustment/pose_adjustment.hpp"
#include "pose_adjustment/demo_iridescence.hpp"
#include "pose_adjustment/customfactor.hpp"

void readG2OFile(const std::string& filename, NonlinearFactorGraph& graph, Values& initialEstimate){
    // Open the .g2o file
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

        if (tag == "VERTEX_SE3:QUAT") {
            // Parsing a vertex line
            size_t id;
            double x, y, z, qx, qy, qz, qw;
            ss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;

            // Create a Pose3 object
            Rot3 rotation = Rot3::Quaternion(qw, qx, qy, qz);
            Point3 translation(x, y, z);
            Pose3 pose(rotation, translation);

            // Insert into initial estimates
            initialEstimate.insert(Symbol('x', id), pose);
        } else if (tag == "EDGE_SE3:QUAT") {
            // Parsing an edge line
            size_t id1, id2;
            double x, y, z, qx, qy, qz, qw;
            double info[21];
            ss >> id1 >> id2 >> x >> y >> z >> qx >> qy >> qz >> qw;

            // Read information matrix (upper triangular part)
            for (int i = 0; i < 21; ++i) {
                ss >> info[i];
            }

            // Create a Pose3 object for the transformation
            Rot3 rotation = Rot3::Quaternion(qw, qx, qy, qz);
            Point3 translation(x, y, z);
            Pose3 pose(rotation, translation);

            // Convert the information matrix to Eigen format
            Matrix6 information;
            information << info[15], info[16], info[17], info[3], info[8], info[12],
                           info[16], info[18], info[19], info[4], info[9], info[13],
                           info[17], info[19], info[20], info[5], info[10], info[14],
                           info[3], info[4], info[5], info[0], info[1], info[2],
                           info[8], info[9], info[10], info[1], info[6], info[7],
                           info[12], info[13], info[14], info[2], info[7], info[11];

            // Create noise model from information matrix
            SharedNoiseModel model = noiseModel::Gaussian::Information(information);

            // Create and add BetweenFactor to the graph
            graph.emplace_shared<CustomBetweenFactor>(Symbol('x', id1), Symbol('x', id2), pose, model);
        }
    }

    infile.close();
}


int main(int argc, char** argv) {

    // Read graph and values from the file
    NonlinearFactorGraph graph;
    Values initial;
    std::string filename = "../Data/sphere2500.g2o";

    // Read g2o file that a trajectory and relative poses (i.e., odometry constraint and loop constraint) defined.
    // Then, initial poses (initial) and a pose graph (graph) are defined by the file.
    readG2OFile(filename, graph, initial);

    // iridescence viewer
    guik::LightViewer* raw_ptr = guik::LightViewer::instance();
    std::shared_ptr<guik::LightViewer> viewer(raw_ptr);
    // Draw lines for initial poses
    draw_3d_trajectory_and_only_3_constraints(graph, initial, viewer, 0);
    std::cout << "Drawing the initial line" << std::endl;

    // Add prior on the first pose

    if (!initial.empty()) {
        auto priorModel = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        Key firstkey = initial.keys().front();
        std::cout << "Adding prior to the first key: " << firstkey << std::endl;
        graph.addPrior(firstkey, Pose3(), priorModel);
    } else {
        std::cerr << "Initial estimate is empty, cannot add prior." << std::endl;
        return -1;
    }

    // Optimize the graph
    std::cout<< "Optimizing the factor graph" << std::endl;
    GaussNewtonOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();
    std::cout << "Optimization complete" << std::endl;

    // draw the optimized line
    draw_3d_trajectory_and_only_3_constraints(graph, result, viewer, 1);

    std::cout << "initial error=" << graph.error(initial) << std::endl;
    std::cout << "final error=" << graph.error(result) << std::endl;

    // result.print("result");

    //Register a callback for UI rendering
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

