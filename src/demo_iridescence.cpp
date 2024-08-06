#include "pose_adjustment/demo_iridescence.hpp"


void draw_3d_trajectory_and_all_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id) {
    std::vector<Eigen::Vector3f> graph_vertices;
    std::vector<Eigen::Vector3f> value_vertices;

    // Extract the vertices from the NonlinearFactorGraph
    for (const auto& factor : graph) {
        for (const auto& key : factor->keys()) {
            if (input_values.exists<gtsam::Pose3>(key)) {
                gtsam::Pose3 pose = input_values.at<gtsam::Pose3>(key);
                Eigen::Vector3f trans(pose.x(), pose.y(), pose.z());
                graph_vertices.push_back(trans);
            }
        }
    }

    // Extract the vertices from the gtsam::Values object
    for (const auto& key_value_pair : input_values) {
        auto key = key_value_pair.key;
        if (input_values.exists<gtsam::Pose3>(key)) {
            gtsam::Pose3 pose = input_values.at<gtsam::Pose3>(key);
            Eigen::Vector3f trans(pose.x(), pose.y(), pose.z());
            value_vertices.push_back(trans);
        }
    }
    bool line_strip = true;
    auto setting0_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting0_v = guik::FlatRed().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting1_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(100.0f, 0.0f, 0.0f);
    auto setting1_v = guik::FlatGreen().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(100.0f, 0.0f, 0.0f);
    // Draw the Line for inital
    if (line_id == 0){
    // Draw the lines (GL_LINES) for graph
        viewer->update_drawable("graph_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(graph_vertices, line_strip), setting0_g);   

    // Draw the lines (GL_LINES) for values
        viewer->update_drawable("value_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting0_v);
    }
    else{ // Draw the Line for optimized
    // Draw the lines (GL_LINES) for graph
        viewer->update_drawable("graph_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(graph_vertices, line_strip), setting1_g);   

        // Draw the lines (GL_LINES) for values
        viewer->update_drawable("value_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting1_v);
    }
}

struct VertexWithKey {
    Eigen::Vector3f vertex;
    Key key;
};

void draw_3d_trajectory_and_only_3_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id, const bool is_custom_factor){

    std::vector<std::pair<VertexWithKey, VertexWithKey>> graph_edges;
    std::vector<VertexWithKey> graph_vertices;
    std::vector<std::vector<Eigen::Vector3f>> top3_vertices;
    std::vector<Eigen::Vector3f> value_vertices;
    std::vector<Eigen::Vector4f> line_colors;

    // Set CustomBetweenFactor for line_colors
    if (is_custom_factor == true) {
        std::cout << "custom factor is used\n";
        for (const auto& factor : graph) {
            auto customFactor = boost::dynamic_pointer_cast<CustomBetweenFactor>(factor);
            if (customFactor) {
                Key key1 = customFactor->key1();
                Key key2 = customFactor->key2();

                if (input_values.exists<gtsam::Pose3>(key1) && input_values.exists<gtsam::Pose3>(key2)) {
                    gtsam::Pose3 pose1 = input_values.at<gtsam::Pose3>(key1);
                    gtsam::Pose3 pose2 = input_values.at<gtsam::Pose3>(key2);

                    VertexWithKey vertex1{Eigen::Vector3f(pose1.x(), pose1.y(), pose1.z()), key1};
                    VertexWithKey vertex2{Eigen::Vector3f(pose2.x(), pose2.y(), pose2.z()), key2};

                    graph_vertices.push_back(vertex1);
                    graph_vertices.push_back(vertex2);
                    graph_edges.emplace_back(vertex1, vertex2);
                    line_colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, 3.0));
                }
            }
        }
    }
    // Set gtsam::BetweenFactor<gtsam::Pose3> for line_colors
    else {
        for (const auto& factor : graph) {
            auto baseFactor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
            if (baseFactor) {
                Key key1 = baseFactor->key1();
                Key key2 = baseFactor->key2();

                if (input_values.exists<gtsam::Pose3>(key1) && input_values.exists<gtsam::Pose3>(key2)) {
                    gtsam::Pose3 pose1 = input_values.at<gtsam::Pose3>(key1);
                    gtsam::Pose3 pose2 = input_values.at<gtsam::Pose3>(key2);

                    VertexWithKey vertex1{Eigen::Vector3f(pose1.x(), pose1.y(), pose1.z()), key1};
                    VertexWithKey vertex2{Eigen::Vector3f(pose2.x(), pose2.y(), pose2.z()), key2};

                    graph_vertices.push_back(vertex1);
                    graph_vertices.push_back(vertex2);
                    graph_edges.emplace_back(vertex1, vertex2);
                    line_colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, 3.0));
                }
            }
        }
    }

    // Calculate the length of the edges and sort them
    std::sort(graph_edges.begin(), graph_edges.end(),
        [](const std::pair<VertexWithKey, VertexWithKey>& a, const std::pair<VertexWithKey, VertexWithKey>& b) {
            float length_a = (a.second.vertex - a.first.vertex).norm();
            float length_b = (b.second.vertex - b.first.vertex).norm();
            return length_a > length_b;  // Sort in descending order of length
        });

    // Extract the vertices of the longest edges
    for (size_t i = 0; i < std::min(size_t(3), graph_edges.size()); ++i) {
        const auto& edge = graph_edges[i];
        top3_vertices.push_back({edge.first.vertex, edge.second.vertex});

    // std::cout << "Longest Edge " << i + 1 << ": (" << edge.first.vertex.x() << ", " 
    //           << edge.first.vertex.y() << ", " << edge.first.vertex.z() << ", Key: " 
    //           << edge.first.key << ") -> (" 
    //           << edge.second.vertex.x() << ", " << edge.second.vertex.y() << ", " 
    //           << edge.second.vertex.z() << ", Key: " << edge.second.key << ")" << std::endl;
    }

    // Extract the vertices from the gtsam::Values object
    for (const auto& key_value_pair : input_values) {
        auto key = key_value_pair.key;
        if (input_values.exists<gtsam::Pose3>(key)) {
            gtsam::Pose3 pose = input_values.at<gtsam::Pose3>(key);
            Eigen::Vector3f trans(pose.x(), pose.y(), pose.z());
            value_vertices.push_back(trans);
        }
    }

    bool line_strip = true;
    auto setting0_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting0_v = guik::FlatRed().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting1_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(100.0f, 0.0f, 0.0f);
    auto setting1_v = guik::FlatGreen().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(100.0f, 0.0f, 0.0f);

    // Draw the Line for inital
      if (line_id == 0){
        for (int i = 0 ;i < top3_vertices.size(); i++){
        viewer->update_drawable("graph" + std::to_string(line_id) + std::to_string(i), std::make_shared<glk::Lines>(0.1f, top3_vertices.at(i), line_colors ,line_strip),setting0_g);
        }
        viewer -> update_drawable("value_thin_lines" + std::to_string(line_id) + std::to_string(0), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting0_v);
      }

      else {
        for (int i = 0 ;i < top3_vertices.size(); i++){
        viewer->update_drawable("graph" + std::to_string(line_id) + std::to_string(i), std::make_shared<glk::Lines>(0.1f, top3_vertices.at(i), line_colors, line_strip), setting1_g);
        }
        viewer -> update_drawable("value_thin_lines" + std::to_string(line_id) + std::to_string(0), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting1_v);
      }

}

void draw_2d_trajectory_and_all_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id) {
    std::vector<Eigen::Vector3f> graph_vertices;
    std::vector<Eigen::Vector3f> value_vertices;

    // Extract the vertices from the NonlinearFactorGraph
    for (const auto& factor : graph) {
        for (const auto& key : factor->keys()) {
            if (input_values.exists<gtsam::Pose2>(key)) {
                gtsam::Pose2 pose = input_values.at<gtsam::Pose2>(key);
                Eigen::Vector3f trans(pose.x(), pose.y(), 0.0);
                graph_vertices.push_back(trans);
            }
        }
    }

    // Extract the vertices from the gtsam::Values object
    for (const auto& key_value_pair : input_values) {
        auto key = key_value_pair.key;
        if (input_values.exists<gtsam::Pose2>(key)) {
            gtsam::Pose2 pose = input_values.at<gtsam::Pose2>(key);
            Eigen::Vector3f trans(pose.x(), pose.y(), 0.0);
            value_vertices.push_back(trans);
        }
    }
    bool line_strip = true;
    auto setting0_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting0_v = guik::FlatRed().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting1_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(70.0f, 0.0f, 0.0f);
    auto setting1_v = guik::FlatGreen().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(70.0f, 0.0f, 0.0f);

    // Draw the Line for inital
    if (line_id == 0){
    // Draw the lines (GL_LINES) for graph
        viewer->update_drawable("graph_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(graph_vertices, line_strip), setting0_g);   

    // Draw the lines (GL_LINES) for values
        viewer->update_drawable("value_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting0_v);
    }
    else{ // Draw the Line for optimized
    // Draw the lines (GL_LINES) for graph
        viewer->update_drawable("graph_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(graph_vertices, line_strip),setting1_g);   

        // Draw the lines (GL_LINES) for values
        viewer->update_drawable("value_thin_lines" + std::to_string(line_id), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting1_v);
    }

}

void draw_2d_trajectory_and_only_3_constraints(const gtsam::NonlinearFactorGraph &graph, const gtsam::Values &input_values, std::shared_ptr<guik::LightViewer> &viewer, const int line_id){
    
    struct VertexWithKey {
        Eigen::Vector3f vertex;
        Key key;
    };

    std::vector<std::pair<VertexWithKey, VertexWithKey>> graph_edges;
    std::vector<VertexWithKey> graph_vertices;
    std::vector<Eigen::Vector3f> value_vertices;
    std::vector<std::vector<Eigen::Vector3f>> top3_vertices;
    std::vector<Eigen::Vector4f> line_colors;

    for (const auto& factor : graph) {
        // Convert the factor to a BetweenFactor<Pose2>
        auto betweenFactor = boost::dynamic_pointer_cast<BetweenFactor<Pose2>>(factor);
        if (betweenFactor) {
            Key key1 = betweenFactor->key1();
            Key key2 = betweenFactor->key2();

            // Confirm that both keys exist in input_values and are of type gtsam::Pose2
            if (input_values.exists<Pose2>(key1) && input_values.exists<Pose2>(key2)) {
                Pose2 pose1 = input_values.at<Pose2>(key1);
                Pose2 pose2 = input_values.at<Pose2>(key2);

                VertexWithKey vertex1{Eigen::Vector3f(pose1.x(), pose1.y(), 0.0), key1};
                VertexWithKey vertex2{Eigen::Vector3f(pose2.x(), pose2.y(), 0.0), key2};

                graph_vertices.push_back(vertex1);
                graph_vertices.push_back(vertex2);

                // Store edges as pairs
                graph_edges.emplace_back(vertex1, vertex2);
                
                // Add colors
                line_colors.push_back(glk::colormapf(glk::COLORMAP::TURBO, 3.0));
            }
        }
    }

    // Calculate the length of the edges and sort them
    std::sort(graph_edges.begin(), graph_edges.end(),
        [](const std::pair<VertexWithKey, VertexWithKey>& a, const std::pair<VertexWithKey, VertexWithKey>& b) {
            float length_a = (a.second.vertex - a.first.vertex).norm();
            float length_b = (b.second.vertex - b.first.vertex).norm();
            return length_a > length_b;  // Sort in descending order of length
        });

    //  Extract the vertices of the longest edges
    for (size_t i = 0; i < std::min(size_t(3), graph_edges.size()); ++i) {
        const auto& edge = graph_edges[i];
        top3_vertices.push_back({edge.first.vertex, edge.second.vertex});

    // std::cout << "Longest Edge " << i + 1 << ": (" << edge.first.vertex.x() << ", " 
    //           << edge.first.vertex.y()  << ", Key: " 
    //           << static_cast<int>(edge.first.key) << ") -> (" 
    //           << edge.second.vertex.x() << ", " << edge.second.vertex.y() << ", " 
    //           << ", Key: " << static_cast<int>(edge.second.key) << ")" << std::endl;
    }

    // Extract the vertices from the gtsam::Values object
    for (const auto& key_value_pair : input_values) {
        auto key = key_value_pair.key;
        if (input_values.exists<gtsam::Pose2>(key)) {
            gtsam::Pose2 pose = input_values.at<gtsam::Pose2>(key);
            Eigen::Vector3f trans(pose.x(), pose.y(), 0.0);
            value_vertices.push_back(trans);
        }
    }

    bool line_strip = true;
    auto setting0_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting0_v = guik::FlatRed().rotate(1.5708f, {0.0f, 0.0f, 1.0f});
    auto setting1_g = guik::FlatBlue().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(70.0f, 0.0f, 0.0f);
    auto setting1_v = guik::FlatGreen().rotate(1.5708f, {0.0f, 0.0f, 1.0f}).translate(70.0f, 0.0f, 0.0f);

    // Draw the Line for inital
      guik::FlatColor line_color = guik::FlatColor(1.0f, 0.0f, 0.0f, 0.0f);

      if (line_id == 0){
        for (int i = 0 ;i < top3_vertices.size(); i++){
        viewer->update_drawable("graph" + std::to_string(line_id) + std::to_string(i), std::make_shared<glk::Lines>(0.1f, top3_vertices.at(i), line_colors ,line_strip), setting0_g);
        }
        viewer -> update_drawable("value_thin_lines" + std::to_string(line_id) + std::to_string(0), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting0_v);
      }

      else {
        for (int i = 0 ;i < top3_vertices.size(); i++){
        viewer->update_drawable("graph" + std::to_string(line_id) + std::to_string(i), std::make_shared<glk::Lines>(0.1f, top3_vertices.at(i), line_colors, line_strip), setting1_g);
        }
        viewer -> update_drawable("value_thin_lines" + std::to_string(line_id) + std::to_string(0), std::make_shared<glk::ThinLines>(value_vertices, line_strip), setting1_v);
      }

}

