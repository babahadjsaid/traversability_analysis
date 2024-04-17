
#include "traversability_analysis/traversabilityAnalysis.hpp"







int main(int argc, char** argv) {
   rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto TA = std::make_shared<traversability_analysis::TraversabilityAnalysis>(std::string("traversability_analysis"),options);
    
    exec.add_node(TA);


    exec.spin();

    rclcpp::shutdown();
    return 0;
}
