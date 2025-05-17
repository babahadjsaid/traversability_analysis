#define NOUTILITY
#include "traversability_analysis/traversabilityAnalysis.hpp"







int main(int argc, char** argv) {
   rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;
    std::shared_ptr<rclcpp::Node> nodeHandle = std::make_shared<rclcpp::Node>("traversability_analysis", options);
    
    auto TA = std::make_shared<traversability_analysis::TraversabilityAnalysis>(nodeHandle);
    
    exec.add_node(nodeHandle);
    std::thread loopthread(&traversability_analysis::TraversabilityAnalysis::PubGlobalMap, TA);
    
    const auto methodStartTime = std::chrono::system_clock::now();
    exec.spin();
    const std::chrono::duration<double> durationOfFunction = std::chrono::system_clock::now() - methodStartTime;
    double durationOfFunctionMS = 1000 * durationOfFunction.count();
    rclcpp::shutdown();
    std::cout << "The function " << "spin" << " Took " << durationOfFunctionMS << " ms " << std::endl;
    
    for (auto& [key, value] : TA->times_) {
        if (value.empty()) continue;

        // Calculate mean
        float sum = 0.0;
        for (float time : value) {
            sum += time;
        }
        float mean = sum / value.size();

        // Calculate variance
        float variance_sum = 0.0;
        for (float time : value) {
            variance_sum += (time - mean) * (time - mean);
        }
        float variance = variance_sum / value.size();

        // Display the information
        std::cout << "Function: " << key << std::endl;
        std::cout << "Mean Time: " << mean << " ms" << std::endl;
        std::cout << "Variance: " << variance << " ms^2" << std::endl;
        std::cout << "------------------------" << std::endl;
        }
    
    
    delete &(*TA);
    return 0;
}
