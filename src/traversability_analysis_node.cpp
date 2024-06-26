#define NOUTILITY
#include "traversability_analysis/traversabilityAnalysis.hpp"







int main(int argc, char** argv) {
   rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto TA = std::make_shared<traversability_analysis::TraversabilityAnalysis>(std::string("traversability_analysis"),options);
    
    
    exec.add_node(TA);
    std::thread loopthread(&traversability_analysis::TraversabilityAnalysis::PubGlobalMap, TA);

    exec.spin();

    rclcpp::shutdown();
    std::cout<<"The algorithm took on average "<<(TA->avgTime_/TA->numFrames_)*1000<<" to treat each frame."<<std::endl;
    return 0;
}
