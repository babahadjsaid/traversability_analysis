#include "traversability_analysis/VisualLoger.hpp"
#include <opencv2/imgcodecs.hpp>
 
VisualLoger::VisualLoger(std::string Dir, std::string filename) : filename_(Dir+filename), Dir_(Dir) {
    FrameId_ = 0;
    // Open file and write the opening brace
    createDirectoryIfNotExists(Dir);
    std::ofstream file(filename_, std::ios::out | std::ios::trunc);
    if (file.is_open()) {
        file << "{\n\"Scenes\": [\n";  // Start JSON array
        file.close();
    }
}

VisualLoger::~VisualLoger() {
    // Close JSON array and add closing brace to the file
    std::ofstream file(filename_, std::ios::out | std::ios::app);
    if (file.is_open()) {
        auto write_pos = (int) file.tellp();
        
        file << "{}\n]\n}";  // Close JSON array and object
        file.close();
    }else {
        std::cerr << "Unable to open file: " << filename_ << std::endl;
    }
}

void VisualLoger::addSceneField(const std::string field_name, const nlohmann::json json_value) {
    mtx_.lock();
    // Add a new field to the current scene
    current_scene_[field_name] = json_value;
    mtx_.unlock();
}

void VisualLoger::wrapUpScene() {
    std::cout << "Wrapping up scene" << std::endl;
    ImageThread_.wait();
    std::cout << "Image thread finished" << std::endl;
    FrameThread_.wait();
    std::cout << "Frame thread finished" << std::endl;
    ClustersThread_.wait();
    std::cout << "Clusters thread finished" << std::endl;
    mtx_.lock();
    current_scene_["0-ID"] = FrameId_;
    // Save the current scene to disk
    std::ofstream file(filename_, std::ios::out | std::ios::app);
    if (file.is_open()) {
        
        file << current_scene_.dump(4);  // Dump the current scene with 4-space indent

        file << ",\n";  // Add comma for next scene (to be removed by destructor if last)
        file.close();
    }
    FrameId_++;
    current_scene_.clear();  // Clear the current scene for the next instance
    mtx_.unlock();
    
}

void VisualLoger::addClustersFieldTask(const std::string field_name, ClustersManager clusters, std::promise<bool> task_promise) {
    // Add clusters field to the current scene
    nlohmann::json clusters_json = clusters.serializeMultipleClustersToJson(Dir_);
    mtx_.lock();
    current_scene_[field_name] = clusters_json;
    mtx_.unlock();
    task_promise.set_value(true);
}

void VisualLoger::saveFrameTask(pcl::PointCloud<PointType> pc, std::promise<bool> task_promise) {
    mtx_.lock();
    if (pc.points.size() == 0)
    {
        current_scene_["Frame"] = "empty";
        mtx_.unlock();
        return ;
    }
    std::string directory = Dir_+"frames/";
    createDirectoryIfNotExists(directory);
    std::stringstream ss;
    ss << directory <<"Frame_" << FrameId_ << ".pcd";
    std::string filename = ss.str();
    current_scene_["Frame"] = filename;
    mtx_.unlock();
    pcl::PCDWriter writer;
    writer.writeBinaryCompressed(filename, pc);
    // free(&pc);
    task_promise.set_value(true);
    
}

void VisualLoger::saveImageTask(cv::Mat image, std::promise<bool> task_promise) {

    mtx_.lock();
    if (image.empty())
    {
        current_scene_["Image"] = "empty";
        mtx_.unlock();
        return ;
    }
    
    std::string directory = Dir_+"images/";
    createDirectoryIfNotExists(directory);
    std::stringstream ss;
    ss << directory << "image_" << FrameId_ << ".png";
    std::string filename = ss.str();
    current_scene_["Image"] = filename;
    mtx_.unlock();
    cv::imwrite(filename, image);
    // free(&image);
    task_promise.set_value(true);

}


