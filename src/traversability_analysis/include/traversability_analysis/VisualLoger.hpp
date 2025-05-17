
#pragma once

#include "traversability_analysis/utilities.hpp"
#include "traversability_analysis/ClustersManager.hpp" 


class VisualLoger {
public:
    VisualLoger(std::string Dir, std::string filename);
    ~VisualLoger();
    void addSceneField(const std::string field_name, const nlohmann::json json_value) ;
    void wrapUpScene();
    
   
    void addClustersField(const std::string& field_name, ClustersManager &clusters){
        std::promise<bool> task_promise; // Promise to signal the completion of the task.
        ClustersThread_ = task_promise.get_future(); 
        std::thread t(&VisualLoger::addClustersFieldTask, this, field_name, clusters, std::move(task_promise));
        t.detach();
    }
    void saveFrame(pcl::PointCloud<PointType> pc){
        // No mutex required because the data are not being modified.
        std::promise<bool> task_promise; // Promise to signal the completion of the task.
        FrameThread_ = task_promise.get_future(); 
        std::thread t(&VisualLoger::saveFrameTask, this, std::move(pc), std::move(task_promise));
        t.detach();

    }
    void saveImage(cv::Mat &image){
        std::promise<bool> task_promise; // Promise to signal the completion of the task.
        ImageThread_ = task_promise.get_future();
        std::thread t(&VisualLoger::saveImageTask, this, image.clone(), std::move(task_promise));
        t.detach();
    }


private:
    void addClustersFieldTask(const std::string field_name, ClustersManager clusters, std::promise<bool> task_promise);
    void saveFrameTask(pcl::PointCloud<PointType> pc, std::promise<bool> task_promise);
    void saveImageTask(cv::Mat image, std::promise<bool> task_promise);

    std::string filename_,Dir_;          // Filename of the JSON file
    nlohmann::json current_scene_;  // Current scene being constructed
    int64_t FrameId_;
    std::mutex mtx_;
    std::future<bool> ClustersThread_, FrameThread_, ImageThread_;
    
};





// TODO:
// 1. Implement the VisualLoger class. (Status: Done)
// 2. Add the function to save frame to disk. (Status: Pending)
// 2. Add the ability to save images to the log. (Status: Pending)
// 3. Add the Level of information functionality. (Status: Pending)


