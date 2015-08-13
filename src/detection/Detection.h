#ifndef DETECTION_H_
#define DETECTION_H_

#include "iostream"
#include <opencv2/core/core.hpp>

class Detection
{
  public:
    Detection();
    ~Detection();

    cv::Point3f getRoot3DCamera() const
    {
      return root_3D_camera_;
    }

    cv::Point3f getRoot3DBase() const
    {
      return root_3D_base_;
    }

    cv::Mat getFrameRgbVis() const
    {
      return frame_rgb_vis_;
    }

    cv::Mat getFrameDepthVis() const
    {
      return frame_depth_vis_;
    }

    void setRoot3DCamera(const cv::Point3f &root_3D_camera);
    void setRoot3DBase(const cv::Point3f &root_3D_base);

    void setFrameRgb(const cv::Mat &frame_rgb);
    void setFrameRgbVis(const cv::Mat &frame_rgb);
    void setFrameDepth(const cv::Mat &frame_depth);
    void setFrameDepthVis(const cv::Mat &frame_depth_vis);

    void setTrainDescriptorFolder(const std::string &train_descriptor_folder);
    void setTrainMeshFolder(const std::string &train_mesh_folder);
    void setTrainRootFolder (const std::string &train_root_folder);

    int detect();


  private:
    cv::Point3f root_3D_camera_;
    cv::Point3f root_3D_base_;
    cv::Mat frame_rgb_;
    cv::Mat frame_rgb_vis_;
    cv::Mat frame_depth_;
    cv::Mat frame_depth_vis_;

    std::string train_descriptor_folder_;
    std::string train_mesh_folder_;
    std::string train_root_folder_;

};



#endif /* DETECTION_H_ */
