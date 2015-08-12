#ifndef ROOTFRAME_H_
#define ROOTFRAME_H_

#include <iostream>
#include <opencv2/core/core.hpp>

class RootFrame
{
  public:
    RootFrame ();
    ~ RootFrame ();

    cv::Mat
    getRootRotation () const
    {
      return rotation_matrix_;
    }

    void
    setRootRotation (const cv::Mat &rotation_matrix);

    void
    save (const std::string path);

    void
    load (const std::string path);

  private:
    /**
     * Rotation matrix (3x3)
     */
    cv::Mat rotation_matrix_;

};

#endif /* SRC_DETECTION_ROOTFRAME_H_ */
