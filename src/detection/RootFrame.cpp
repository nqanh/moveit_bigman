#include "RootFrame.h"

RootFrame::RootFrame ()
{
}

RootFrame::~RootFrame ()
{
}

void
RootFrame::setRootRotation (const cv::Mat &rotation_matrix)
{
  rotation_matrix_.push_back (rotation_matrix);
}

void
RootFrame::save (const std::string path)
{
  cv::FileStorage storage (path, cv::FileStorage::WRITE);
  storage << "rotation_matrix" << rotation_matrix_;
  storage.release ();
}

void
RootFrame::load (const std::string path)
{
  cv::FileStorage storage (path, cv::FileStorage::READ);
  storage["rotation_matrix"] >> rotation_matrix_;
  storage.release ();
}

