#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc_c.h> // cvFindContours
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

// Function prototypes
void
subtractPlane (const cv::Mat& depth,
               cv::Mat& mask,
               std::vector<CvPoint>& chain,
               double f);

std::vector<CvPoint>
maskFromTemplate (const std::vector<cv::linemod::Template>& templates,
                  int num_modalities,
                  cv::Point offset,
                  cv::Size size,
                  cv::Mat& mask,
                  cv::Mat& dst);

void
templateConvexHull (const std::vector<cv::linemod::Template>& templates,
                    int num_modalities,
                    cv::Point offset,
                    cv::Size size,
                    cv::Mat& dst);

void
drawResponse (const std::vector<cv::linemod::Template>& templates,
              int num_modalities,
              cv::Mat& dst,
              cv::Point offset,
              int T);

cv::Mat
displayQuantized (const cv::Mat& quantized);

// Copy of cv_mouse from cv_utilities
class Mouse
{
  public:
    static void
    start (const std::string& a_img_name)
    {
      cvSetMouseCallback (a_img_name.c_str (), Mouse::cv_on_mouse, 0);
    }
    static int
    event (void)
    {
      int l_event = m_event;
      m_event = -1;
      return l_event;
    }
    static int
    x (void)
    {
      return m_x;
    }
    static int
    y (void)
    {
      return m_y;
    }

  private:
    static void
    cv_on_mouse (int a_event,
                 int a_x,
                 int a_y,
                 int,
                 void *)
    {
      m_event = a_event;
      m_x = a_x;
      m_y = a_y;
    }

    static int m_event;
    static int m_x;
    static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;

static void
help ()
{
  printf ("Usage: openni_demo [templates.yml]\n\n"
          "Place your object on a planar, featureless surface. With the mouse,\n"
          "frame it in the 'color' window and right click to learn a first template.\n"
          "Then press 'l' to enter online learning mode, and move the camera around.\n"
          "When the match score falls between 90-95%% the demo will add a new template.\n\n"
          "Keys:\n"
          "\t h   -- This help page\n"
          "\t l   -- Toggle online learning\n"
          "\t m   -- Toggle printing match result\n"
          "\t t   -- Toggle printing timings\n"
          "\t w   -- Write learned templates to disk\n"
          "\t [ ] -- Adjust matching threshold: '[' down,  ']' up\n"
          "\t q   -- Quit\n\n");
}

// Adapted from cv_timer in cv_utilities
class Timer
{
  public:
    Timer () :
        start_ (0),
        time_ (0)
    {
    }

    void
    start ()
    {
      start_ = cv::getTickCount ();
    }

    void
    stop ()
    {
      CV_Assert(start_ != 0);
      int64 end = cv::getTickCount ();
      time_ += end - start_;
      start_ = 0;
    }

    double
    time ()
    {
      double ret = time_ / cv::getTickFrequency ();
      time_ = 0;
      return ret;
    }

  private:
    int64 start_, time_;
};

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector>
readLinemod (const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
//  cv::FileStorage fs (filename, cv::FileStorage::READ);
  cv::FileStorage fs ("linemod_templates.yml", cv::FileStorage::READ);
  detector->read (fs.root ());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin (), iend = fn.end (); i != iend; ++i)
    detector->readClass (*i);

  return detector;
}

int
main (int argc,
      char * argv[])
{

  //cv::CommandLineParser parser(argc, argv, 1);

  // Various settings and flags
  bool show_match_result = true;
  bool show_timings = false;
  bool learn_online = false;
  int num_classes = 0;
  int matching_threshold = 60;

  cv::Size roi_size (120, 60);
//  int learning_lower_bound = 90;
//  int learning_upper_bound = 95;

// Timers
  Timer extract_timer;
  Timer match_timer;

  // Initialize HighGUI
  help ();
  cv::namedWindow ("color");
  cv::namedWindow ("normals");
  Mouse::start ("color");

  //Debuging...
  // /home/anh/catkin_ws/build/pre_grasp/linemod_templates.yml

  // Initialize LINEMOD data structures
  cv::Ptr<cv::linemod::Detector> detector;
  std::string filename;

  printf ("Loading linemod template......................... \n");

  detector = readLinemod (argv[1]);

  std::vector<std::string> ids = detector->classIds ();
  num_classes = detector->numClasses ();
  printf ("Loaded %s with %d classes and %d templates\n", argv[1], num_classes, detector->numTemplates ());
  if (!ids.empty ())
  {
    printf ("Class ids:\n");
    std::copy (ids.begin (), ids.end (), std::ostream_iterator<std::string> (std::cout, "\n"));
  }

  int num_modalities = (int) detector->getModalities ().size ();
  printf ("Number of modalities: %d\n", num_modalities);

  // Open Kinect sensor
  cv::VideoCapture capture (CV_CAP_OPENNI);
  if (!capture.isOpened ())
  {
    printf ("Could not open OpenNI-capable sensor\n");
    return -1;
  }
  capture.set (CV_CAP_PROP_OPENNI_REGISTRATION, 1);
//  double focal_length = capture.get (CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
  //printf("Focal length = %f\n", focal_length);

  // Main loop
//  bool isFound = false;
  cv::Mat color, depth;
  for (;;)
  {
    // Capture next color/depth pair
    capture.grab ();
    capture.retrieve (depth, CV_CAP_OPENNI_DEPTH_MAP);
    capture.retrieve (color, CV_CAP_OPENNI_BGR_IMAGE);

    std::vector<cv::Mat> sources;
    sources.push_back (color);
    sources.push_back (depth);
    cv::Mat display = color.clone ();

    // Perform matching
    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;
    match_timer.start ();
    detector->match (sources, (float) matching_threshold, matches, class_ids, quantized_images);
    match_timer.stop ();

    double focal_length = capture.get (CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH);
    printf ("Focal length = %f\n", focal_length);

    int classes_visited = 0;
    std::set<std::string> visited;

    printf ("Matches size: %d\n", (int) matches.size ());
    for (int i = 0; (i < (int) matches.size ()) && (classes_visited < num_classes); ++i)
    {
      cv::linemod::Match m = matches[i];

      if (visited.insert (m.class_id).second)
      {
        ++classes_visited;

        std::cout << "Visited : " << classes_visited << std::endl;

        if (show_match_result)
        {
          printf ("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s; template: %3d\n", m.similarity, m.x, m.y, m.class_id.c_str (), m.template_id);
        }

        // Draw matching template
        const std::vector<cv::linemod::Template>& templates = detector->getTemplates (m.class_id, m.template_id);
        drawResponse (templates, num_modalities, display, cv::Point (m.x, m.y), detector->getT (0));
      }
    }

    if (show_match_result && matches.empty ())
      printf ("No matches found...\n");

    if (show_timings)
      printf ("Matching: %.2fs\n", match_timer.time ());

    cv::imshow ("color", display);
    cv::imshow ("normals", quantized_images[1]);

//    printf ("Waiting for input ... \n");
//    getchar ();

    cv::FileStorage fs;
    char key = (char) cvWaitKey (10);
    if (key == 'q' || key == 'n')
      break;

    switch (key)
    {
      case 'h':
        help ();
        break;
      case 'm':
        // toggle printing match result
        show_match_result = !show_match_result;
        printf ("Show match result %s\n", show_match_result ? "ON" : "OFF");
        break;
      case 't':
        // toggle printing timings
        show_timings = !show_timings;
        printf ("Show timings %s\n", show_timings ? "ON" : "OFF");
        break;
      case 'l':
        // toggle online learning
        learn_online = !learn_online;
        printf ("Online learning %s\n", learn_online ? "ON" : "OFF");
        break;
      case '[':
        // decrement threshold
        matching_threshold = std::max (matching_threshold - 1, -100);
        printf ("New threshold: %d\n", matching_threshold);
        break;
      case ']':
        // increment threshold
        matching_threshold = std::min (matching_threshold + 1, +100);
        printf ("New threshold: %d\n", matching_threshold);
        break;
      default:
        ;
    }
  }
  return 0;
}

std::vector<CvPoint>
maskFromTemplate (const std::vector<cv::linemod::Template>& templates,
                  int num_modalities,
                  cv::Point offset,
                  cv::Size size,
                  cv::Mat& mask,
                  cv::Mat& dst)
{
  templateConvexHull (templates, num_modalities, offset, size, mask);

  const int OFFSET = 30;
  cv::dilate (mask, mask, cv::Mat (), cv::Point (-1, -1), OFFSET);

  CvMemStorage * lp_storage = cvCreateMemStorage (0);
  CvTreeNodeIterator l_iterator;
  CvSeqReader l_reader;
  CvSeq * lp_contour = 0;

  cv::Mat mask_copy = mask.clone ();
  IplImage mask_copy_ipl = mask_copy;
  cvFindContours (&mask_copy_ipl, lp_storage, &lp_contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  std::vector<CvPoint> l_pts1;  // to use as input to cv_primesensor::filter_plane

  cvInitTreeNodeIterator (&l_iterator, lp_contour, 1);
  while ( (lp_contour = (CvSeq *) cvNextTreeNode (&l_iterator)) != 0)
  {
    CvPoint l_pt0;
    cvStartReadSeq (lp_contour, &l_reader, 0);
    CV_READ_SEQ_ELEM(l_pt0, l_reader);
    l_pts1.push_back (l_pt0);

    for (int i = 0; i < lp_contour->total; ++i)
    {
      CvPoint l_pt1;
      CV_READ_SEQ_ELEM(l_pt1, l_reader);
      /// @todo Really need dst at all? Can just as well do this outside
      cv::line (dst, l_pt0, l_pt1, CV_RGB(0, 255, 0), 2);

      l_pt0 = l_pt1;
      l_pts1.push_back (l_pt0);
    }
  }
  cvReleaseMemStorage (&lp_storage);

  return l_pts1;
}

// Adapted from cv_show_angles
cv::Mat
displayQuantized (const cv::Mat& quantized)
{
  cv::Mat color (quantized.size (), CV_8UC3);
  for (int r = 0; r < quantized.rows; ++r)
  {
    const uchar* quant_r = quantized.ptr (r);
    cv::Vec3b* color_r = color.ptr<cv::Vec3b> (r);

    for (int c = 0; c < quantized.cols; ++c)
    {
      cv::Vec3b& bgr = color_r[c];
      switch (quant_r[c])
      {
        case 0:
          bgr[0] = 0;
          bgr[1] = 0;
          bgr[2] = 0;
          break;
        case 1:
          bgr[0] = 55;
          bgr[1] = 55;
          bgr[2] = 55;
          break;
        case 2:
          bgr[0] = 80;
          bgr[1] = 80;
          bgr[2] = 80;
          break;
        case 4:
          bgr[0] = 105;
          bgr[1] = 105;
          bgr[2] = 105;
          break;
        case 8:
          bgr[0] = 130;
          bgr[1] = 130;
          bgr[2] = 130;
          break;
        case 16:
          bgr[0] = 155;
          bgr[1] = 155;
          bgr[2] = 155;
          break;
        case 32:
          bgr[0] = 180;
          bgr[1] = 180;
          bgr[2] = 180;
          break;
        case 64:
          bgr[0] = 205;
          bgr[1] = 205;
          bgr[2] = 205;
          break;
        case 128:
          bgr[0] = 230;
          bgr[1] = 230;
          bgr[2] = 230;
          break;
        case 255:
          bgr[0] = 0;
          bgr[1] = 0;
          bgr[2] = 255;
          break;
        default:
          bgr[0] = 0;
          bgr[1] = 255;
          bgr[2] = 0;
          break;
      }
    }
  }

  return color;
}

// Adapted from cv_line_template::convex_hull
void
templateConvexHull (const std::vector<cv::linemod::Template>& templates,
                    int num_modalities,
                    cv::Point offset,
                    cv::Size size,
                    cv::Mat& dst)
{
  std::vector<cv::Point> points;
  for (int m = 0; m < num_modalities; ++m)
  {
    for (int i = 0; i < (int) templates[m].features.size (); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      points.push_back (cv::Point (f.x, f.y) + offset);
    }
  }

  std::vector<cv::Point> hull;
  cv::convexHull (points, hull);

  dst = cv::Mat::zeros (size, CV_8U);
  const int hull_count = (int) hull.size ();
  const cv::Point* hull_pts = &hull[0];
  cv::fillPoly (dst, &hull_pts, &hull_count, 1, cv::Scalar (255));
}

void
drawResponse (const std::vector<cv::linemod::Template>& templates,
              int num_modalities,
              cv::Mat& dst,
              cv::Point offset,
              int T)
{
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    // NOTE: Original demo recalculated max response for each feature in the TxT
    // box around it and chose the display color based on that response. Here
    // the display color just depends on the modality.
    cv::Scalar color = COLORS[m];

    for (int i = 0; i < (int) templates[m].features.size (); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt (f.x + offset.x, f.y + offset.y);
      cv::circle (dst, pt, T / 2, color);
    }

    int xtep = 0;
  }
}
