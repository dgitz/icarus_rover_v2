#include "visionhelper.h"

VisionHelper::VisionHelper()
{
}
VisionHelper::~VisionHelper()
{
}
cv::Mat VisionHelper::Detect_Edges(cv::Mat gray_image,int threshold)
{
   	cv::Mat det_edges;
   	cv::blur(gray_image,det_edges,cv::Size(3,3));
   	cv::Canny(det_edges,det_edges,threshold,threshold*3,3);
   	return det_edges;
}
