//
// Created by robostar on 2019/11/3.
//

cv::Mat bgr_roi, hsv_roi, color_mask_roi;       //hsv不知道是不是最好的,目前还行
bgr_roi = img_backups(min_rect);
cv::cvtColor(bgr_roi, hsv_roi, cv::COLOR_BGR2HSV
);

if (autohit::color_type == 0)
cv::inRange(hsv_roi, cv::Scalar(100, 100, 100), cv::Scalar(124, 255, 255), color_mask_roi
);
else
{
cv::Mat hsv1, hsv2;
cv::inRange(hsv_roi, cv::Scalar(0, 43, 46), cv::Scalar(50, 255, 255), hsv1
);
cv::inRange(hsv_roi, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), hsv2
);
color_mask_roi = hsv1 + hsv2;
}

int correct_pxl = 0;
for (
int j = 0;
j<color_mask_roi.
rows;
j++)
{
auto *hsv_ptr = color_mask_roi.ptr<uchar>(j);
auto *bgr_ptr = bgr_roi.ptr<uchar>(j);

for (
int k = 0;
k<color_mask_roi.
cols;
k++)
{
auto hsv_val = hsv_ptr[k];
auto b = bgr_ptr[k * 3];
auto r = bgr_ptr[k * 3 + 2];

if (autohit::color_type == 0)    //蓝色
{
//                            if (hsv_val && (b > 50) && (r < 80) && (b - 2 * r > 10))
if(hsv_val)
correct_pxl++;
}
else                  //红色
{
//                            if (hsv_val && (r > 70) && (r - b > 50) && (b < 110))
//                            if(hsv_val)
if(
hsv_val &&r
- b > 80)
correct_pxl++;
}
}
}

float pxl_persent = (float) correct_pxl / area_of_contour;
if (pxl_persent<autohit::pixel_percent_limit)
{
#ifdef show
cv::putText(contours_img, "pxl", min_ellipse.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
continue;
}
}