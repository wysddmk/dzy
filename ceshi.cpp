#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp> //OpenCV highgui 模块头文件 ~
#include <opencv2/imgproc/imgproc.hpp> //OpenCV 图像处理头文件
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using namespace std;
using namespace cv;
int findLowestRotatedRectContourIndexR(const vector<vector<Point>> &contours);
Point2f calculateThirdPoint(const Point2f &P1, const Point2f &P2, float t);
pair<float, float> getLongAndShortEdges(const RotatedRect &rotatedRect);
int main()
{
    int number[2] = {40, 17};

    cout << "蓝色请输入0  红色请输入1";
    int choice;
    cin >> choice;
    int numthreshold = number[choice];
    VideoCapture video("l1.mp4");
    while (1)
    {
        Mat image;
        video >> image;
        if (image.empty())
        {
            break;
        }
        Mat gray, gray1;
        cvtColor(image, gray, COLOR_BGR2GRAY);

        threshold(gray, gray1, numthreshold, 255, 0);
        Mat kernel = getStructuringElement(0, Size(3, 3));

        morphologyEx(gray1, gray1, 2, kernel);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(gray1, contours, hierarchy, 3, 1);
        vector<vector<Point>> Rcontour;
        vector<vector<Point>> filteredContours;
        vector<vector<Point>> repair;
        vector<vector<Point>> resultcontour;
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            RotatedRect rect1 = minAreaRect(contours[i]);
            pair<float, float> aaa = getLongAndShortEdges(rect1);
            double aspectRatio = aaa.first;
            double fillrate = area / aaa.second;

            if (fillrate < 0.215 && 1000 < area && area < 3200 && 1.26 < aspectRatio && aspectRatio < 1.32 && hierarchy[i][2] == -1 && hierarchy[i][3] == -1)
            {
                filteredContours.push_back(contours[i]);
            }
            if (0.765 < fillrate && 0.975 < aspectRatio && aspectRatio < 1.135 && 135 < area && area < 500 && hierarchy[i][2] == -1 && hierarchy[i][3] == -1)
            {
                Rcontour.push_back(contours[i]);
            }
            if (0.625 < fillrate && 5.26 < aspectRatio && aspectRatio < 5.84 && 100 < area && area < 1600 && hierarchy[i][2] == -1 && hierarchy[i][3] == -1)
            {
                repair.push_back(contours[i]);
            }
        }

        if (Rcontour.size() > 0)
        {
            int a = findLowestRotatedRectContourIndexR(Rcontour);
            float t;
            drawContours(image, Rcontour, a, Scalar(0, 255, 0), 4);
            if (filteredContours.size() > 0)
            {
                resultcontour.push_back(filteredContours[0]);
                t = 1.68;
            }
            else if (repair.size() > 0)
            {
                resultcontour.push_back(repair[0]);
                t = 2.075;
            }

            if (resultcontour.size() > 0)
            {
                drawContours(image, resultcontour, 0, Scalar(0, 0, 255));
                RotatedRect juxing = minAreaRect(resultcontour[0]);
                Point2f centerJ = juxing.center;

                RotatedRect R = minAreaRect(Rcontour[a]);

                Point2f centerR = R.center;
                circle(image, centerR, 3, Scalar(0, 255, 0), 3);
                Point2f result = calculateThirdPoint(centerJ, centerR, t);
                circle(image, result, 2, Scalar(0, 255, 0), 7, 3, 0);
            }
        }
        imshow("result", image);
        waitKey(15);
    }
    return 0;
}
int findLowestRotatedRectContourIndexR(const vector<vector<Point>> &contours)
{

    double maxY = -1;
    int index = -1;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);
        cv::Point2f center = rotatedRect.center; // 获取中心点

        if (center.y > maxY)
        {
            maxY = center.y;
            index = i;
        }
    }

    return index; // 返回 y 值最大的轮廓索引
}

Point2f calculateThirdPoint(const Point2f &P1, const Point2f &P2, float t)
{
    // 计算向量 P2P1
    cv::Point2f vectorP2P1 = P1 - P2;

    // 计算 P2P1 的模长（即两点之间的距离）
    float magnitudeP2P1 = cv::norm(vectorP2P1);
    // 单位化向量 P2P1
    cv::Point2f unitVectorP2P1 = vectorP2P1 / magnitudeP2P1;
    float distanceFromP2 = t * magnitudeP2P1;
    // 计算第三点的位置，即在单位向量方向上移动 distanceFromP2 的距离
    cv::Point2f P3 = P2 + unitVectorP2P1 * distanceFromP2;

    return P3;
}
pair<float, float> getLongAndShortEdges(const RotatedRect &rotatedRect)
{
    float longEdge, shortEdge;
    if (rotatedRect.size.width > rotatedRect.size.height)
    {
        longEdge = rotatedRect.size.width;
        shortEdge = rotatedRect.size.height;
    }
    else
    {
        longEdge = rotatedRect.size.height;
        shortEdge = rotatedRect.size.width;
    }

    float aspectRatio = longEdge / shortEdge;
    float area = longEdge * shortEdge;
    return make_pair(aspectRatio, area);
}
