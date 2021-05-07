#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//根据中心点找四角最远的点
void GetRectPoints(Point2f vetPoints[], Point2f center, vector<Point> convex);
//根据最小矩形点找最近的四边形点
void GetPointsFromRect(Point2f vetPoints[], Point2f rectPoints[], vector<Point> convex);

//采用重新定义点做直线拟合后找到的对应点
void GetPointsFromFitline(Point2f newPoints[], Point2f vetPoints[], Point2f rectPoints[], float dist = 15.0f);

//求两条直线的交点
Point2f GetCrossPoint(Vec4f Line1, Vec4f Line2);

//排序旋转矩形坐标点
void SortRotatedRectPoints(Point2f vetPoints[], RotatedRect rect);
//计算距离
float CalcPointDistance(Point2f point1, Point2f point2);



int main(int argc, char** argv) {

	Mat src = imread("E:/DCIM/tsnew.jpg");
	Mat gray, dst, dst2, result;
	//图像缩放
	resize(src, gray, Size(0, 0), 0.2, 0.2);
	imshow("src", gray);

	//灰度图
	cvtColor(gray, dst, COLOR_BGRA2GRAY);

	//高斯滤波
	GaussianBlur(dst, dst, Size(5, 5), 0.5, 0.5);
	//imshow("gray", dst);

	//二值化
	threshold(dst, dst2, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//imshow("thresh", dst2);

	//形态学开操作
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	morphologyEx(dst2, dst2, MORPH_OPEN, kernel);
	//imshow("morph", dst2);

	//Canny边缘检测
	Canny(dst2, result, 127, 255, 7, true);
	//imshow("canny", result);

	//查找轮廓
	vector<vector<Point>> contours;
	findContours(result, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	cout << contours.size() << endl;
	vector<vector<Point>> contours_poly(contours.size());
	Mat tmpgray;
	gray.copyTo(tmpgray);
	for (int i = 0; i < contours.size(); ++i) {
		//计算轮廓周长，大于图像宽度的才算主体
		double dlen = arcLength(contours[i], true);
		if (dlen > gray.cols) {
			//多边形拟合
			approxPolyDP(Mat(contours[i]), contours_poly[i], 10, true);

			cout << "当前：" << i << " 点数：" << contours_poly[i].size() << endl;
			
			if (contours_poly[i].size() >= 4) {
				//获取最小旋转矩形
				RotatedRect rRect = minAreaRect(contours_poly[i]);
				Point2f vertices[4];
				//重新排序矩形坐标点，按左上，右上，右下，左下顺序
				SortRotatedRectPoints(vertices, rRect);

				cout << vertices[0] << vertices[1] << vertices[2] << vertices[3] << endl;

				//根据获得的4个点画线
				for (int k = 0; k < 4; ++k) {
					line(gray, vertices[k], vertices[(k + 1) % 4], Scalar(255, 0, 0));
				}

				//多边形拟合的画出轮廓
				drawContours(gray, contours_poly, i, Scalar(0, 255, 0));

				//计算四边形的四点坐标
				Point2f rPoints[4];
				//GetRectPoints(rPoints, rRect.center, contours_poly[i]);
				GetPointsFromRect(rPoints, vertices, contours_poly[i]);
				for (int k = 0; k < 4; ++k) {
					line(gray, rPoints[k], rPoints[(k + 1) % 4], Scalar(255, 255, 255));
				}


				//采用离最小矩形四个点最近的重新设置范围，将所在区域的点做直线拟合再看看结果
				Point2f newPoints[4];
				GetPointsFromFitline(newPoints, rPoints, vertices);
				for (int k = 0; k < 4; ++k) {
					line(gray, newPoints[k], newPoints[(k + 1) % 4], Scalar(255, 100, 255));
				}


				//根据最小矩形和多边形拟合的最大四个点计算透视变换矩阵		
				Point2f rectPoint[4];
				//计算旋转矩形的宽和高
				float rWidth = CalcPointDistance(vertices[0], vertices[1]);
				float rHeight = CalcPointDistance(vertices[1], vertices[2]);
				//计算透视变换的左上角起始点
				float left = gray.cols;
				float top = gray.rows;
				for (int i = 0; i < 4; ++i) {
					if (left > newPoints[i].x) left = newPoints[i].x;
					if (top > newPoints[i].y) top = newPoints[i].y;
				}

				rectPoint[0] = Point2f(left, top);
				rectPoint[1] = rectPoint[0] + Point2f(rWidth, 0);
				rectPoint[2] = rectPoint[1] + Point2f(0, rHeight);
				rectPoint[3] = rectPoint[0] + Point2f(0, rHeight);
				

				//计算透视变换矩阵		
				Mat warpmatrix = getPerspectiveTransform(rPoints, rectPoint);
				Mat resultimg;
                //透视变换
				warpPerspective(tmpgray, resultimg, warpmatrix, resultimg.size(), INTER_LINEAR);
				imshow("resultimg", resultimg);

			}
		}
	}
	imshow("src2", gray);

	waitKey(0);
	return 0;
}

//根据中心点找最远的四个点
void GetRectPoints(Point2f vetPoints[], Point2f center, vector<Point> convex)
{
	//定义最远的4个点，0--左上， 1--右上， 2--右下  3--左下
	float ltdist = 0.0f;  //左上的最大距离 
	float rtdist = 0.0f;  //右上的最大距离 
	float rbdist = 0.0f;  //右下的最大距离 
	float lbdist = 0.0f;  //左下的最大距离

	for (auto curpoint : convex) {
		//计算点的距离 
		float curdist = CalcPointDistance(center, curpoint);

		if (curpoint.x < center.x && curpoint.y < center.y)
		{
			//判断是否在左上
			if (curdist > ltdist) {
				ltdist = curdist;
				vetPoints[0] = curpoint;
			}
		}
		else if (curpoint.x > center.x && curpoint.y < center.y) {
			//判断在右上
			if (curdist > rtdist) {
				rtdist = curdist;
				vetPoints[1] = curpoint;
			}
		}
		else if (curpoint.x > center.x && curpoint.y > center.y) {
			//判断在右下
			if (curdist > rbdist) {
				rbdist = curdist;
				vetPoints[2] = curpoint;
			}
		}
		else if (curpoint.x < center.x && curpoint.y > center.y) {
			//判断在左下
			if (curdist > lbdist) {
				lbdist = curdist;
				vetPoints[3] = curpoint;
			}
		}

	}
	
}

//根据最小矩形点找最近的四边形点
//第一参数为输出的点，第二个参数为矩形的4个点，第三个为多边形拟合的点 
void GetPointsFromRect(Point2f vetPoints[], Point2f rectPoints[], vector<Point> convex)
{
	//定义最远的4个点，0--左上， 1--右上， 2--右下  3--左下
	float ltdist = 99999999.9f;  //左上的最大距离 
	float rtdist = 99999999.9f;  //右上的最大距离 
	float rbdist = 99999999.9f;  //右下的最大距离 
	float lbdist = 99999999.9f;  //左下的最大距离
	float curdist = 0.0f; //当前点的计算距离
	

	for (auto curpoint : convex) {
		//计算左上点的距离 
		curdist = CalcPointDistance(rectPoints[0], curpoint);
		if (curdist < ltdist) {
			ltdist = curdist;
			vetPoints[0] = curpoint;
		}
		//计算右上角的点距离
		curdist = CalcPointDistance(rectPoints[1], curpoint);
		if (curdist < rtdist) {
			rtdist = curdist;
			vetPoints[1] = curpoint;
		}
		//计算右下角点的距离
		curdist = CalcPointDistance(rectPoints[2], curpoint);
		if (curdist < rbdist) {
			rbdist = curdist;
			vetPoints[2] = curpoint;
		}
		//计算左下角点的距离
		curdist = CalcPointDistance(rectPoints[3], curpoint);
		if (curdist < lbdist) {
			lbdist = curdist;
			vetPoints[3] = curpoint;
		}
	}
}

//重新计算距离变换的4个坐标点
//思路：旋转矩形的点和上一步获取的临近点判断距离，如果小于阈值都列入，大于阈值按最近距离的阈值处理
void GetPointsFromFitline(Point2f newPoints[], Point2f vetPoints[], Point2f rectPoints[], float dist)
{
	//1.重新规划区域点
	float curdist = CalcPointDistance(rectPoints[0], vetPoints[0]);
	newPoints[0] = curdist <= dist ? rectPoints[0] : vetPoints[0] + Point2f(-dist, -dist);

	curdist = CalcPointDistance(rectPoints[1], vetPoints[1]);
	newPoints[1] = curdist <= dist ? rectPoints[1] : vetPoints[1] + Point2f(dist, -dist);

	curdist = CalcPointDistance(rectPoints[2], vetPoints[2]);
	newPoints[2] = curdist <= dist ? rectPoints[2] : vetPoints[2] + Point2f(dist, dist);

	curdist = CalcPointDistance(rectPoints[3], vetPoints[3]);
	newPoints[3] = curdist <= dist ? rectPoints[3] : vetPoints[3] + Point2f(-dist, dist);

    //左侧区域点为最小旋转矩形的左上左下和离的最近的左上左下组成
	vector<Point2f> lArea;
	lArea.push_back(newPoints[0]);
	lArea.push_back(vetPoints[0]);
	lArea.push_back(vetPoints[3]);
	lArea.push_back(newPoints[3]);

	//顶部区域点为最小外接矩形左上右上和最近点的左上右上组成
	vector<Point2f> tArea;
	tArea.push_back(newPoints[0]);
	tArea.push_back(newPoints[1]);
	tArea.push_back(vetPoints[1]);
	tArea.push_back(vetPoints[0]);

	//右侧区域点为最近点的右上右下和最小外接矩形的右上和右下组成
	vector<Point2f> rArea;
	rArea.push_back(vetPoints[1]);
	rArea.push_back(newPoints[1]);
	rArea.push_back(newPoints[2]);
	rArea.push_back(vetPoints[2]);

	//底部区域点为最近点的左下右下和最小外接矩形的左下右下组成
	vector<Point2f> bArea;
	bArea.push_back(vetPoints[3]);
	bArea.push_back(vetPoints[2]);
	bArea.push_back(newPoints[2]);
	bArea.push_back(newPoints[3]);

	//2.根据区域做直线拟合
	//左边区域
	Vec4f lLine;
	fitLine(lArea, lLine, DIST_L2, 0, 0.01, 0.01);
	//顶部区域
	Vec4f tLine;
	fitLine(tArea, tLine, DIST_L2, 0, 0.01, 0.01);
	//右边区域
	Vec4f rLine;
	fitLine(rArea, rLine, DIST_L2, 0, 0.01, 0.01);
	//顶部区域
	Vec4f bLine;
	fitLine(bArea, bLine, DIST_L2, 0, 0.01, 0.01);

	//3.根据直线拟合的求每两条直线的交叉点为我们的多边形顶点
    //左上顶点
	newPoints[0] = GetCrossPoint(lLine, tLine);
	//右上顶点
	newPoints[1] = GetCrossPoint(tLine, rLine);
	//右下顶点
	newPoints[2] = GetCrossPoint(rLine, bLine);
	//左下顶点
	newPoints[3] = GetCrossPoint(bLine, lLine);
}

//求两条直线的交叉点
Point2f GetCrossPoint(Vec4f Line1, Vec4f Line2)
{
	double k1, k2;
	//Line1的斜率
	k1 = Line1[1] / Line1[0];
	//Line2的斜率
	k2 = Line2[1] / Line2[0];

	Point2f crossPoint;
	crossPoint.x = (k1 * Line1[2] - Line1[3] - k2 * Line2[2] + Line2[3]) / (k1 - k2);
	crossPoint.y = (k1 * k2 * (Line1[2] - Line2[2]) + k1 * Line2[3] - k2 * Line1[3]) / (k1 - k2);
	return crossPoint;
}

//重新排序旋转矩形坐标点
void SortRotatedRectPoints(Point2f vetPoints[], RotatedRect rect)
{
	rect.points(vetPoints);

	cout << vetPoints[0] << vetPoints[1] << vetPoints[2] << vetPoints[3] << endl;
	cout << rect.angle << endl;

	Point2f curpoint;
	//根据Rect的坐标点，Y轴最大的为P[0]，p[0]围着center顺时针旋转, 
	//旋转角度为负的话即是P[0]在左下角，为正P[0]是右下角
    //重新排序坐标点
	if (rect.angle > 0) {
		curpoint = vetPoints[0];
		vetPoints[0] = vetPoints[2];
		vetPoints[2] = curpoint;
		curpoint = vetPoints[1];
		vetPoints[1] = vetPoints[3];
		vetPoints[3] = curpoint;
	}
	else if (rect.angle < 0) {
		curpoint = vetPoints[0];
		vetPoints[0] = vetPoints[1];
		vetPoints[1] = vetPoints[2];
		vetPoints[2] = vetPoints[3];
		vetPoints[3] = curpoint;
	}

}

//计算两点间的距离
float CalcPointDistance(Point2f point1, Point2f point2)
{
	//计算两个点的Point差值
	Point2f tmppoint = point1 - point2;
	//利用欧几里德距离计算H
	return sqrt(pow(tmppoint.x, 2) + pow(tmppoint.y, 2));
}
