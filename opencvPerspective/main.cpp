#include<iostream>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

//�������ĵ����Ľ���Զ�ĵ�
void GetRectPoints(Point2f vetPoints[], Point2f center, vector<Point> convex);
//������С���ε���������ı��ε�
void GetPointsFromRect(Point2f vetPoints[], Point2f rectPoints[], vector<Point> convex);

//�������¶������ֱ����Ϻ��ҵ��Ķ�Ӧ��
void GetPointsFromFitline(Point2f newPoints[], Point2f vetPoints[], Point2f rectPoints[], float dist = 15.0f);

//������ֱ�ߵĽ���
Point2f GetCrossPoint(Vec4f Line1, Vec4f Line2);

//������ת���������
void SortRotatedRectPoints(Point2f vetPoints[], RotatedRect rect);
//�������
float CalcPointDistance(Point2f point1, Point2f point2);



int main(int argc, char** argv) {

	Mat src = imread("E:/DCIM/tsnew.jpg");
	Mat gray, dst, dst2, result;
	//ͼ������
	resize(src, gray, Size(0, 0), 0.2, 0.2);
	imshow("src", gray);

	//�Ҷ�ͼ
	cvtColor(gray, dst, COLOR_BGRA2GRAY);

	//��˹�˲�
	GaussianBlur(dst, dst, Size(5, 5), 0.5, 0.5);
	//imshow("gray", dst);

	//��ֵ��
	threshold(dst, dst2, 0, 255, THRESH_BINARY | THRESH_OTSU);
	//imshow("thresh", dst2);

	//��̬ѧ������
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5), Point(-1, -1));
	morphologyEx(dst2, dst2, MORPH_OPEN, kernel);
	//imshow("morph", dst2);

	//Canny��Ե���
	Canny(dst2, result, 127, 255, 7, true);
	//imshow("canny", result);

	//��������
	vector<vector<Point>> contours;
	findContours(result, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

	cout << contours.size() << endl;
	vector<vector<Point>> contours_poly(contours.size());
	Mat tmpgray;
	gray.copyTo(tmpgray);
	for (int i = 0; i < contours.size(); ++i) {
		//���������ܳ�������ͼ���ȵĲ�������
		double dlen = arcLength(contours[i], true);
		if (dlen > gray.cols) {
			//��������
			approxPolyDP(Mat(contours[i]), contours_poly[i], 10, true);

			cout << "��ǰ��" << i << " ������" << contours_poly[i].size() << endl;
			
			if (contours_poly[i].size() >= 4) {
				//��ȡ��С��ת����
				RotatedRect rRect = minAreaRect(contours_poly[i]);
				Point2f vertices[4];
				//���������������㣬�����ϣ����ϣ����£�����˳��
				SortRotatedRectPoints(vertices, rRect);

				cout << vertices[0] << vertices[1] << vertices[2] << vertices[3] << endl;

				//���ݻ�õ�4���㻭��
				for (int k = 0; k < 4; ++k) {
					line(gray, vertices[k], vertices[(k + 1) % 4], Scalar(255, 0, 0));
				}

				//�������ϵĻ�������
				drawContours(gray, contours_poly, i, Scalar(0, 255, 0));

				//�����ı��ε��ĵ�����
				Point2f rPoints[4];
				//GetRectPoints(rPoints, rRect.center, contours_poly[i]);
				GetPointsFromRect(rPoints, vertices, contours_poly[i]);
				for (int k = 0; k < 4; ++k) {
					line(gray, rPoints[k], rPoints[(k + 1) % 4], Scalar(255, 255, 255));
				}


				//��������С�����ĸ���������������÷�Χ������������ĵ���ֱ������ٿ������
				Point2f newPoints[4];
				GetPointsFromFitline(newPoints, rPoints, vertices);
				for (int k = 0; k < 4; ++k) {
					line(gray, newPoints[k], newPoints[(k + 1) % 4], Scalar(255, 100, 255));
				}


				//������С���κͶ������ϵ�����ĸ������͸�ӱ任����		
				Point2f rectPoint[4];
				//������ת���εĿ�͸�
				float rWidth = CalcPointDistance(vertices[0], vertices[1]);
				float rHeight = CalcPointDistance(vertices[1], vertices[2]);
				//����͸�ӱ任�����Ͻ���ʼ��
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
				

				//����͸�ӱ任����		
				Mat warpmatrix = getPerspectiveTransform(rPoints, rectPoint);
				Mat resultimg;
                //͸�ӱ任
				warpPerspective(tmpgray, resultimg, warpmatrix, resultimg.size(), INTER_LINEAR);
				imshow("resultimg", resultimg);

			}
		}
	}
	imshow("src2", gray);

	waitKey(0);
	return 0;
}

//�������ĵ�����Զ���ĸ���
void GetRectPoints(Point2f vetPoints[], Point2f center, vector<Point> convex)
{
	//������Զ��4���㣬0--���ϣ� 1--���ϣ� 2--����  3--����
	float ltdist = 0.0f;  //���ϵ������� 
	float rtdist = 0.0f;  //���ϵ������� 
	float rbdist = 0.0f;  //���µ������� 
	float lbdist = 0.0f;  //���µ�������

	for (auto curpoint : convex) {
		//�����ľ��� 
		float curdist = CalcPointDistance(center, curpoint);

		if (curpoint.x < center.x && curpoint.y < center.y)
		{
			//�ж��Ƿ�������
			if (curdist > ltdist) {
				ltdist = curdist;
				vetPoints[0] = curpoint;
			}
		}
		else if (curpoint.x > center.x && curpoint.y < center.y) {
			//�ж�������
			if (curdist > rtdist) {
				rtdist = curdist;
				vetPoints[1] = curpoint;
			}
		}
		else if (curpoint.x > center.x && curpoint.y > center.y) {
			//�ж�������
			if (curdist > rbdist) {
				rbdist = curdist;
				vetPoints[2] = curpoint;
			}
		}
		else if (curpoint.x < center.x && curpoint.y > center.y) {
			//�ж�������
			if (curdist > lbdist) {
				lbdist = curdist;
				vetPoints[3] = curpoint;
			}
		}

	}
	
}

//������С���ε���������ı��ε�
//��һ����Ϊ����ĵ㣬�ڶ�������Ϊ���ε�4���㣬������Ϊ�������ϵĵ� 
void GetPointsFromRect(Point2f vetPoints[], Point2f rectPoints[], vector<Point> convex)
{
	//������Զ��4���㣬0--���ϣ� 1--���ϣ� 2--����  3--����
	float ltdist = 99999999.9f;  //���ϵ������� 
	float rtdist = 99999999.9f;  //���ϵ������� 
	float rbdist = 99999999.9f;  //���µ������� 
	float lbdist = 99999999.9f;  //���µ�������
	float curdist = 0.0f; //��ǰ��ļ������
	

	for (auto curpoint : convex) {
		//�������ϵ�ľ��� 
		curdist = CalcPointDistance(rectPoints[0], curpoint);
		if (curdist < ltdist) {
			ltdist = curdist;
			vetPoints[0] = curpoint;
		}
		//�������Ͻǵĵ����
		curdist = CalcPointDistance(rectPoints[1], curpoint);
		if (curdist < rtdist) {
			rtdist = curdist;
			vetPoints[1] = curpoint;
		}
		//�������½ǵ�ľ���
		curdist = CalcPointDistance(rectPoints[2], curpoint);
		if (curdist < rbdist) {
			rbdist = curdist;
			vetPoints[2] = curpoint;
		}
		//�������½ǵ�ľ���
		curdist = CalcPointDistance(rectPoints[3], curpoint);
		if (curdist < lbdist) {
			lbdist = curdist;
			vetPoints[3] = curpoint;
		}
	}
}

//���¼������任��4�������
//˼·����ת���εĵ����һ����ȡ���ٽ����жϾ��룬���С����ֵ�����룬������ֵ������������ֵ����
void GetPointsFromFitline(Point2f newPoints[], Point2f vetPoints[], Point2f rectPoints[], float dist)
{
	//1.���¹滮�����
	float curdist = CalcPointDistance(rectPoints[0], vetPoints[0]);
	newPoints[0] = curdist <= dist ? rectPoints[0] : vetPoints[0] + Point2f(-dist, -dist);

	curdist = CalcPointDistance(rectPoints[1], vetPoints[1]);
	newPoints[1] = curdist <= dist ? rectPoints[1] : vetPoints[1] + Point2f(dist, -dist);

	curdist = CalcPointDistance(rectPoints[2], vetPoints[2]);
	newPoints[2] = curdist <= dist ? rectPoints[2] : vetPoints[2] + Point2f(dist, dist);

	curdist = CalcPointDistance(rectPoints[3], vetPoints[3]);
	newPoints[3] = curdist <= dist ? rectPoints[3] : vetPoints[3] + Point2f(-dist, dist);

    //��������Ϊ��С��ת���ε��������º��������������������
	vector<Point2f> lArea;
	lArea.push_back(newPoints[0]);
	lArea.push_back(vetPoints[0]);
	lArea.push_back(vetPoints[3]);
	lArea.push_back(newPoints[3]);

	//���������Ϊ��С��Ӿ����������Ϻ������������������
	vector<Point2f> tArea;
	tArea.push_back(newPoints[0]);
	tArea.push_back(newPoints[1]);
	tArea.push_back(vetPoints[1]);
	tArea.push_back(vetPoints[0]);

	//�Ҳ������Ϊ�������������º���С��Ӿ��ε����Ϻ��������
	vector<Point2f> rArea;
	rArea.push_back(vetPoints[1]);
	rArea.push_back(newPoints[1]);
	rArea.push_back(newPoints[2]);
	rArea.push_back(vetPoints[2]);

	//�ײ������Ϊ�������������º���С��Ӿ��ε������������
	vector<Point2f> bArea;
	bArea.push_back(vetPoints[3]);
	bArea.push_back(vetPoints[2]);
	bArea.push_back(newPoints[2]);
	bArea.push_back(newPoints[3]);

	//2.����������ֱ�����
	//�������
	Vec4f lLine;
	fitLine(lArea, lLine, DIST_L2, 0, 0.01, 0.01);
	//��������
	Vec4f tLine;
	fitLine(tArea, tLine, DIST_L2, 0, 0.01, 0.01);
	//�ұ�����
	Vec4f rLine;
	fitLine(rArea, rLine, DIST_L2, 0, 0.01, 0.01);
	//��������
	Vec4f bLine;
	fitLine(bArea, bLine, DIST_L2, 0, 0.01, 0.01);

	//3.����ֱ����ϵ���ÿ����ֱ�ߵĽ����Ϊ���ǵĶ���ζ���
    //���϶���
	newPoints[0] = GetCrossPoint(lLine, tLine);
	//���϶���
	newPoints[1] = GetCrossPoint(tLine, rLine);
	//���¶���
	newPoints[2] = GetCrossPoint(rLine, bLine);
	//���¶���
	newPoints[3] = GetCrossPoint(bLine, lLine);
}

//������ֱ�ߵĽ����
Point2f GetCrossPoint(Vec4f Line1, Vec4f Line2)
{
	double k1, k2;
	//Line1��б��
	k1 = Line1[1] / Line1[0];
	//Line2��б��
	k2 = Line2[1] / Line2[0];

	Point2f crossPoint;
	crossPoint.x = (k1 * Line1[2] - Line1[3] - k2 * Line2[2] + Line2[3]) / (k1 - k2);
	crossPoint.y = (k1 * k2 * (Line1[2] - Line2[2]) + k1 * Line2[3] - k2 * Line1[3]) / (k1 - k2);
	return crossPoint;
}

//����������ת���������
void SortRotatedRectPoints(Point2f vetPoints[], RotatedRect rect)
{
	rect.points(vetPoints);

	cout << vetPoints[0] << vetPoints[1] << vetPoints[2] << vetPoints[3] << endl;
	cout << rect.angle << endl;

	Point2f curpoint;
	//����Rect������㣬Y������ΪP[0]��p[0]Χ��center˳ʱ����ת, 
	//��ת�Ƕ�Ϊ���Ļ�����P[0]�����½ǣ�Ϊ��P[0]�����½�
    //�������������
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

//���������ľ���
float CalcPointDistance(Point2f point1, Point2f point2)
{
	//�����������Point��ֵ
	Point2f tmppoint = point1 - point2;
	//����ŷ����¾������H
	return sqrt(pow(tmppoint.x, 2) + pow(tmppoint.y, 2));
}
