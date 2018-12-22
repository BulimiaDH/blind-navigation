#include <iostream>
#include <math.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace xfeatures2d;

int intensity(Mat img1, Mat img2, int dx, int dy,float fx) {
	assert(dx > 0 && dy > 0 && fx > 0);
	cvtColor(img1, img1, CV_RGB2GRAY);
	cvtColor(img2, img2, CV_RGB2GRAY);

	const int w = img1.cols;
	const int h = img1.rows;

	vector<pair<float, float>> intensities;
	float xx, yy;
	float x0, y0, x1, y1;
	int flag = 0;
	for(float angle = -40; angle <= 40; angle++) {
		float intensity = 0;
		int cnt = 0;
		Mat trans(h, w, CV_8UC1, Scalar::all(0));
		Mat mask(h, w, CV_8UC1, Scalar::all(0));
		for(int x = 0; x < w; x += dx) {
			for(int y = 0; y < h; y += dy) {
				xx = (x - w/2.0f)/fx;
				yy = (y - h/2.0f)/fx;
				yy = yy/(-sin(angle / 180 * M_PI) * xx + cos(angle / 180 * M_PI));
				xx = (cos(angle / 180 * M_PI) * xx + sin(angle / 180 * M_PI)) / (-sin(angle / 180 * M_PI) * xx + cos(angle / 180 * M_PI));
				xx = xx * fx + w / 2.0f;
				yy = yy * fx + h / 2.0f;
				
				x0 = floor(xx);
				x1 = ceil(xx);
				y0 = floor(yy);
				y1 = ceil(yy);

				if(xx >= 0 && xx < w && yy >= 0 && yy < h) {
					float p00 = img1.at<unsigned char>(y0, x0);
				       	float p01 = img1.at<unsigned char>(y1, x0);
					float p10 = img1.at<unsigned char>(y0, x1);
					float p11 = img1.at<unsigned char>(y1, x1);
					float p0 = p00 + (p01 - p00) * (yy - y0);
					float p1 = p10 + (p11 - p10) * (yy - y0);
					unsigned char p = p0 + (p1 - p0) * (xx - x0) + 0.5;
				
					trans.at<unsigned char>(y, x) = p;
					mask.at<unsigned char>(y, x) = 1;
				}
			}
		}
		for(int x = 0; x < w; x++) {
			for(int y = 0; y < h; y++) {
				if(mask.at<unsigned char>(y, x) == 1) {
					intensity += abs(trans.at<unsigned char>(y, x) - img2.at<unsigned char>(y, x));
					cnt++;
				}
			}
		}
		intensities.push_back(make_pair(intensity / cnt, angle));
	}
	sort(intensities.begin(), intensities.end());
	return intensities[0].second;
}

int main() {
        Mat img1 = imread("/home/blindfind/Documents/Blindfind3/testDataset/Test/test2/track2/t1139101167.256761_rBO_x4.68387_y22.329_a0.159288.png");
        Mat img2 = imread("/home/blindfind/Documents/Blindfind3/testDataset/Test/test2/track2/t1139101167.056802_rBO_x4.57558_y22.3224_a0.233685.png");
		int angle = intensity(img1, img2, 1, 1, 386.14);
	cout << angle << endl;
	return 0;
}
