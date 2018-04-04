//*********************************************************************************************************
// Author: Yuhua Xu, 2018, 03, 30. xyh_nudt@163.com
// Reference image based phase unwrapping framework for a structured light system.
//*********************************************************************************************************
#include "stdafx.h"
#include <Windows.h>
#include "opencv2\opencv.hpp"
#include "opencv2\core\core.hpp"
#include <omp.h>
#include <iostream>
#include <fstream>
#include "StereoMatchOneShotAPI.h"

using namespace std;

#pragma comment (lib, "opencv_core2413.lib")
#pragma comment (lib, "opencv_highgui2413.lib")
//#pragma comment (lib, "opencv_ml2413.lib")
#pragma comment (lib, "opencv_calib3d2413.lib")
#pragma comment (lib, "opencv_imgproc2413.lib")

float* retPhiRef = NULL;

// get CPU number
int GetCpuNum()
{
	SYSTEM_INFO si;
	GetSystemInfo(&si);
	int count = si.dwNumberOfProcessors;
	return count;
}

void ReadTxtData32F(char* pPath, vector<float> &data, int &row, int &col)
{
	data.clear();

	//float F, B, cx, cy, cxR;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;

	while (getline(fin, line)) // 
	{
		bool isAnnotationLine = false;

		int comma_n = 0;
		int comma_n2 = 0;
		//vector<string> str_list;
		vector<double> db_list;
		do
		{
			string temp_s = "";
			comma_n = line.find(" ");
			//comma_n2 = line.find(";");
			if (-1 == comma_n)
			{
				temp_s = line.substr(0, line.length());
				//str_list.push_back(temp_s);
				if (temp_s.size()>0)
					db_list.push_back(atof(temp_s.c_str())); // string to double

				break;
			}
			temp_s = line.substr(0, comma_n);
			if (temp_s == "#")
			{
				isAnnotationLine = true;
				break;
			}
			line.erase(0, comma_n + 1);
			//str_list.push_back(temp_s);
			if (temp_s.size()>0)
				db_list.push_back(atof(temp_s.c_str()));

		} while (comma_n >= 0);

		if (isAnnotationLine == true) // 
		{
			continue;
		}

		if (nLines == 0)
		{
			col = db_list.size();
		}
		if (db_list.size() <= 0)
		{
			continue;
		}

		for (int i = 0; i<db_list.size(); i++)
		{
			data.push_back(db_list[i]);
		}

		if (db_list.size()>0)
		{
			nLines++;
		}
	}

	row = nLines;

	return;
}

void OutData2Txt(float *pData2D, int w, int h, int ws, char* filename)
{
	if (pData2D == 0)
		return;

	std::cout << "output 2d data to txt file" << endl;
	ofstream outm(filename, ios::trunc);

	for (int u = 0; u<h; u++)
	{
		float* pDataRow = pData2D + u*ws;
		for (int v = 0; v<w; v++)
		{
			outm << pDataRow[v] << " ";
		}
		outm << endl;
	}
	outm.close();
}

// smoothing 1D
int GaussSmoothData1D(float* srcData, float* dstData, int N, float sigma, int winR)
{
	if ((srcData == NULL) || (dstData == NULL))
	{
		return -1;
	}

	int winSize = winR * 2 + 1;

	//CMemoryPool memPool;

	const float MIN_SIGMA = 0.001f; // 
	if (sigma < MIN_SIGMA)
		sigma = MIN_SIGMA;

	int w = N, h = 1, ws = N;

	// cal template size
	const int MAX_WIN_SIZE = 201;
	if (winSize >= MAX_WIN_SIZE)
		winSize = MAX_WIN_SIZE;

	float fInvSigma = 1 / sigma;

	float aGray[MAX_WIN_SIZE] = { 0 }; // 

	// cal template coefficients
	float aTemplate[MAX_WIN_SIZE] = { 0 };
	float fAcc = 0;
	for (int i = -winR; i <= winR; i++)
	{
		aTemplate[i + winR] = exp(-i*i*0.5f*fInvSigma*fInvSigma) * fInvSigma;
		fAcc += aTemplate[i + winR];
	}

	// normalization
	float fInvAcc = 1 / fAcc;
	for (int i = 0; i < winSize; i++)
	{
		//aTemplate[i] /= fAcc;
		aTemplate[i] *= fInvAcc;
	}

	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	memset(dstData, 0, w*h * sizeof(float));

	int w_winR = w - winR;
	int h_winR = h - winR;

	for (int j = 0; j < w; j++)
	{
		float fAccGray = 0;
		for (int m = -winR; m <= winR; m++)
		{
			int x = j + m;
			if (x < 0)
				x = 0;
			else if (x > w_1)
				x = w_1;

			fAccGray += aTemplate[m + winR] * srcData[x];
		}

		dstData[j] = fAccGray;
	}

	return 0;
}

// convert absolute phase map to 3d point cloud
int AbsolutePhase2PointCloud(float *absPhi, int width, int height, std::vector<CvPoint3D32f> &points, 
	    cv::Mat camK, cv::Mat R_cam, cv::Mat T_cam, 
	    cv::Mat projK, cv::Mat R_proj, cv::Mat T_proj, float nFringes,
	    const std::vector<float> &phiLUT)
{
	points.clear();
	float pi = 3.1415926;

	cv::Mat Pp = cv::Mat::zeros(3, 4, CV_64F);
	//cv::Mat Pc = cv::Mat::zeros(3, 4, CV_64F);
	cv::Mat RTp = cv::Mat::zeros(3, 4, CV_64F);
	cv::Mat RTc = cv::Mat::zeros(3, 4, CV_64F);
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			RTc.at<double>(i, j) = R_cam.at<double>(i, j);
			RTp.at<double>(i, j) = R_proj.at<double>(i, j);
		}
		RTc.at<double>(i, 3) = T_cam.at<double>(i, 0);
		RTp.at<double>(i, 3) = T_proj.at<double>(i, 0);
	}
	std::cout << "R_cam " << R_cam << endl;
	std::cout << "T_cam " << T_cam << endl;
	std::cout << "camK " << camK << endl;
	std::cout << "RTc " << RTc << endl;

	cv::Mat Pc = camK*RTc;
	Pp = projK*RTp;

	cout<<endl;
	std::cout << "R_proj " << R_proj << endl;
	std::cout << "T_proj " << T_proj << endl;
	std::cout << "projK " << projK << endl;
	std::cout << "RTp " << RTp << endl;

	std::cout <<"Pc "<< Pc << endl;
	std::cout << "Pp " << Pp << endl;

	cv::Mat M33 = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat M31 = cv::Mat::zeros(3, 1, CV_64F);

	int wp = 1024;
	for (int r = 0; r < height; r++)
	{
		for (int c = 0; c < width; c++)
		{
			float phi = absPhi[r*width + c];
			if (phi <= 0)
				continue;

			float step = 0.02f;
			// phase gamma correction
			int k = phi / (2*pi);
			float phi2 = phi - k * 2 * pi;
			int iLut = int(phi2 / step);
			float compensation = 0;
			if (!phiLUT.empty())
			{
				compensation = phiLUT[iLut];
			}

			phi += compensation;

			// convert phi to xp
			float xp = phi*wp / (2 * pi*nFringes);
			float xc = c;
			float yc = r;

			double p11 = Pc.at<double>(0, 0);
			double p12 = Pc.at<double>(0, 1);
			double p13 = Pc.at<double>(0, 2);
			double p14 = Pc.at<double>(0, 3);

			double p21 = Pc.at<double>(1, 0);
			double p22 = Pc.at<double>(1, 1);
			double p23 = Pc.at<double>(1, 2);
			double p24 = Pc.at<double>(1, 3);

			double p31 = Pc.at<double>(2, 0);
			double p32 = Pc.at<double>(2, 1);
			double p33 = Pc.at<double>(2, 2);
			double p34 = Pc.at<double>(2, 3);

			double q11 = Pp.at<double>(0, 0);
			double q12 = Pp.at<double>(0, 1);
			double q13 = Pp.at<double>(0, 2);
			double q14 = Pp.at<double>(0, 3);

			double q21 = Pp.at<double>(1, 0);
			double q22 = Pp.at<double>(1, 1);
			double q23 = Pp.at<double>(1, 2);
			double q24 = Pp.at<double>(1, 3);

			double q31 = Pp.at<double>(2, 0);
			double q32 = Pp.at<double>(2, 1);
			double q33 = Pp.at<double>(2, 2);
			double q34 = Pp.at<double>(2, 3);

			M33.at<double>(0, 0) = p11 - xc*p31;
			M33.at<double>(0, 1) = p12 - xc*p32;
			M33.at<double>(0, 2) = p13 - xc*p33;

			M33.at<double>(1, 0) = p21 - yc*p31;
			M33.at<double>(1, 1) = p22 - yc*p32;
			M33.at<double>(1, 2) = p23 - yc*p33;

			M33.at<double>(2, 0) = q11 - xp*q31;
			M33.at<double>(2, 1) = q12 - xp*q32;
			M33.at<double>(2, 2) = q13 - xp*q33;

			M31.at<double>(0, 0) = Pc.at<double>(2, 3)*xc - Pc.at<double>(0, 3);
			M31.at<double>(1, 0) = Pc.at<double>(2, 3)*yc - Pc.at<double>(1, 3);
			M31.at<double>(2, 0) = Pp.at<double>(2, 3)*xp - Pp.at<double>(0, 3);

			cv::Mat pt = M33.inv()*M31;
			CvPoint3D32f temp;
			temp.x = pt.at<double>(0, 0);
			temp.y = pt.at<double>(1, 0);
			temp.z = pt.at<double>(2, 0);
			if (temp.z < 0)
			{
				temp.x *= -1;
				temp.y *= -1;
				temp.z *= -1;
			}

			points.push_back(temp);
		}
	}

	return 0;
}

// read yml file ("test.yml")
int ReadCalibParams(char* filePath, cv::Mat &cam_K, cv::Mat &cam_kc,
	                 cv::Mat &proj_K, cv::Mat &proj_kc, 
	                 cv::Mat &R, cv::Mat &T)
{
	cv::FileStorage fs2(filePath, cv::FileStorage::READ);

	// first method: use (type) operator on FileNode.
	int frameCount = (int)fs2["frameCount"];

	//std::string date;
	//fs2["calibrationDate"] >> date;

	//cv::Mat cam_K, cam_kc;
	fs2["cam_K"] >> cam_K;
	fs2["cam_kc"] >> cam_kc;

	//cv::Mat proj_K, proj_kc;
	fs2["proj_K"] >> proj_K;
	fs2["proj_kc"] >> proj_kc;

	//cv::Mat R, T;
	fs2["R"] >> R;
	fs2["T"] >> T;

	cout << "cam_K: " << cam_K << endl
		<< "cam_kc: " << cam_kc << endl
		<< "proj_K: " << proj_K << endl
		<< "proj_kc: " << proj_kc << endl
		<< "R: " << R << endl
		<< "T: " << T << endl<<endl;

	fs2.release();

	return 0;
}

// unwrapping for a planar target using a binary pattern
// nFringe----the number of fringes
int UnwrappingPhase(IplImage* bina, float *rtmPhi, float *absPhi, int nFringes)
{
	int w = bina->width;
	int h = bina->height;

	int w2 = w / 4;

	float pi = 3.1415926f;

	for(int r=0; r<h; r++)
	{ 
		unsigned char* binaRow = (unsigned char*)bina->imageData + r*w;
		float *rtmPhiRow = rtmPhi + r*w;
		float *absPhiRow = absPhi + r*w;

		// 查找中间点(相位应该为0)
		for (int c = w2; c < w-w2; c++)
		{
			if (rtmPhiRow[c]<=-4)
			{
				continue;
			}
			if (binaRow[c] - binaRow[c - 1] > 0)
			{
				r = r;
				int k = 0;
				
				// 处理右边
				k = nFringes / 2;
				for (int c2=c; c2<w; c2++)
				{
					if (rtmPhiRow[c2] <= -4)
					{
						continue;
					}
					if ((rtmPhiRow[c2]-rtmPhiRow[c2-1])<-pi)
					{
						k += 1;
					}
					absPhiRow[c2] = k * 2 * pi + rtmPhiRow[c2];
				}

				// 处理左边
				k = nFringes / 2;
				for (int c2 = c; c2>=0; c2--)
				{
					if (rtmPhiRow[c2] <= -4)
					{
						continue;
					}
					if ((rtmPhiRow[c2] - rtmPhiRow[c2 + 1])>pi)
					{
						k -= 1;
					}
					absPhiRow[c2] = k * 2 * pi + rtmPhiRow[c2];

					//kMap[r*w + c2] = k;
				}

				//kMap[r*w + c] = 1;
				break;
			}
		}
	}

	//OutData2Txt(kMap, w, h, w, "d:/kMap.txt");

	//delete[] kMap;

	return 0;
}

// Wrap phase shift
int WrapPhaseShift(IplImage** src0, int nImages, float* phi, int diffT)
{
	if (nImages != 3)
		return -1;

	//cvSaveImage("d:/phase0.bmp", src0[0]);
	//cvSaveImage("d:/phase1.bmp", src0[1]);
	//cvSaveImage("d:/phase2.bmp", src0[2]);

	int w = src0[0]->width;
	int h = src0[0]->height;

	memset(phi, 0, w*h * sizeof(float));

	// 转灰度图
	IplImage* gray[100] = { NULL };
	for (int n = 0; n<nImages; n++)
	{
		gray[n] = cvCreateImage(cvSize(w, h), 8, 1);
		if (src0[n]->nChannels == 1)
		{
			cvCopy(src0[n], gray[n]);
		}
		else
		{
			cvCvtColor(src0[n], gray[n], CV_BGR2GRAY);
		}
	}

	//cvSaveImage("d:/phase20.bmp", gray[0]);
	//cvSaveImage("d:/phase21.bmp", gray[1]);
	//cvSaveImage("d:/phase22.bmp", gray[2]);

	int ws8 = src0[0]->widthStep;

	float sqrt3 = sqrt(3.0f);

	int cpuNum = std::max(GetCpuNum() - 1, 1);
	omp_set_num_threads(cpuNum);
	cout << "cpu number " << cpuNum << endl;
#pragma omp parallel for 
	for (int r = 0; r<h; r++)
	{
		unsigned char* row1 = (unsigned char*)gray[0]->imageData + r*ws8;
		unsigned char* row2 = (unsigned char*)gray[1]->imageData + r*ws8;
		unsigned char* row3 = (unsigned char*)gray[2]->imageData + r*ws8;

		for (int c = 0; c<w; c++)
		{
			float I1 = (float)row1[c];
			float I2 = (float)row2[c];
			float I3 = (float)row3[c];

			float maxI = std::max(I1, I2);
			maxI = std::max(maxI, I3);

			float I1_I2 = abs(I1 - I2);
			float I2_I3 = abs(I2 - I3);
			float I1_I3 = abs(I1 - I3);
			if ((I1_I2<diffT) && (I2_I3<diffT) && (I1_I3<diffT))
			{
				*(phi + r*w + c) = -4;
				continue;
			}

			float phiVal = atan2(sqrt3*(I1 - I3), (2 * I2 - I1 - I3)); // [-pi, pi]

			if (phiVal<0)
				phiVal += 2 * 3.1415926f;

			*(phi + r*w + c) = phiVal; // [0, 2*pi]
		}
	}

	for (int i = 0; i<nImages; i++)
	{
		cvReleaseImage(&gray[i]);
	}

	return 0;
}

int Gray2Bina(const char grayCode[], char binaCode[], int N)
{
	binaCode[0] = grayCode[0];
	for (int i = 1; i<N; i++)
	{
		if (binaCode[i - 1] != grayCode[i])
			binaCode[i] = 1;
		else
			binaCode[i] = 0;
	}

	return 0;
}

int Bina2Gray(const char binaCode[], char grayCode[], int N)
{
	grayCode[0] = binaCode[0];
	for (int i = 1; i<N; i++)
	{
		if (binaCode[i - 1] != binaCode[i])
			grayCode[i] = 1;
		else
			grayCode[i] = 0;
	}

	return 0;
}

int Bina2Decimal(char binaCode[], int length, float &decVal)
{
	decVal = 0;

	for (int i = 0; i<length; i++)
	{
		if (binaCode[i]>0)
			decVal += pow(2.0f, length - 1 - i);
	}

	return 0;
}

int DecodeGray(IplImage** src0, int nImages, int diffT, float *column, int wProject)
{
	if (src0 == NULL)
		return -1;
	int w, h, ws8;
	w = src0[0]->width;
	h = src0[0]->height;

	IplImage* validMap = cvCreateImage(cvSize(w, h), 8, 1);
	cvZero(validMap);

	const int NaN = 128;

	IplImage* gray[100] = { NULL };
	for (int n = 0; n<nImages; n++)
	{
		gray[n] = cvCreateImage(cvSize(w, h), 8, 1);
		if (src0[n]->nChannels == 1)
		{
			cvCopy(src0[n], gray[n]);
		}
		else
		{
			cvCvtColor(src0[n], gray[n], CV_BGR2GRAY);
		}
	}

	ws8 = gray[0]->widthStep;

	// Segmentation
	IplImage* bina[100 / 2] = { NULL };

	for (int n = 0; n<nImages / 2; n++)
	{
		bina[n] = cvCreateImage(cvSize(w, h), 8, 1);
		cvZero(bina[n]);
	}

	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			for (int n = 0; n<nImages / 2; n++)
			{
				//bina[n] = cvCreateImage(cvSize(w,h), 8, 1);

				int n1 = 2 * n;
				int n2 = 2 * n + 1;

				unsigned char* pRow1 = (unsigned char*)gray[n1]->imageData + r*ws8;
				unsigned char* pRow2 = (unsigned char*)gray[n2]->imageData + r*ws8;
				unsigned char* pRowB = (unsigned char*)bina[n]->imageData + r*ws8;

				int diff = pRow1[c] - pRow2[c];
				if (diff >= 0)
				{
					pRowB[c] = 255;
				}
				else if (diff<0)
				{
					pRowB[c] = 0;
				}
				else
				{
					pRowB[c] = NaN;
				}

				if (abs(diff)>diffT)
				{
					*(validMap->imageData + r*ws8 + c) = 255;
				}
			}
		}
	}

	int length = int(log((double)wProject) / log(2.0) + 0.1);
	char grayCode[20] = { 0 }, binaCode[20] = { 0 };
	memset(grayCode, 1, sizeof(grayCode));
	memset(binaCode, 1, sizeof(binaCode));

	// Decode
	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			bool valid = true;
			for (int n = 0; n<nImages / 2; n++)
			{
				int val = *((unsigned char*)bina[n]->imageData + r*ws8 + c);
				if (val == NaN)
				{
					valid = false;
					break;
				}
				else if (val == 255)
					grayCode[n] = 1;
				else if (val == 0)
					grayCode[n] = 0;
			}
			if (*((unsigned char*)validMap->imageData + r*ws8 + c) == 0)
			{
				valid = false;
			}

			if (valid)
			{
				Gray2Bina(grayCode, binaCode, length);

				//memcpy(binaCode, grayCode, sizeof(grayCode)); // 直接用binaCode

				float decVal = 0;
				Bina2Decimal(binaCode, length, decVal);
				// 
				*(column + r*w + c) = decVal;
			}
			else
			{
				*(column + r*w + c) = -1;
			}
		}
	}

	//SmoothColumn(column, w, h, 7);

	for (int i = 0; i<nImages; i++)
	{
		cvReleaseImage(&gray[i]);
	}

	for (int n = 0; n<nImages / 2; n++)
	{
		cvReleaseImage(&bina[n]);
	}

	cvReleaseImage(&validMap);

	return 0;
}

// src0---half black / half white
// src1---half white / half black
int SegBW(IplImage* src0, IplImage* src1, IplImage* bina)
{
	if (src0 == NULL)
		return -1;

	int w, h, ws8;
	w = src0->width;
	h = src0->height;
	ws8 = src0->widthStep;

	for (int r = 0; r < h; r++)
	{
		for (int c = 0; c < w; c++)
		{
			unsigned char* pRow1 = (unsigned char*)src0->imageData + r*ws8;
			unsigned char* pRow2 = (unsigned char*)src1->imageData + r*ws8;
			unsigned char* pRowB = (unsigned char*)bina->imageData + r*ws8;

			int diff = pRow1[c] - pRow2[c];
			if (diff >0)
			{
				pRowB[c] = 255;
			}
			else
			{
				pRowB[c] = 0;
			}
		}
	}

	return 0;
}

// src0---half black / half white
// src1---half white / half black
int SegBW2(IplImage* src0, IplImage* bina)
{
	if (src0 == NULL)
		return -1;

	int w, h, ws8;
	w = src0->width;
	h = src0->height;
	ws8 = src0->widthStep;

	float T = 0;
	for (int r = 0; r < h; r++)
	{
		for (int c = 0; c < w; c++)
		{
			unsigned char* pRow = (unsigned char*)src0->imageData + r*ws8;
			T = T + float(pRow[c]);
		}
	}
	T /= w*h;

	for (int r = 0; r < h; r++)
	{
		unsigned char* pRow1 = (unsigned char*)src0->imageData + r*ws8;
		unsigned char* pRowB = (unsigned char*)bina->imageData + r*ws8;
		for (int c = 0; c < w; c++)
		{
			//int diff = pRow1[c] - pRow2[c];
			if (pRow1[c]>int(T))
			{
				pRowB[c] = 255;
			}
			else
			{
				pRowB[c] = 0;
			}
		}
	}

	return 0;
}

//********************************************************************
// unwrapping using gray coding
// the 0th pattern is all black, the 1th pattern is all white
//********************************************************************
int DecodeGrayBW(IplImage** src0, int nImages, int diffT, float *column, int wProject)
{
	if (src0 == NULL)
		return -1;
	int w, h, ws8;
	w = src0[0]->width;
	h = src0[0]->height;

	IplImage* validMap = cvCreateImage(cvSize(w, h), 8, 1);
	cvZero(validMap);

	const int NaN = 128;

	// 转灰度图
	IplImage* gray[100] = { NULL };
	for (int n = 0; n<nImages; n++)
	{
		gray[n] = cvCreateImage(cvSize(w, h), 8, 1);
		if (src0[n]->nChannels == 1)
		{
			cvCopy(src0[n], gray[n]);
		}
		else
		{
			cvCvtColor(src0[n], gray[n], CV_BGR2GRAY);
		}
	}

	ws8 = gray[0]->widthStep;

	// Segmentation
	IplImage* bina[100 / 2] = { NULL };

	for (int n = 0; n<nImages - 2; n++) // 第1张为全白pattern, 第2张为全黑pattern
	{
		bina[n] = cvCreateImage(cvSize(w, h), 8, 1);
		cvZero(bina[n]);
	}

	IplImage* thresholdMap = cvCreateImage(cvSize(w, h), 8, 1);
	// compute threshold-map
	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			int idx = r*ws8 + c;
			*(thresholdMap->imageData + idx) = (*((byte*)gray[0]->imageData + idx) + *((byte*)gray[1]->imageData + idx)) / 2;
		}
	}

	int cpuNum = max(GetCpuNum() - 1, 1);
	omp_set_num_threads(cpuNum);
	cout << "cpu number " << cpuNum << endl;

#pragma omp parallel for 
	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			for (int n = 0; n<nImages - 2; n++)
			{
				unsigned char* pRow = (unsigned char*)gray[n + 2]->imageData + r*ws8;

				unsigned char* pRow2 = (unsigned char*)thresholdMap->imageData + r*ws8; // 阈值

				unsigned char* pRowB = (unsigned char*)bina[n]->imageData + r*ws8; // 二值标记

				int diff = pRow[c] - pRow2[c]; // 

				if (diff >= 0)
				{
					pRowB[c] = 255;
				}
				else if (diff<0)
				{
					pRowB[c] = 0;
				}
				else
				{
					pRowB[c] = NaN;
				}

				if (abs(diff)>diffT)
				{
					*(validMap->imageData + r*ws8 + c) = 255;
				}
			}
		}
	}

	int length = int(log((double)wProject) / log(2.0) + 0.1);

	char grayCode[20] = { 0 }, binaCode[20] = { 0 };
	memset(grayCode, 1, sizeof(grayCode));
	memset(binaCode, 1, sizeof(binaCode));

	// *******Decode***********
	//int cpuNum = max(GetCpuNum()-1, 1);
	//	omp_set_num_threads(cpuNum);
	//#pragma omp parallel for 
	for (int r = 0; r<h; r++)
	{
		for (int c = 0; c<w; c++)
		{
			bool valid = true;
			for (int n = 0; n<nImages - 2; n++)
			{
				int val = *((unsigned char*)bina[n]->imageData + r*ws8 + c);
				if (val == NaN)
				{
					valid = false;
					break;
				}
				else if (val == 255)
					grayCode[n] = 1;
				else if (val == 0)
					grayCode[n] = 0;
			}
			if (*((unsigned char*)validMap->imageData + r*ws8 + c) == 0)
			{
				valid = false;
			}

			if (valid)
			{
				Gray2Bina(grayCode, binaCode, length);

				float decVal = 0;
				Bina2Decimal(binaCode, length, decVal);
				// 
				*(column + r*w + c) = decVal;
			}
			else
			{
				*(column + r*w + c) = -1;
			}
		}
	}

	for (int i = 0; i<nImages; i++)
	{
		cvReleaseImage(&gray[i]);
	}

	for (int n = 0; n<nImages - 2; n++)
	{
		cvReleaseImage(&bina[n]);
	}

	cvReleaseImage(&validMap);

	cvReleaseImage(&thresholdMap);

	return 0;
}

// unwrapping using the reference image
int Unwrapping_RefImg(int w, int h, float *rtmPhi, float *refPhi, float *absPhi, float *disp, int minDisp)
{
	float *phi0 = new float[w*h];
	float *rphi0 = new float[w*h];

	float pi = 3.14159f;

	for (int r = 0; r < h; r++)
	{
		for(int c=0; c<w; c++)
		{
			if ((r == 179) && (c == 108))
			{
				r = r;
			}
			int idx = r*w + c;
			if (rtmPhi[r*w + c] <= -4)
				continue;

			int dx = int(disp[idx]-0.5f);

			if(dx<=minDisp)
			{
				//absPhi[idx] = 0;
				continue;
			}
			int xRef = c - dx;
			if (xRef < 0)
				continue;
			float roughPhi = refPhi[r*w+ xRef];

			phi0[idx] = roughPhi;

			rphi0[idx] = retPhiRef[r*w + xRef];
			
			int k = int((roughPhi - rtmPhi[idx]) / (2 * pi) + 0.5f);

			absPhi[idx] = 2 * k*pi + rtmPhi[idx];
		}
	}

	OutData2Txt(phi0, w, h, w, "phi0.txt");
	OutData2Txt(rphi0, w, h, w, "rphi0.txt");

	delete[] phi0;
	delete[] rphi0;

	return 0;
}

//******************************************************************
// compute the absolute phase map for the reference image
//******************************************************************
void RefImgAbsPhase(IplImage** binaImages, int nBinaImages, IplImage** fringeImages, int nFringeImages, 
	          float *absPhi, float* retPhiRef, float nFringes)
{
	float pi = 3.1415926f;

	int w = binaImages[0]->width;
	int h = binaImages[0]->height;

	//float* rtmPhi =new float[w*h];
	float *column = new float[w*h];
	memset(retPhiRef, 0, sizeof(float)*w*h);
	
	int diffT = 18;
	IplImage* bina = cvCreateImage(cvSize(w, h), 8, 1);
	//SegBW(binaImages[0], binaImages[1], bina);
	//cvSaveImage("d:/bw.bmp", bina);
	SegBW2(binaImages[0], bina);
	cvSaveImage("bw2.bmp", bina);

	WrapPhaseShift(fringeImages, nFringeImages, retPhiRef, diffT);
	UnwrappingPhase(bina, retPhiRef, absPhi, nFringes);
	
	OutData2Txt(retPhiRef, w, h, w, "rtmPhiRef.txt");
	OutData2Txt(absPhi,     w, h, w, "absPhiRef.txt");
	
	delete[] column;
	cvReleaseImage(&bina);

	return;
}

// 参考图解相
int Unwrapping(const float* refPhase, int w, int h, float *depth, float *rtmPhase, int minDisp)
{
	float pi = 3.1415926f;
	for (int r = 0; r < h; r++)
	{
		for(int c=0; c<w; c++)
		{
			int idx = r*w + c;
			float depthVal = depth[idx];
			if (depthVal <= minDisp)
				continue;
			float a_phi0 = refPhase[idx];

			float r_phi = rtmPhase[idx];
			
			int k = int((a_phi0 - r_phi) / (2 * pi) + 0.5f);

			rtmPhase[idx] += k * 2 * pi;
		}
	}

	return 0;
}

void StereoRectify(cv::Mat &map1, cv::Mat &map2, IplImage* src, IplImage* dst)
{
	cv::Mat srcMat(src);
	cv::Mat dstMat;
	
	cv::remap(srcMat, dstMat, map1, map2, cv::INTER_LINEAR);

	//deep copy
	IplImage recImg1_Ipl = dstMat;

	cvCopy(&recImg1_Ipl, dst);

	return;
}

// cam_K_new---------new intrisinc camera parameter
// R_cam_new---------new rotation matrix of camera
void ComputeRectifyMap(const cv::Mat &cam_K, const cv::Mat &cam_kc, 
	            cv::Mat &R, const cv::Mat &T, cv::Size size, 
	            cv::Mat &map1, cv::Mat &map2, 
	            cv::Mat &cam_K_new, cv::Mat &R_cam_new)
{
	cv::Mat R_cam, T_cam;
	R_cam = R.inv();
	T_cam = -R.inv()*T;

	cout << "T_cam " << T_cam << endl;

	cv::Mat Op = cv::Mat::zeros(3, 1, CV_64F); // proj center
	cv::Mat Oc; // cam center
	Oc = -R_cam.inv()*T_cam;
	cv::Mat vBase = Op - Oc;

	double baseline = cv::norm(vBase);
	vBase = vBase / baseline;// r1_new
	cout << "r1_old: " << R_cam.row(0) << endl;
	cout << "r1_new: " << vBase.t() << endl;

	cv::Mat r2_new, r3_new;
	cv::Mat r3_old = R_cam.row(2);
	cv::Mat r3_old_t = r3_old.t();
	r2_new = r3_old_t.cross(vBase);

	r3_new = vBase.cross(r2_new);

	R_cam_new = cv::Mat::eye(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
	{
		R_cam_new.at<double>(0, i) = vBase.at<double>(i, 0);
		R_cam_new.at<double>(0, i) = vBase.at<double>(i, 0);
	}
	for (int i = 0; i < 3; i++)
	{
		R_cam_new.at<double>(1, i) = r2_new.at<double>(i, 0);
		R_cam_new.at<double>(1, i) = r2_new.at<double>(i, 0);
	}
	for (int i = 0; i < 3; i++)
	{
		R_cam_new.at<double>(2, i) = r3_new.at<double>(i, 0);
		R_cam_new.at<double>(2, i) = r3_new.at<double>(i, 0);
	}

	cout << "R_cam_new: " << R_cam_new << endl;

	std::cout << std::endl;
	cout << "R_cam " << endl << R_cam << endl;
	cv::Size imageSize;

	int m1type = CV_32FC1;

	cv::Mat R10;
	R10 = R_cam_new*R_cam.inv();

	cout << "R10\n" << R10 << endl;
	//cv::Mat cam_K_new;
	cam_K_new = cv::Mat::eye(3, 3, CV_32F);
	cam_K.copyTo(cam_K_new);
	cam_K_new.at<double>(0, 2) -= 270; // 390
	cout << "cam_K\n" << cam_K << endl << "cam_K_new\n" << cam_K_new;

	// compute mapping matrix
	cv::initUndistortRectifyMap(cam_K, cam_kc, R10, cam_K_new, size, m1type, map1, map2);

	return;
}

int Reconstruct(IplImage** fringeImages, int nImages, IplImage* speckleRef, float *refPhi, float *refPhiR,
			int minDisp,
	      std::vector<CvPoint3D32f> &points, float* a_phi, void* pMatcher)
{
	int w = fringeImages[0]->width;
	int h = fringeImages[0]->height;

	// 相位计算
	int diffT = 18;
	float *r_phi = new float[w*h];
	WrapPhaseShift(fringeImages, 3, r_phi, diffT);

	OutData2Txt(r_phi, w, h, w, "rtmPhi.txt");

	// speckle image matching
	float *dispMap = new float[w*h];
	//cvSaveImage("d:/L.bmp", fringeImages[3]);
	//cvSaveImage("d:/R.bmp", speckleRef);
	int nRet = StereoMatchPhase(pMatcher, fringeImages[3], speckleRef, r_phi, refPhiR, dispMap);
	//int nRet = StereoMatchPhase(pMatcher, fringeImages[3], speckleRef, NULL, NULL, dispMap);

	OutData2Txt(dispMap, w, h, w, "disp.txt");

	// unwrapping using the reference image
	Unwrapping_RefImg(w, h, r_phi, refPhi, a_phi, dispMap, minDisp);

	OutData2Txt(a_phi, w, h, w, "absPhi2.txt");

	delete[] r_phi;
	delete[] dispMap;

	return 0;
}

void WritePointCloud(CvPoint3D32f *ppoints, int npt, char* filename)
{
	if (ppoints == 0)
		return;

	cout << "Write Point Cloud to "<<filename << endl;
	FILE*fp = fopen(filename, "w");

	if (fp == NULL)
		return;

	for (int v = 0; v<npt; v++)
	{
		fprintf(fp, "%f ", ppoints[v].x);
		fprintf(fp, "%f ", ppoints[v].y);
		fprintf(fp, "%f\n", ppoints[v].z);
	}

	fclose(fp);
}

// testing pipeline
int main()
{
	printf("Start testing phase unwrapping pipeline.....\n\n");

	int w = 640;
	int h = 480;

	// Read calibration files
	cv::Mat cam_K, cam_kc, proj_K, proj_kc, R, T;
	cv::Mat cam_K_new, R_cam_new;
	cv::Mat map1, map2;
	{
		char filePath[256] = "dataset\\testImage5\\calibration.yml";

		ReadCalibParams(filePath, cam_K, cam_kc, proj_K, proj_kc, R, T);

		cv::Size size;
		size.width = w;
		size.height = h;
		ComputeRectifyMap(cam_K, cam_kc, R, T, size, map1, map2, cam_K_new, R_cam_new);
	}

	// Read ref images
	int nBinaImages = 1;
	IplImage* binaImages[22];
	IplImage* binaImages_rect[22];
	for(int n=0; n<nBinaImages; n++)
	{
		char name[256];
		sprintf_s(name, "dataset\\testImage5\\ref\\cam_%02d.png", n);
		binaImages[n] = cvLoadImage(name, 0);

		//cvShowImage("SRC", binaImages[n]);
		//cvWaitKey(100);

		binaImages_rect[n] = cvCloneImage(binaImages[n]);

		StereoRectify(map1, map2, binaImages[n], binaImages_rect[n]);
	}

	// Read ref fringe images and speckle image
	int nFringeImages = 3;
	IplImage* fringeImages[4];
	IplImage* fringeImages_rect[4]; // the last image is the speckle image.
	for (int n = 0; n<nFringeImages+1; n++)
	{
		char name[256];
		sprintf_s(name, "dataset\\testImage5\\ref\\ref-speckle\\cam_%02d.png", n);
		fringeImages[n] = cvLoadImage(name, 0);

		cvShowImage("SRC", fringeImages[n]); cvWaitKey(40);

		fringeImages_rect[n] = cvCloneImage(fringeImages[n]);

		StereoRectify(map1, map2, fringeImages[n], fringeImages_rect[n]);
	}

	// Compute the absolute phase for ref image
	float N_fringes = 25.6f;
	float* absPhiRef = new float[w*h];
	retPhiRef = new float[w*h];
	RefImgAbsPhase(binaImages_rect, nBinaImages, fringeImages_rect, nFringeImages, 
		 absPhiRef, retPhiRef, N_fringes);

	//OutData2Txt(absPhi, w, h, w, "d:/absPhi.txt");

	IplImage* speckleImg = fringeImages_rect[3];

	//*******************************************************
	// Processing real-time images
	//*******************************************************
	{
		// Read test fringe images and speckle image
		int nFringeImages = 3;
		IplImage* fringeImages[4];
		IplImage* fringeImages_rect[4];
		for (int n = 0; n<nFringeImages + 1; n++)
		{
			char name[256];
			sprintf_s(name, "dataset\\testImage5\\david1\\cam_%02d.png", n);
			fringeImages[n] = cvLoadImage(name, 0);

			cvShowImage("SRC", fringeImages[n]); cvWaitKey(40);

			fringeImages_rect[n] = cvCloneImage(fringeImages[n]);

			StereoRectify(map1, map2, fringeImages[n], fringeImages_rect[n]);
		}
		vector<CvPoint3D32f> points;
		void *pMatcher;
		int minDisp = -50;
		int maxDisp = 165;
		InitMatch2(pMatcher, maxDisp, minDisp, 9, 3);

		float *absPhi = new float[w*h];
		Reconstruct(fringeImages_rect, 4, speckleImg, absPhiRef, retPhiRef, minDisp, points, absPhi, pMatcher);

		// read phase error compensation table from file
		int row, col;
		std::vector<float> phiLut0, phiLut;
		ReadTxtData32F("dataset\\testImage5\\lut.txt", phiLut0, row, col);
		phiLut = phiLut0;

		// filtering for LUT
		if (!phiLut.empty())
		{
			int winR = 3;
			float sigma = 1.0f;
			GaussSmoothData1D(&phiLut0[0], &phiLut[0], phiLut.size(), sigma, winR);
			OutData2Txt(&phiLut[0], 1, phiLut.size(), 1, "lut-f.txt");
		}

		// generate 3d points using absolute phase map
		cv::Mat R_proj = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat T_proj = cv::Mat::zeros(3, 1, CV_64F);
		AbsolutePhase2PointCloud(absPhi, w, h, points, 
			cam_K_new, R_cam_new, T, proj_K, R_proj, T_proj, N_fringes, phiLut);

		WritePointCloud(&points[0], points.size(), "points.xyz");

		delete[] absPhi;
		for (int n = 0; n < nFringeImages + 1; n++)
		{
			cvReleaseImage(&fringeImages[n]);
			cvReleaseImage(&fringeImages_rect[n]);
		}
	}
	//*************************************************************

	//*************************************************************
	// release memory
	//*************************************************************
	delete[] absPhiRef;
	delete[] retPhiRef;
	
	for (int n = 0; n < nBinaImages; n++)
	{
		cvReleaseImage(&binaImages[n]);
		cvReleaseImage(&binaImages_rect[n]);
	}
	for (int n = 0; n < nFringeImages + 1; n++)
	{
		cvReleaseImage(&fringeImages[n]);
		cvReleaseImage(&fringeImages_rect[n]);
	}

    return 0;
}

