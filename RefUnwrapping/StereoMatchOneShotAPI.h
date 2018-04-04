//*********************************************************************************************************
// Author: Yuhua Xu, 2018, 03, 30. xyh_nudt@163.com
// Reference image based phase unwrapping framework for a structured light system.
//*********************************************************************************************************
#pragma once
#include "opencv2\opencv.hpp"

#if defined (_WINDLL) || defined(_CONSOLE) || defined(_WINDOWS)
#ifndef API_MODULE_TYPE
#define API_MODULE_TYPE _declspec(dllexport)
#else
#define API_MODULE_TYPE _declspec(dllimport)
#endif
#else
#define API_MODULE_TYPE 2
#endif

//***********************************************************
// initializing the stereo matching library
// maxDisp.................maximum disparity
// winSize...................matching window size
// minSeedSupport..........minimum number of support of seed points(>=2)
//***********************************************************
API_MODULE_TYPE
int InitMatch(void* &matcher, int maxDisp, int winSize, int minSeedSupport);

// stereo matching
API_MODULE_TYPE
int StereoMatch(void* pMatcher, IplImage* leftSrc, IplImage* rightSrc,
	IplImage* leftRec, IplImage* rightRec, float *dispMap);

//***********************************************************
// initializing the stereo matching library
// maxDisp.................maximum disparity
// winSize...................matching window size
// minSeedSupport..........minimum number of support of seed points(>=2)
//***********************************************************
API_MODULE_TYPE
int InitMatch2(void* &matcher, int maxDisp, int minDisp, int winSize, int minSeedSupport);

// stereo mathcing (the images are rectified)
API_MODULE_TYPE
int StereoMatch2(void* pMatcher, IplImage* leftRec, IplImage* rightRec, float *D1);

// stereo mathcing (the images are rectified and the phase information is available)
API_MODULE_TYPE
int StereoMatchPhase(void* pMatcher, IplImage* leftRec, IplImage* rightRec,
	float *leftPhi, float *rightPhi, float *D1);

// compute point cloud from the disparity map
API_MODULE_TYPE
int ConvertDisparityToWorldPoints(void* pMatcher, float *dispMap, std::vector<CvPoint3D32f> &points, float maxDepth = 5000);