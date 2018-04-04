#pragma once
#pragma warning(disable:4996)
#pragma warning(disable:4267)
typedef unsigned char BYTE;
#include "C:/3rdParty/OpenCV2413/include/opencv2/core/version.hpp"
#include "C:/3rdParty/OpenCV2413/include/opencv2/opencv.hpp"
typedef cv::Mat1d		 ditMat1d;
typedef cv::Mat1b		 ditMat1b;
typedef cv::Mat1s		 ditMat1s;
typedef cv::Mat1i		 ditMat1i;
typedef cv::Mat3b        ditMat3b;
typedef cv::Vec2d		 ditVec2d;
typedef cv::Vec3d		 ditVec3d;
typedef cv::Vec3f		 ditVec3f;
typedef cv::Vec3b		 ditVec3b;
typedef cv::Point3d      ditPt3d;
typedef cv::Point2i      ditPt2i;
typedef cv::Point2d      ditPt2d;
#define CV_VERSION_ID CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

//#ifdef _DEBUG
//#define cvLib(NAME) "C:/3rdParty/lib/OpenCV2413/opencv_" NAME CV_VERSION_ID "d.lib"
//#else
#define cvLib(NAME) "C:/3rdParty/OpenCV2413/lib/opencv_" NAME CV_VERSION_ID ".lib"
//#endif

#pragma comment(lib, cvLib("calib3d"))
#pragma comment(lib, cvLib("contrib"))
#pragma comment(lib, cvLib("core"))
#pragma comment(lib, cvLib("features2d"))
#pragma comment(lib, cvLib("flann"))
#pragma comment(lib, cvLib("gpu"))
#pragma comment(lib, cvLib("highgui"))
#pragma comment(lib, cvLib("imgproc"))
#pragma comment(lib, cvLib("legacy"))
#pragma comment(lib, cvLib("ml"))
#pragma comment(lib, cvLib("nonfree"))
#pragma comment(lib, cvLib("objdetect"))
#pragma comment(lib, cvLib("ocl"))
#pragma comment(lib, cvLib("photo"))
#pragma comment(lib, cvLib("stitching"))
#pragma comment(lib, cvLib("superres"))
#pragma comment(lib, cvLib("ts"))
#pragma comment(lib, cvLib("video"))

#ifndef ditMIN
#define ditMIN(a,b)  ((a) > (b) ? (b) : (a))
#endif

#ifndef ditMAX
#define ditMAX(a,b)  ((a) < (b) ? (b) : (a))
#endif


#ifndef PI			// π
#define PI			3.1415926535897932384626433832795028841971693993751
#endif

#ifndef PI2			// 2π
#define PI2			6.283185307179586476925286766558//(PI+ PI)
#endif

#ifndef PI_2		// π/2
#define PI_2		1.5707963267948966192313216916395//PI/ 2.0
#endif

#ifndef PI_4		// π/4
#define PI_4		0.78539816339744830961566084581975//PI/ 4.0
#endif

#ifndef EP			
#define EP			2.7182818284590452353602874713527
#endif

#ifndef D2R			
#define D2R			0.017453292519943295769236907684886
#endif

#ifndef R2D			
#define R2D			57.295779513082320876798154814105
#endif

#ifndef VM_EPSILON		
#define VM_EPSILON		1.110223E-30
#endif

#ifndef VM_PRECISION	
#define VM_PRECISION	1.0E-6
#endif
#define BOUND(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define	FTOI(a)			   ( (a) < 0 ? ((int)(a-0.5)) : ((int)(a+0.5)) )

#define	ditdeg2rad(a)			   ((a) * D2R)
#define	ditrad2deg(a)			   ((a) * R2D)

template<class Type>
void AllType_to_BYTE(Type *In,  BYTE *Out,  int Size)
{
	if (Size<= 0)
		return;
	Type max_r = In[0];
	Type min_r = In[0];
	for(int i= 0; i< Size; i++)
	{
		if( In[i] > max_r )
			max_r = In[i];
		else if ( In[i] < min_r )
			min_r = In[i];
	}
	max_r -= min_r;
	if( max_r != 0 )
	{
		for(int i= 0; i< Size; i++)
		{
			Out[i] = (BYTE)( (In[i]- min_r) * 255.0 / max_r + 0.5 );
		}
	}
}