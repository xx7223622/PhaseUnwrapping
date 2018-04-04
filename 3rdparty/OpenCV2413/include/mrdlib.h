#ifndef _MRDLIB_H_
#define _MRDLIB_H_

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <iostream>
using namespace std;
#ifndef NOT_USE_DLIB_NAMESPACE
using namespace dlib;
#endif
#ifdef WIN32
#if _DEBUG
#pragma comment(lib,"dlibd.lib")
#else
#pragma comment(lib,"dlib.lib")
#endif
#endif
#endif