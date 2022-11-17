// OpenCVWebcamTest.cpp
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//#using < System.dll >			// for System::IO:Ports
//#pragma comment(lib, "MSCOREE.lib")
#include <cstdio>
#include<iostream>
#include<conio.h>           // may have to modify this line if not using Windows
#include "Serial.h"
#include "SerialUtil.h"




void PrintCommState(DCB dcb) {
	//  Print some of the DCB structure values
	printf("\nBaudRate = %d, ByteSize = %d, Parity = %d, StopBits = %d\n",
		dcb.BaudRate,
		dcb.ByteSize,
		dcb.Parity,
		dcb.StopBits);
}


BOOL SendCmd(HANDLE hComm, char * lpBuf, DWORD dwToWrite, DWORD &dwWritten) {
	OVERLAPPED osWrite = { 0 };
	DWORD dwRes;
	BOOL fRes = FALSE;

	// Create this write operation's OVERLAPPED structure's hEvent.
	osWrite.hEvent = ::CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		// error creating overlapped event handle
		return FALSE;
	}

	// Issue write.
	if (!::WriteFile(hComm, lpBuf, dwToWrite, &dwWritten, &osWrite)) {
		if (::GetLastError() != ERROR_IO_PENDING) {
			// WriteFile failed, but isn't delayed. Report error and abort.
			fRes = FALSE;
		}
		else {
			// Write is pending.
			dwRes = ::WaitForSingleObject(osWrite.hEvent, INFINITE);
			switch (dwRes) {
				// OVERLAPPED structure's event has been signaled. 
			case WAIT_OBJECT_0:

				if (!::GetOverlappedResult(hComm, &osWrite, &dwWritten, FALSE))
					fRes = FALSE;
				else
					// Write operation completed successfully.
					fRes = TRUE;
				break;

			default:
				// An error has occurred in WaitForSingleObject.
				// This usually indicates a problem with the
				// OVERLAPPED structure's event handle.
				fRes = FALSE;
				break;
			}
		}
	}
	else {

		// WriteFile completed immediately.
		fRes = TRUE;
	}

	if (osWrite.hEvent != NULL) ::CloseHandle(osWrite.hEvent);
	return fRes;
}

const int max_value_H = 360 / 2;
const int max_value = 255;
const cv::String window_capture_name = "Video Capture";
const cv::String window_detection_name = "Object Detection";
//int high_H = max_value_H, high_S = max_value, high_V = max_value;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = 180, high_S = 100, high_V = 196;
cv::Vec3b current_HSV;
int find_Fig = 0;
int rectSize = max_value * 100;


struct mouse_p {
	int x = 0;
	int y = 0;
	cv::Vec3b color;
} pixel_xy;


enum FIG_TYPE {
	NO_FIGURE,
	RECTANGLE,
	CIRCLE
};

const int figSeq_LeftSide[3] = { FIG_TYPE::RECTANGLE, FIG_TYPE::RECTANGLE, FIG_TYPE::NO_FIGURE };
const int figSeq_RightSide[3] = { FIG_TYPE::CIRCLE, FIG_TYPE::RECTANGLE, FIG_TYPE::RECTANGLE };
int figure_sequence[3] = { FIG_TYPE::NO_FIGURE, FIG_TYPE::NO_FIGURE, FIG_TYPE::NO_FIGURE }; // types of figures: Left: RectRectCirc, Right: CRR
cv::RotatedRect figure_sequence_rr[3];

enum SIDE {
	NO_SIDE,
	LEFT,
	RIGHT
};

double circleArea(cv::RotatedRect cv_circle) {
	// S_кв / S_круга = 4R ^ 2 / пR ^ 2 = 4 / п		M_PI = 3.14159265358979323846
	// S_circle = pi*S_square/4
	// Pi can be calculated as atan(1)*4
	double area = cv_circle.size.area() * atan(1);
	return area;
}

int isRect(std::vector<cv::Point> contour) {
	if (contour.size() >= 4) {
		cv::RotatedRect minRect = cv::minAreaRect(contour);
		double area = cv::contourArea(contour);
		if (area*1.2 >= minRect.size.area() && area <= minRect.size.area()*1.2) {
			return 1;
		}
	}
	return 0;
}

int isCircle(std::vector<cv::Point> contour) {
	if (contour.size() > 5) {
		cv::RotatedRect minCircle = cv::fitEllipse(cv::Mat(contour));
		double area = cv::contourArea(contour);
		if (area*1.1 >= circleArea(minCircle) && (area / 1.1 <= circleArea(minCircle))) {
			return 1;
		}
	}
	return 0;
}

int isRightest(cv::RotatedRect rectA, cv::RotatedRect rectB) {
	int triC = sqrt(pow(rectA.center.x - rectB.center.x, 2) + pow(rectA.center.y - rectB.center.y, 2));
	if (((rectA.size.width / 2 + rectB.size.width / 2) < triC )&& (triC < (rectA.size.width / 2 + rectB.size.width / 2)*2.5))
		if (rectA.center.x < rectB.center.x)
			return 1;
	return 0;
}
int isLeftest(cv::RotatedRect rectA, cv::RotatedRect rectB) {
	int triC = sqrt(pow(rectA.center.x - rectB.center.x, 2) + pow(rectA.center.y - rectB.center.y, 2));
	if (((rectA.size.width / 2 + rectB.size.width / 2) < triC) && (triC < (rectA.size.width / 2 + rectB.size.width / 2)*2.5))
		if (rectA.center.x > rectB.center.x)
			return 1;
	return 0;
}

int isSide(int *fig_seq, int n = 3)		// for future if sequence will > 3
{
	int i = 0;
	while (fig_seq[i] == figSeq_LeftSide[i]) {
		i++;
	}
	if (i == n)
		return SIDE::LEFT;
	else {
		i = 0;
		while (fig_seq[i] == figSeq_RightSide[i]) {
			i++;
		}
		if (i == n)
			return SIDE::RIGHT;
		else
			return SIDE::NO_SIDE;
	}
}

static void on_low_H_thresh_trackbar(int, void *) {
	low_H = min(high_H - 1, low_H);
	cv::setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *) {
	high_H = max(high_H, low_H + 1);
	cv::setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *) {
	low_S = min(high_S - 1, low_S);
	cv::setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *) {
	high_S = max(high_S, low_S + 1);
	cv::setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *) {
	low_V = min(high_V - 1, low_V);
	cv::setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *) {
	high_V = max(high_V, low_V + 1);
	cv::setTrackbarPos("High V", window_detection_name, high_V);
}

static void mouse_callback(int  event, int  x, int  y, int  flag, void *param) {
	// set color HSV +-10
	if (event == cv::EVENT_LBUTTONUP/*cv::EVENT_LBUTTONDOWN */) {
		pixel_xy.color = current_HSV;
		cv::setTrackbarPos("Low H", window_detection_name, (pixel_xy.color[0]) - 10);
		cv::setTrackbarPos("High H", window_detection_name, (pixel_xy.color[0]) + 10);
		cv::setTrackbarPos("Low S", window_detection_name, (pixel_xy.color[1]) - 10);
		cv::setTrackbarPos("High S", window_detection_name, (pixel_xy.color[1]) + 10);
		cv::setTrackbarPos("Low V", window_detection_name, (pixel_xy.color[2]) - 10);
		cv::setTrackbarPos("High V", window_detection_name, (pixel_xy.color[2]) + 50);
	}
	if (event == cv::EVENT_MOUSEMOVE) {
		//std::cout << "(" << x << ", " << y << ")" << std::endl;
		pixel_xy.x = x;
		pixel_xy.y = y;
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
	cv::VideoCapture capWebcam(0);// ("http://192.168.2.32/mjpg/video.mjpg");            // declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
	//array<Object^>^ objectArray = SerialPort::GetPortNames();

	if (capWebcam.isOpened() == false) {									// check if VideoCapture object was associated to webcam successfully                      
		std::cout << "error: capWebcam not accessed successfully\n\n";      // if not, print error message to std out
		_getch();                                                           // may have to modify this line if not using Windows
		return(0);                                                          // and exit program
	}
	
	char comn[6] = { "COM5 " };
	DCB dcb;
	if (argc > 0) {
		if (strcmp(argv[1], "\p"))
			//comn = atoi(argv[2]);
			strcpy_s(comn, argv[2]);
	}
	char sPort[10];
	sprintf_s(sPort, "\\\\.\\%s", comn);
	//CreateFileA()
	std::cout << argv[0] << argv[1] << argv[2] << std::endl;

	cv::namedWindow(window_detection_name);
	cv::setMouseCallback(window_detection_name, mouse_callback);
	// Trackbars to set thresholds for HSV values
	cv::createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
	cv::createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
	cv::createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
	cv::createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
	cv::createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
	cv::createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
	cv::createTrackbar("Rect size", window_detection_name, &rectSize, max_value);
	cv::setTrackbarPos("Rect size", window_detection_name, 10000);
	char charCheckForEscKey = 0;
	cv::Mat frame, frame_small, frame_HSV, frame_threshold, frame_small_01, frame_drawing;

	std::vector<std::vector<cv::Point>> contours;
	cv::Mat box;
	cv::Vec4i hierarchy;


	BOOL bSuccess = FALSE;

	SerialUtil* su = new SerialUtil(sPort);
	HANDLE hPort = su->SP->getHandle();
	string str;
	if(SerialUtil::SP->IsConnected()) {

		printf(sPort);
		printf("\n");
		dcb.BaudRate = CBR_9600;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;

		PrintCommState(dcb);      //  Output to console
		bSuccess = SetCommState(hPort, &dcb);
		if(bSuccess)
			std::cout << "Success port setting state" << std::endl;
		// Contains various COM timeouts 
		COMMTIMEOUTS CommTimeouts;

		// Timeouts in msec.
		CommTimeouts.ReadIntervalTimeout = MAXDWORD;
		CommTimeouts.ReadTotalTimeoutMultiplier = 10;
		CommTimeouts.ReadTotalTimeoutConstant = 100;
		CommTimeouts.WriteTotalTimeoutMultiplier = 10;
		CommTimeouts.WriteTotalTimeoutConstant = 100;
		// Set COM timeouts.
		bSuccess = ::SetCommTimeouts(hPort, &CommTimeouts);
		if (bSuccess)
			std::cout << "Success port timeout setting" << std::endl;

		bSuccess = PurgeComm(hPort, PURGE_RXCLEAR | PURGE_TXCLEAR);
		if (bSuccess)
		std::cout << "Success port cleaning" << std::endl;
		
		char cmd[] = "0";

		for (int i = 0; i < 10; i++) {
			capWebcam >> frame;
			cv::resize(frame, frame_small, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
		}
		while (true) {
			capWebcam >> frame;
			DWORD dwWritten;

			cv::resize(frame, frame_small_01, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
			if (frame.empty()) {
				std::cout << "No frame" << std::endl;
				break;
			}
			// Convert from BGR to HSV colorspace
			for (int i = 0; i < frame_small_01.rows; i++)
				for (int j = 0; j < frame_small_01.cols; j++) {
					if (frame_small.at<cv::Vec3b>(i, j)
						[0] - frame_small_01.at<cv::Vec3b>(i, j)[0])
						frame_small.at<cv::Vec3b>(i, j)[0] = frame_small.at<cv::Vec3b>(i, j)[0] * 0.6 + frame_small_01.at<cv::Vec3b>(i, j)[0] * 0.4;
					frame_small.at<cv::Vec3b>(i, j)[1] = frame_small.at<cv::Vec3b>(i, j)[1] * 0.6 + frame_small_01.at<cv::Vec3b>(i, j)[1] * 0.4;
					frame_small.at<cv::Vec3b>(i, j)[2] = frame_small.at<cv::Vec3b>(i, j)[2] * 0.6 + frame_small_01.at<cv::Vec3b>(i, j)[2] * 0.4;
				}
			cvtColor(frame_small, frame_HSV, cv::COLOR_BGR2HSV);
			frame_drawing = frame_small.clone();

			current_HSV = frame_HSV.at<cv::Vec3b>(pixel_xy.y, pixel_xy.x);
			inRange(frame_HSV, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);

			cv::findContours(frame_threshold, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
			cv::Mat drawing = cv::Mat::zeros(frame_threshold.size(), CV_8UC3);

			int maxSizeRect = frame_small.size().area() / 6;
			cv::RotatedRect minRect;
			cv::RotatedRect minCircle1;
			cv::RotatedRect minCircle2;

			for (int i = 0; i < contours.size(); i++) // while figures finded about 3
			{
				cv::Scalar color = cv::Scalar(255, 255, 255);
				minRect = cv::minAreaRect(cv::Mat(contours[i]));

				double area = cv::contourArea(contours[i]);

				if (area >= 100 && area <= maxSizeRect)	// ai - finded Figures (first 1 rectangle)
				{
					if (isRect(contours[i])) {
						std::cout << "\tRectangle";
						for (int j = 0; j < contours.size(); j++) {
							if (isCircle(contours[j])) {
								if (contours[j].size() > 5) {
									minCircle1 = cv::fitEllipse(cv::Mat(contours[j]));
									if (isRightest(minRect, minCircle1)) {
										std::cout << "\tCircle";
										for (int y = 0; y < contours.size(); y++) {
											if (isCircle(contours[y])) {
												if (contours[y].size() > 5) {
													minCircle2 = cv::fitEllipse(cv::Mat(contours[y]));
													if (isRightest(minCircle1, minCircle2)) {
														std::cout << "\tCircle";
														cv::Point2f rect_points[4];
														minRect.points(rect_points);
														for (int k = 0; k < 4; k++) {
															line(frame_drawing, rect_points[k], rect_points[(k + 1) % 4], color, 2, 8);
														}
														ellipse(frame_drawing, minCircle1, color, 2, 8);
														ellipse(frame_drawing, minCircle2, color, 2, 8);
														std::cout << "\t Right side";
														cmd[0] = 55;// '7';
														if (SendCmd(hPort, (char*)cmd, 1, dwWritten))
															cout << "sent" << endl;
														std::cout << "Send cmd " << cmd << std::endl;
														find_Fig = 2;
													}
												}
											}
										}
									}
									else {
										if (isLeftest(minRect, minCircle1)) {
											std::cout << "\tCircle";
											for (int y = 0; y < contours.size(); y++) {
												if (isCircle(contours[y])) {
													if (contours[y].size() > 5) {
														minCircle2 = cv::fitEllipse(cv::Mat(contours[y]));
														if (isLeftest(minCircle1, minCircle2)) {
															std::cout << "\tCircle";
															cv::Point2f rect_points[4];
															minRect.points(rect_points);
															for (int k = 0; k < 4; k++) {
																line(frame_drawing, rect_points[k], rect_points[(k + 1) % 4], color, 2, 8);
															}
															ellipse(frame_drawing, minCircle1, color, 2, 8);
															ellipse(frame_drawing, minCircle2, color, 2, 8);
															std::cout << "\t Left side";
															cmd[0] = 53;// '5';
															if (SendCmd(hPort, (char*)cmd, 1, dwWritten))
																cout << "sent" << endl;
															std::cout << "Send cmd " << cmd << std::endl;
															find_Fig = 2;
														}
													}
												}
											}
										}
									}
								}
							}
						}
						std::cout << std::endl;
						find_Fig = 1;
					}
					else {
						find_Fig = 0;
					}
				}
				else {
					find_Fig = 0;

				}
			}

			// Show the frames
			cv::imshow(window_detection_name, frame_drawing);
			cv::imshow(window_detection_name, frame_threshold); 
			std::string _s_rcvd = su->read();
			std::cout << "Read from com port: " << _s_rcvd << std::endl;

			//frame_threshold
			char key = (char)cv::waitKey(30);
			if (key == 'q' || key == 27) {
				break;
			}
			else {

			}
		}
	}
	return(0);
}










