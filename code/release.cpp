#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/dnn.hpp"
#include "rollingAverage.h"

int main(void)
{
	cv::VideoCapture inputVideo("../media/motorway.mp4");
	inputVideo.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
	inputVideo.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	if (!inputVideo.isOpened())
	{
		std::cout << "\nError opening video capture object\n";
		return -1;
	}

	bool recordOuput = false;
	cv::VideoWriter ouputFrameVideo;

	std::vector<std::string> modelNames;
	std::map<std::string, cv::Scalar> modelNamesAndColourList;
	std::ifstream modelNamesFile("../yolo/coco.names");
	if (modelNamesFile.is_open())
	{
		std::string line;
		for (int i = 0; std::getline(modelNamesFile, line); i++)
		{
			#ifdef __linux__
			line.pop_back();
			#endif
			modelNamesAndColourList.insert(std::pair<std::string, cv::Scalar>(line, cv::Scalar(255, 255, 255))); // white
			modelNames.push_back(line);
		}

		// Set these as custom colours
		modelNamesAndColourList["car"] = cv::Scalar(255, 64, 64);				// blue
		modelNamesAndColourList["truck"] = cv::Scalar(255, 64, 255);			// purple
		modelNamesAndColourList["bus"] = cv::Scalar(64, 64, 255);				// red
		modelNamesAndColourList["traffic light"] = cv::Scalar(64, 255, 255);	// yellow

		modelNamesFile.close();
	}
	else
	{
		std::cout << "\nError opening coco.names file stream or file\n";
		return -3;
	}

	cv::dnn::Net net = cv::dnn::readNetFromDarknet("../yolo/yolov4.cfg", "../yolo/yolov4.weights");
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
	std::vector<std::string> unconnectedOutLayersNames = net.getUnconnectedOutLayersNames();



	constexpr int ROI_TOP_HEIGHT = 660;
	constexpr int ROI_BOTTOM_HEIGHT = 840;
	constexpr int ROI_TOP_WIDTH = 200;
	constexpr int ROI_BOTTOM_WIDTH = 900;

	constexpr int CANNY_LOWER_THRESHOLD = 128;
	constexpr int CANNY_UPPER_THRESHOLD = 255;

	constexpr int HOUGHP_THRESHOLD = 32;
	constexpr int HOUGHP_MIN_LINE_LENGTH = 16;
	constexpr int HOUGHP_MAX_LINE_GAP = 8;

	constexpr double HORIZONTAL_GRADIENT_THRESHOLD = 0.15;
	constexpr int HORIZONTAL_LENGTH_THRESHOLD = 50;
	constexpr int HORIZONTAL_COUNT_THRESHOLD = 10;

	constexpr int SOLID_LINE_LENGTH_THRESHOLD = 75;

	constexpr int HORIZONTAL_LINE_STATE_ROLLING_AVERAGE = 10;
	constexpr int LINE_STATE_ROLLING_AVERAGE = 10;
	constexpr int DRIVING_STATE_ROLLING_AVERAGE = 10;

	int VIDEO_WIDTH = inputVideo.get(cv::CAP_PROP_FRAME_WIDTH);
	int VIDEO_HEIGHT = inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT);
	int VIDEO_FPS = inputVideo.get(cv::CAP_PROP_FPS);
	int FRAME_COUNT_THRESHOLD = VIDEO_FPS / 3.;

	constexpr int BLOB_SIZE = 608;
	constexpr double YOLO_CONFIDENCE_THRESHOLD = 0.4;
	constexpr double YOLO_NMS_THRESHOLD = 0.4;
	constexpr int BOUNDING_BOX_BUFFER = 5;

	constexpr int FONT_FACE = cv::FONT_HERSHEY_DUPLEX;
	constexpr double FONT_SCALE = 1;
	constexpr int FONT_THICKNESS = 1;





	std::vector<cv::Mat> outputBlobs;
	std::vector<cv::Rect> objectBoundingBoxes, preNMSObjectBoundingBoxes;
	std::vector<std::string> objectNames, preNMSObjectNames;
	std::vector<float> objectConfidences, preNMSObjectConfidences;
	double centerX, centerY, width, height, confidence;
	cv::Point classID;
	std::vector<int> indicesAfterNMS;
	std::string trafficLightState;

	cv::Mat frame, unEditedFrame, blobFromImg, ROIFrame, cannyFrame, blankFrame;

	rollingAverage horizontalLineStateRollingAverage(HORIZONTAL_LINE_STATE_ROLLING_AVERAGE, 2);
	rollingAverage leftLineTypeRollingAverage(LINE_STATE_ROLLING_AVERAGE, 3);
	rollingAverage middleLineTypeRollingAverage(LINE_STATE_ROLLING_AVERAGE, 3);
	rollingAverage rightLineTypeRollingAverage(LINE_STATE_ROLLING_AVERAGE, 3);
	rollingAverage drivingStateRollingAverage(DRIVING_STATE_ROLLING_AVERAGE, 5);

	std::vector<cv::Vec4i> houghLines;
	std::vector<cv::Point> lanePoints;

	std::vector<cv::Point> maskDimensions;
	maskDimensions.push_back(cv::Point(VIDEO_WIDTH / 2 - ROI_TOP_WIDTH / 2, ROI_TOP_HEIGHT));
	maskDimensions.push_back(cv::Point(VIDEO_WIDTH / 2 + ROI_TOP_WIDTH / 2, ROI_TOP_HEIGHT));
	maskDimensions.push_back(cv::Point(VIDEO_WIDTH / 2 + ROI_BOTTOM_WIDTH / 2, ROI_BOTTOM_HEIGHT));
	maskDimensions.push_back(cv::Point(VIDEO_WIDTH / 2 - ROI_BOTTOM_WIDTH / 2, ROI_BOTTOM_HEIGHT));

	int horizontalCount;

	double leftX1, leftX2, leftY1, leftY2, rightX1, rightX2, rightY1, rightY2;
	double mLeftEdgeOfMask, cLeftEdgeOfMask, mRightEdgeOfMask, cRightEdgeOfMask;
	double topMidPoint, bottomOneThird, bottomTwoThird;
	double mLeftThresholdEdge, cLeftThresholdEdge, mRightThresholdEdge, cRightThresholdEdge;

	mLeftEdgeOfMask = ((double)maskDimensions[0].y - (double)maskDimensions[3].y) / (double)((double)maskDimensions[0].x - (double)maskDimensions[3].x);
	cLeftEdgeOfMask = maskDimensions[0].y - mLeftEdgeOfMask * maskDimensions[0].x;

	mRightEdgeOfMask = ((double)maskDimensions[1].y - (double)maskDimensions[2].y) / (double)((double)maskDimensions[1].x - (double)maskDimensions[2].x);
	cRightEdgeOfMask = maskDimensions[1].y - mRightEdgeOfMask * maskDimensions[1].x;

	topMidPoint = maskDimensions[0].x + ((double)maskDimensions[1].x - (double)maskDimensions[0].x) / 2.;
	bottomOneThird = maskDimensions[3].x + ((double)maskDimensions[2].x - (double)maskDimensions[3].x) / 3.;
	bottomTwoThird = maskDimensions[3].x + 2. * ((double)maskDimensions[2].x - (double)maskDimensions[3].x) / 3.;

	mLeftThresholdEdge = ((double)ROI_TOP_HEIGHT - (double)ROI_BOTTOM_HEIGHT) / (topMidPoint - bottomOneThird);
	cLeftThresholdEdge = ROI_TOP_HEIGHT - mLeftThresholdEdge * topMidPoint;

	mRightThresholdEdge = ((double)ROI_TOP_HEIGHT - (double)ROI_BOTTOM_HEIGHT) / (topMidPoint - bottomTwoThird);
	cRightThresholdEdge = ROI_TOP_HEIGHT - mRightThresholdEdge * topMidPoint;

	double mLeftLaneEdge = 0, cLeftLaneEdge = 0, mRightLaneEdge = 0, cRightLaneEdge = 0;
	bool lineIsInBoundingBox;
	int xLowerRange, xUpperRange, yLowerRange, yUpperRange;
	double dx, dy, gradient;

	std::deque<int> leftLineTypesForDisplay(5, 0), middleLineTypesForDisplay(5, 0), rightLineTypesForDisplay(5, 0);
	std::vector<cv::Vec4i> leftLines, middleLines, rightLines;
	double leftLineAverageSize, middleLineAverageSize, rightLineAverageSize;
	int leftLineType, middleLineType, rightLineType, drivingState;
	int leftMinY = 0, rightMinY = 0, minY;

	double averageDistanceFromLeft = 0, averageDistanceFromRight = 0;
	double withinLaneCurrentDifference;
	double changingLanesCurrentDifference, changingLanesPreviousDifference = 0;
	int changingLanesframeCount = 0;
	std::string turningRequiredToReturnToCenter, currentTurningState;
	int turningRequiredToReturnToCenter_int;

	int NonZeroPixelsInGreen, NonZeroPixelsInRed;
	cv::Mat warpedimage, ImageInHSV;
	std::vector<cv::Point2f> srcTrafficLight, dstTrafficLight;

	std::string titleText = "", rightInfoTitleText = "", giveWayWarningText = "", FPSText = "", recordingOuputText;
	int baseline = 0;
	cv::Size textSize;
	cv::Point textOrg;
	cv::Rect rightInfoRect(1495, 25, 400, 360);
	cv::Rect recordOutputRect(1495, 410, 400, 50);
	cv::Rect FPSRect(25, 25, 350, 100);

	long frameNumber = 0;
	double previousFPS = 0;
	double previousAverageFPS = 0;

	int i, j;



	while (1)
	{
		auto start = std::chrono::high_resolution_clock::now();



		frameNumber++;
		if (frameNumber != 1)
			previousAverageFPS = (previousAverageFPS*(frameNumber-2) + previousFPS)/(frameNumber-1);



		inputVideo >> frame;
		if (frame.empty())
			break;
		unEditedFrame = frame.clone();



		objectBoundingBoxes.clear();
		objectNames.clear();
		objectConfidences.clear();
		preNMSObjectBoundingBoxes.clear();
		preNMSObjectNames.clear();
		preNMSObjectConfidences.clear();
		houghLines.clear();
		leftLines.clear();
		middleLines.clear();
		rightLines.clear();
		lanePoints.clear();



		cv::dnn::blobFromImage(frame, blobFromImg, 1 / 255.0, cv::Size(BLOB_SIZE, BLOB_SIZE), cv::Scalar(0), true, false, CV_32F);
		net.setInput(blobFromImg);
		net.forward(outputBlobs, unconnectedOutLayersNames);

		for (i = 0; i < outputBlobs.size(); i++)
		{
			for (j = 0; j < outputBlobs[i].rows; j++)
			{
				cv::minMaxLoc(outputBlobs[i].row(j).colRange(5, outputBlobs[i].cols), NULL, &confidence, NULL, &classID);

				if (confidence > YOLO_CONFIDENCE_THRESHOLD)
				{
					centerX = outputBlobs[i].at<float>(j, 0) * (double)VIDEO_WIDTH;
					centerY = outputBlobs[i].at<float>(j, 1) * (double)VIDEO_HEIGHT;
					width = outputBlobs[i].at<float>(j, 2) * (double)VIDEO_WIDTH + BOUNDING_BOX_BUFFER;
					height = outputBlobs[i].at<float>(j, 3) * (double)VIDEO_HEIGHT + BOUNDING_BOX_BUFFER;

					if (centerY < ROI_BOTTOM_HEIGHT)
					{
						preNMSObjectBoundingBoxes.push_back(cv::Rect(centerX - width / 2, centerY - height / 2, width, height));
						preNMSObjectNames.push_back(modelNames[classID.x]);
						preNMSObjectConfidences.push_back(confidence);

					}
				}
			}
		}

		cv::dnn::NMSBoxes(preNMSObjectBoundingBoxes, preNMSObjectConfidences, 0.0, YOLO_NMS_THRESHOLD, indicesAfterNMS);

		for (i = 0; i < indicesAfterNMS.size(); i++)
		{
			objectBoundingBoxes.push_back(preNMSObjectBoundingBoxes[indicesAfterNMS[i]]);
			objectNames.push_back(preNMSObjectNames[indicesAfterNMS[i]]);
			objectConfidences.push_back(preNMSObjectConfidences[indicesAfterNMS[i]]);

			trafficLightState = "";

			if (objectNames.back() == "traffic light")
			{
				srcTrafficLight.clear();
				srcTrafficLight.push_back(cv::Point2f(objectBoundingBoxes.back().x, objectBoundingBoxes.back().y));
				srcTrafficLight.push_back(cv::Point2f(objectBoundingBoxes.back().x + objectBoundingBoxes.back().width, objectBoundingBoxes.back().y));
				srcTrafficLight.push_back(cv::Point2f(objectBoundingBoxes.back().x, objectBoundingBoxes.back().y + objectBoundingBoxes.back().height));
				srcTrafficLight.push_back(cv::Point2f(objectBoundingBoxes.back().x + objectBoundingBoxes.back().width, objectBoundingBoxes.back().y + objectBoundingBoxes.back().height));

				dstTrafficLight.clear();
				dstTrafficLight.push_back(cv::Point2f(0, 0));
				dstTrafficLight.push_back(cv::Point2f(100, 0));
				dstTrafficLight.push_back(cv::Point2f(0, 200));
				dstTrafficLight.push_back(cv::Point2f(100, 200));

				cv::warpPerspective(unEditedFrame, warpedimage, cv::getPerspectiveTransform(srcTrafficLight, dstTrafficLight, 0), cv::Size(100, 200));

				cv::cvtColor(warpedimage, ImageInHSV, cv::COLOR_BGR2HSV);
				cv::inRange(ImageInHSV, cv::Scalar(32, 32, 32), cv::Scalar(80, 255, 255), ImageInHSV);
				NonZeroPixelsInGreen = cv::countNonZero(ImageInHSV);

				cv::cvtColor(warpedimage, ImageInHSV, cv::COLOR_BGR2HSV);
				cv::inRange(ImageInHSV, cv::Scalar(0, 64, 64), cv::Scalar(10, 255, 255), ImageInHSV);
				NonZeroPixelsInRed = cv::countNonZero(ImageInHSV);

				if ((NonZeroPixelsInGreen > NonZeroPixelsInRed) && (NonZeroPixelsInGreen > 1000))
					trafficLightState = " (Green)";
				else if ((NonZeroPixelsInRed > NonZeroPixelsInGreen) && (NonZeroPixelsInRed > 1000))
					trafficLightState = " (Red)";
			}

			cv::rectangle(frame, objectBoundingBoxes.back(), modelNamesAndColourList[objectNames.back()], 1, cv::LINE_AA);

			std::string name = objectNames.back() + ": " + std::to_string((int)(100 * objectConfidences.back())) + "%" + trafficLightState;
			int size;
			if (objectBoundingBoxes.back().width > name.size() * 9)
				size = objectBoundingBoxes.back().width;
			else
				size = name.size() * 9;
			cv::rectangle(frame, cv::Rect(objectBoundingBoxes.back().x, objectBoundingBoxes.back().y - 15, size, 15), modelNamesAndColourList[objectNames.back()], cv::FILLED, cv::LINE_AA);
			cv::putText(frame, name, cv::Point(objectBoundingBoxes.back().x, objectBoundingBoxes.back().y - 2), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0), 1, cv::LINE_AA);
		}



		blankFrame = cv::Mat::zeros(VIDEO_HEIGHT, VIDEO_WIDTH, frame.type());
		cv::fillConvexPoly(blankFrame, maskDimensions, cv::Scalar(255, 255, 255), cv::LINE_AA, 0);
		cv::bitwise_and(blankFrame, frame, ROIFrame);

		cv::cvtColor(ROIFrame, ROIFrame, cv::COLOR_BGR2GRAY);

		cv::Canny(ROIFrame, cannyFrame, CANNY_LOWER_THRESHOLD, CANNY_UPPER_THRESHOLD, 3, true);

		cv::HoughLinesP(cannyFrame, houghLines, 1, CV_PI / 180, HOUGHP_THRESHOLD, HOUGHP_MIN_LINE_LENGTH, HOUGHP_MAX_LINE_GAP);



		for (i = horizontalCount = leftLineAverageSize = middleLineAverageSize = rightLineAverageSize = 0; i < houghLines.size(); i++)
		{
			for (j = 0, lineIsInBoundingBox = false; j < objectBoundingBoxes.size(); j++)
			{
				xLowerRange = objectBoundingBoxes[j].x;
				xUpperRange = objectBoundingBoxes[j].x + objectBoundingBoxes[j].width;
				yLowerRange = objectBoundingBoxes[j].y;
				yUpperRange = objectBoundingBoxes[j].y + objectBoundingBoxes[j].height;

				if (((houghLines[i][0] >= xLowerRange) && (houghLines[i][0] <= xUpperRange)) &&
					((houghLines[i][1] >= yLowerRange) && (houghLines[i][1] <= yUpperRange)))
				{
					lineIsInBoundingBox = true;
					break;
				}

				if (((houghLines[i][2] >= xLowerRange) && (houghLines[i][2] <= xUpperRange)) &&
					((houghLines[i][3] >= yLowerRange) && (houghLines[i][3] <= yUpperRange)))
				{
					lineIsInBoundingBox = true;
					break;
				}
			}
			if (lineIsInBoundingBox)
				continue;

			dx = (double)houghLines[i][0] - (double)houghLines[i][2];
			dy = (double)houghLines[i][1] - (double)houghLines[i][3];
			if (dx == 0)
				continue;
			gradient = dy / dx;

			if (std::fabs(gradient) < HORIZONTAL_GRADIENT_THRESHOLD)
			{
				if (((houghLines[i][1] <= ROI_TOP_HEIGHT + 1) && (houghLines[i][3] <= ROI_TOP_HEIGHT + 1)) ||
					((houghLines[i][1] >= ROI_BOTTOM_HEIGHT - 1) && (houghLines[i][3] >= ROI_BOTTOM_HEIGHT - 1)))
					continue;

				if (std::sqrt(dy * dy + dx * dx) > HORIZONTAL_LENGTH_THRESHOLD)
				{
					horizontalCount++;
					continue;
				}
			}

			else
			{
				leftY1 = mLeftEdgeOfMask * houghLines[i][0] + cLeftEdgeOfMask;
				leftY2 = mLeftEdgeOfMask * houghLines[i][2] + cLeftEdgeOfMask;
				if ((houghLines[i][1] <= leftY1 + 1) && (houghLines[i][3] <= leftY2 + 1))
					continue;

				leftY1 = mLeftThresholdEdge * houghLines[i][0] + cLeftThresholdEdge;
				leftY2 = mLeftThresholdEdge * houghLines[i][2] + cLeftThresholdEdge;

				if ((houghLines[i][1] < leftY1) && (houghLines[i][3] < leftY2) && gradient < 0)
				{
					leftLines.push_back(houghLines[i]);
					leftLineAverageSize += std::sqrt((houghLines[i][0] - houghLines[i][2]) * (houghLines[i][0] - houghLines[i][2]) + (houghLines[i][1] - houghLines[i][3]) * (houghLines[i][1] - houghLines[i][3]));
					continue;
				}

				rightY1 = mRightEdgeOfMask * houghLines[i][0] + cRightEdgeOfMask;
				rightY2 = mRightEdgeOfMask * houghLines[i][2] + cRightEdgeOfMask;
				if ((houghLines[i][1] <= rightY1 + 1) && (houghLines[i][3] <= rightY2 + 1))
					continue;

				rightY1 = mRightThresholdEdge * houghLines[i][0] + cRightThresholdEdge;
				rightY2 = mRightThresholdEdge * houghLines[i][2] + cRightThresholdEdge;

				if ((houghLines[i][1] < rightY1) && (houghLines[i][3] < rightY2) && gradient > 0)
				{
					rightLines.push_back(houghLines[i]);
					rightLineAverageSize += std::sqrt((houghLines[i][0] - houghLines[i][2]) * (houghLines[i][0] - houghLines[i][2])	+ (houghLines[i][1] - houghLines[i][3]) * (houghLines[i][1] - houghLines[i][3]));
					continue;
				}

				middleLines.push_back(houghLines[i]);

				middleLineAverageSize += std::sqrt((houghLines[i][0] - houghLines[i][2]) * (houghLines[i][0] - houghLines[i][2]) + (houghLines[i][1] - houghLines[i][3]) * (houghLines[i][1] - houghLines[i][3]));
			}
		}



		if (horizontalCount > HORIZONTAL_COUNT_THRESHOLD)
		{
			if (horizontalLineStateRollingAverage.calculateRollingAverage(1))
			{
				giveWayWarningText = "WARNING: Giveway ahead";
				baseline = 0;
				textSize = cv::getTextSize(giveWayWarningText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
				baseline += FONT_THICKNESS;

				textOrg = cv::Point(((double)VIDEO_WIDTH - (double)textSize.width) / 2., 225 + baseline + textSize.height);

				cv::rectangle(frame, textOrg + cv::Point(0, baseline), textOrg + cv::Point(textSize.width, -textSize.height - baseline), cv::Scalar(0), cv::FILLED);

				cv::putText(frame, giveWayWarningText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);
			}
		}
		else
			horizontalLineStateRollingAverage.calculateRollingAverage(0);



		leftLineAverageSize /= (double)leftLines.size();
		middleLineAverageSize /= (double)middleLines.size();
		rightLineAverageSize /= (double)rightLines.size();



		if (leftLines.size() == 0)
			leftLineType = leftLineTypeRollingAverage.calculateRollingAverage(0);
		else if (leftLineAverageSize < SOLID_LINE_LENGTH_THRESHOLD)
			leftLineType = leftLineTypeRollingAverage.calculateRollingAverage(1);
		else
			leftLineType = leftLineTypeRollingAverage.calculateRollingAverage(2);

		if (middleLines.size() == 0)
			middleLineType = middleLineTypeRollingAverage.calculateRollingAverage(0);
		else if (middleLineAverageSize < SOLID_LINE_LENGTH_THRESHOLD)
			middleLineType = middleLineTypeRollingAverage.calculateRollingAverage(1);
		else
			middleLineType = middleLineTypeRollingAverage.calculateRollingAverage(2);

		if (rightLines.size() == 0)
			rightLineType = rightLineTypeRollingAverage.calculateRollingAverage(0);
		else if (rightLineAverageSize < SOLID_LINE_LENGTH_THRESHOLD)
			rightLineType = rightLineTypeRollingAverage.calculateRollingAverage(1);
		else
			rightLineType = rightLineTypeRollingAverage.calculateRollingAverage(2);



		cv::rectangle(frame, rightInfoRect, cv::Scalar(0), cv::FILLED, cv::LINE_AA, 0);

		leftLineTypesForDisplay.push_front(leftLineType);
		leftLineTypesForDisplay.pop_back();

		middleLineTypesForDisplay.push_front(middleLineType);
		middleLineTypesForDisplay.pop_back();

		rightLineTypesForDisplay.push_front(rightLineType);
		rightLineTypesForDisplay.pop_back();

		for (i = 0; i < leftLineTypesForDisplay.size(); i++)
			cv::rectangle(frame, cv::Rect(1595, 80 + i * 50, 4, 25 * leftLineTypesForDisplay[i]), cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
		for (i = 0; i < rightLineTypesForDisplay.size(); i++)
			cv::rectangle(frame, cv::Rect(1795, 80 + i * 50, 4, 25 * rightLineTypesForDisplay[i]), cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);



		if ((leftLines.size() != 0) && (middleLines.size() != 0) && (rightLines.size() != 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(0);

		else if ((leftLines.size() != 0) && (middleLines.size() == 0) && (rightLines.size() != 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(0);

		else if ((leftLines.size() == 0) && (middleLines.size() != 0) && (rightLines.size() == 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(1);

		else if ((leftLines.size() != 0) && (middleLines.size() != 0) && (rightLines.size() == 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(1);

		else if ((leftLines.size() == 0) && (middleLines.size() != 0) && (rightLines.size() != 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(1);

		else if ((leftLines.size() != 0) && (middleLines.size() == 0) && (rightLines.size() == 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(2);

		else if ((leftLines.size() == 0) && (middleLines.size() == 0) && (rightLines.size() != 0))
			drivingState = drivingStateRollingAverage.calculateRollingAverage(3);

		else
			drivingState = drivingStateRollingAverage.calculateRollingAverage(4);



		switch (drivingState)
		{
		case 0:
		{
			if (leftLines.size() != 0)
			{
				for (i = averageDistanceFromLeft = mLeftLaneEdge = cLeftLaneEdge = 0, leftMinY = leftLines[i][1]; i < leftLines.size(); i++)
				{
					leftX1 = (leftLines[i][1] - cLeftEdgeOfMask) / mLeftEdgeOfMask;
					leftX2 = (leftLines[i][3] - cLeftEdgeOfMask) / mLeftEdgeOfMask;

					averageDistanceFromLeft += std::fabs(leftLines[i][0] - leftX1);
					averageDistanceFromLeft += std::fabs(leftLines[i][2] - leftX2);

					if (leftLines[i][1] < leftMinY)
						leftMinY = leftLines[i][1];

					if (leftLines[i][3] < leftMinY)
						leftMinY = leftLines[i][3];

					mLeftLaneEdge += ((double)leftLines[i][1] - (double)leftLines[i][3]) / (double)((double)leftLines[i][0] - (double)leftLines[i][2]);
					cLeftLaneEdge += leftLines[i][1] - (((double)leftLines[i][1] - (double)leftLines[i][3]) / (double)((double)leftLines[i][0] - (double)leftLines[i][2])) * leftLines[i][0];
				}

				averageDistanceFromLeft /= (double)(leftLines.size() * 2);
				mLeftLaneEdge /= (double)(leftLines.size());
				cLeftLaneEdge /= (double)(leftLines.size());
			}

			if (rightLines.size() != 0)
			{
				for (i = averageDistanceFromRight = mRightLaneEdge = cRightLaneEdge = 0, rightMinY = ROI_BOTTOM_HEIGHT; i < rightLines.size(); i++)
				{
					rightX1 = (rightLines[i][1] - cRightEdgeOfMask) / mRightEdgeOfMask;
					rightX2 = (rightLines[i][3] - cRightEdgeOfMask) / mRightEdgeOfMask;

					averageDistanceFromRight += std::fabs(rightLines[i][0] - rightX1);
					averageDistanceFromRight += std::fabs(rightLines[i][2] - rightX2);

					if (rightLines[i][1] < rightMinY)
						rightMinY = rightLines[i][1];

					if (rightLines[i][3] < rightMinY)
						rightMinY = rightLines[i][3];

					mRightLaneEdge += ((double)rightLines[i][1] - (double)rightLines[i][3]) / (double)((double)rightLines[i][0] - (double)rightLines[i][2]);
					cRightLaneEdge += rightLines[i][1] - (((double)rightLines[i][1] - (double)rightLines[i][3]) / (double)((double)rightLines[i][0] - (double)rightLines[i][2])) * rightLines[i][0];
				}

				averageDistanceFromRight /= (double)(rightLines.size() * 2);
				mRightLaneEdge /= (double)(rightLines.size());
				cRightLaneEdge /= (double)(rightLines.size());
			}

			if ((averageDistanceFromLeft - averageDistanceFromRight) > 200)
				withinLaneCurrentDifference = 1;
			else if ((averageDistanceFromLeft - averageDistanceFromRight) < -200)
				withinLaneCurrentDifference = -1;
			else
				withinLaneCurrentDifference = (averageDistanceFromLeft - averageDistanceFromRight) / 200;

			turningRequiredToReturnToCenter_int = (withinLaneCurrentDifference * 100) - (int)(withinLaneCurrentDifference * 100) % 10;

			if (turningRequiredToReturnToCenter_int == 0)
				turningRequiredToReturnToCenter = "In Centre";
			else if (turningRequiredToReturnToCenter_int < 0)
				turningRequiredToReturnToCenter = "Turn Left " + std::to_string(-turningRequiredToReturnToCenter_int) + "%";
			else
				turningRequiredToReturnToCenter = "Turn Right " + std::to_string(turningRequiredToReturnToCenter_int) + "%";

			blankFrame = cv::Mat::zeros(VIDEO_HEIGHT, VIDEO_WIDTH, frame.type());
			cv::rectangle(blankFrame, cv::Rect(1695 - turningRequiredToReturnToCenter_int - 75, 205 - 100, 150, 200), cv::Scalar(0, 200, 200), cv::FILLED, cv::LINE_AA);
			cv::add(frame, blankFrame, frame);

			if ((mLeftLaneEdge != 0) && (mRightLaneEdge != 0))
			{
				if (leftMinY < rightMinY)
					minY = leftMinY;
				else
					minY = rightMinY;

				int intersectionY = (mRightLaneEdge*cLeftLaneEdge - mLeftLaneEdge*cRightLaneEdge) / (mRightLaneEdge - mLeftLaneEdge);

				if (intersectionY < minY)
				{
					blankFrame = cv::Mat::zeros(VIDEO_HEIGHT, VIDEO_WIDTH, frame.type());

					lanePoints.push_back(cv::Point((minY - cLeftLaneEdge) / mLeftLaneEdge, minY));
					lanePoints.push_back(cv::Point((minY - cRightLaneEdge) / mRightLaneEdge, minY));
					lanePoints.push_back(cv::Point((ROI_BOTTOM_HEIGHT - cRightLaneEdge) / mRightLaneEdge, ROI_BOTTOM_HEIGHT));
					lanePoints.push_back(cv::Point((ROI_BOTTOM_HEIGHT - cLeftLaneEdge) / mLeftLaneEdge, ROI_BOTTOM_HEIGHT));

					cv::fillConvexPoly(blankFrame, lanePoints, cv::Scalar(0, 64, 0), cv::LINE_AA, 0);

					cv::add(frame, blankFrame, frame);
				}
			}

			baseline = 0;
			textSize = cv::getTextSize(turningRequiredToReturnToCenter, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
			baseline += FONT_THICKNESS;
			textOrg = cv::Point((rightInfoRect.x + rightInfoRect.width / 2.) - textSize.width / 2., rightInfoRect.y + rightInfoRect.height + baseline - textSize.height);
			cv::putText(frame, turningRequiredToReturnToCenter, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);


			titleText = "Within Detected Lanes";
			rightInfoTitleText = "Detected Lanes";
			changingLanesPreviousDifference = 0;
			changingLanesframeCount = 0;
			currentTurningState.clear();
			break;
		}

		case 1:
		{
			if (middleLines.size() != 0)
			{
				for (i = averageDistanceFromLeft = averageDistanceFromRight = 0; i < middleLines.size(); i++)
				{
					leftY1 = (middleLines[i][1] - cLeftEdgeOfMask) / mLeftEdgeOfMask;
					leftY2 = (middleLines[i][3] - cLeftEdgeOfMask) / mLeftEdgeOfMask;

					averageDistanceFromLeft += std::fabs(middleLines[i][0] - leftY1);
					averageDistanceFromLeft += std::fabs(middleLines[i][2] - leftY2);

					rightY1 = (middleLines[i][1] - cRightEdgeOfMask) / mRightEdgeOfMask;
					rightY2 = (middleLines[i][3] - cRightEdgeOfMask) / mRightEdgeOfMask;

					averageDistanceFromRight += std::fabs(middleLines[i][0] - rightY1);
					averageDistanceFromRight += std::fabs(middleLines[i][2] - rightY2);
				}

				averageDistanceFromLeft /= (double)(middleLines.size() * 2);
				averageDistanceFromRight /= (double)(middleLines.size() * 2);

				changingLanesCurrentDifference = averageDistanceFromLeft - averageDistanceFromRight;
				if (changingLanesframeCount == 0)
					changingLanesPreviousDifference = changingLanesCurrentDifference;

				changingLanesframeCount++;

				if (changingLanesframeCount == FRAME_COUNT_THRESHOLD)
				{
					if ((changingLanesPreviousDifference - changingLanesCurrentDifference) == 0)
						currentTurningState = "(Currently Not Turning)";
					else if ((changingLanesPreviousDifference - changingLanesCurrentDifference) < 0)
						currentTurningState = "(Currently Turning Left)";
					else
						currentTurningState = "(Currently Turning Right)";

					if (changingLanesCurrentDifference != 0)
						changingLanesPreviousDifference = changingLanesCurrentDifference;

					changingLanesframeCount = 0;
				}
			}

			titleText = "WARNING: Changing lanes";
			rightInfoTitleText = "Detected Lanes";

			for (i = 0; i < middleLineTypesForDisplay.size(); i++)
				cv::rectangle(frame, cv::Rect(1695, 80 + i * 50, 4, 25 * middleLineTypesForDisplay[i]), cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);

			if (!currentTurningState.empty())
			{
				baseline = 0;
				textSize = cv::getTextSize(currentTurningState, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
				baseline += FONT_THICKNESS;
				textOrg = cv::Point(((double)VIDEO_WIDTH - (double)textSize.width) / 2., 125 + baseline + textSize.height);
				cv::rectangle(frame, textOrg + cv::Point(0, baseline), textOrg + cv::Point(textSize.width, -textSize.height - baseline), cv::Scalar(0), cv::FILLED);
				cv::putText(frame, currentTurningState, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);
			}
			break;
		}

		case 2:
		{
			titleText = "WARNING: Only left road marking detected";
			rightInfoTitleText = "Detected Lanes";
			changingLanesPreviousDifference = 0;
			changingLanesframeCount = 0;
			currentTurningState.clear();
			break;
		}

		case 3:
		{
			titleText = "WARNING: Only right road marking detected";
			rightInfoTitleText = "Detected Lanes";
			changingLanesPreviousDifference = 0;
			changingLanesframeCount = 0;
			currentTurningState.clear();
			break;
		}

		case 4:
		{
			titleText = "WARNING: No road markings detected";
			rightInfoTitleText = "No Lanes Detected";
			changingLanesPreviousDifference = 0;
			changingLanesframeCount = 0;
			currentTurningState.clear();
			break;
		}

		default:
			std::cout << "\ncurrentDrivingState switch statement error: " << drivingState;
			break;
		}



		baseline = 0;
		textSize = cv::getTextSize(titleText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		baseline += FONT_THICKNESS;
		textOrg = cv::Point(((double)VIDEO_WIDTH - (double)textSize.width) / 2., 25 + baseline + textSize.height);
		cv::rectangle(frame, textOrg + cv::Point(0, baseline), textOrg + cv::Point(textSize.width, -textSize.height - baseline), cv::Scalar(0), cv::FILLED);
		cv::putText(frame, titleText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);



		baseline = 0;
		textSize = cv::getTextSize(rightInfoTitleText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		baseline += FONT_THICKNESS;
		textOrg = cv::Point((rightInfoRect.x + rightInfoRect.width / 2.) - textSize.width / 2., rightInfoRect.y + baseline + textSize.height);
		cv::putText(frame, rightInfoTitleText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);



		cv::rectangle(frame, FPSRect, cv::Scalar(0), cv::FILLED);

		std::stringstream ss1;
		ss1 << std::fixed << std::setprecision(2) << previousAverageFPS;
		FPSText = "Average FPS: " + ss1.str();
		baseline = 0;
		textSize = cv::getTextSize(FPSText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		baseline += FONT_THICKNESS;
		textOrg = cv::Point(30, 25 + baseline + textSize.height);
		cv::putText(frame, FPSText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);

		std::stringstream ss2;
		ss2 << std::fixed << std::setprecision(2) << previousFPS;
		FPSText = "Current FPS: " + ss2.str();
		baseline = 0;
		textSize = cv::getTextSize(FPSText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
		baseline += FONT_THICKNESS;
		textOrg = cv::Point(30, 75 + baseline + textSize.height);
		cv::putText(frame, FPSText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);



		cv::rectangle(frame, recordOutputRect, cv::Scalar(0), cv::FILLED);

		if (recordOuput)
		{
			recordingOuputText = "Recording Output";
			baseline = 0;
			textSize = cv::getTextSize(recordingOuputText, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseline);
			baseline += FONT_THICKNESS;
			textOrg = cv::Point((recordOutputRect.x + recordOutputRect.width / 2.) - textSize.width / 2., recordOutputRect.y + baseline + textSize.height);
			cv::putText(frame, recordingOuputText, textOrg, FONT_FACE, FONT_SCALE, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);

			ouputFrameVideo << frame;
		}

		else
		{
			recordingOuputText = "Press 'r' to start recording";
			baseline = 0;
			textSize = cv::getTextSize(recordingOuputText, FONT_FACE, FONT_SCALE-0.2, FONT_THICKNESS, &baseline);
			baseline += FONT_THICKNESS;
			textOrg = cv::Point((recordOutputRect.x + recordOutputRect.width / 2.) - textSize.width / 2., recordOutputRect.y + baseline + textSize.height+5);
			cv::putText(frame, recordingOuputText, textOrg, FONT_FACE, FONT_SCALE-0.2, cv::Scalar::all(255), FONT_THICKNESS, cv::LINE_AA);
		}



		#ifdef __linux__
		cv::resize(frame, frame, cv::Size(1280, 720));
		#endif
		cv::imshow("frame", frame);

		char key = (char)cv::waitKey(1);
		if (key == 'r')
		{
			if (recordOuput == false)
			{
				recordOuput = true;

				time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
				std::stringstream ss;
				ss << std::put_time(localtime(&now), "%Y-%m-%d %H-%M-%S");
				std::string currentTime = ss.str();

				ouputFrameVideo.open("../media/" + currentTime + " Frame.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT), true);
				if (!ouputFrameVideo.isOpened())
				{
					std::cout << "\nError opening video writer object\n";
					recordOuput = false;
				}
			}
			else
				recordOuput = false;
		}
		else if (key == 'q')
			break;

		previousFPS = 1000 / (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
	}

	inputVideo.release();
	ouputFrameVideo.release();

	cv::destroyAllWindows();

	return 0;
}
