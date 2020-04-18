
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-100, bottom+40), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-100, bottom+80), cv::FONT_ITALIC, 1, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    const float windowScale = 0.5;
    string windowName = "3D Objects";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, imageSize.width * windowScale, imageSize.height * windowScale);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}

void sortLidarPoints(std::vector<LidarPoint> &lidarPoints)
{
    std::sort(lidarPoints.begin(), lidarPoints.end(), [](LidarPoint a, LidarPoint b) {
        return a.x < b.x;  // Sort ascending on the x coordinate only
    });
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
	// sort lidar points
	sortLidarPoints(lidarPointsPrev);
	sortLidarPoints(lidarPointsCurr);
	// take medium values of x
	double d0 = lidarPointsPrev[lidarPointsPrev.size()/2].x;
	double d1 = lidarPointsCurr[lidarPointsCurr.size()/2].x;

	TTC = d1 * (1.0 / frameRate) / (d0 - d1);
}

void printConnections(std::map<int, std::map<int, int>> connections);

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::map<int, std::map<int, int>> connections {};  // map<prevBoxID, map<curBoxID, nConn>>, nConn - number of shared keypoints between curBox and prevBox

    for (auto kpMatch : matches) {
    	cv::KeyPoint prevKp = prevFrame.keypoints[kpMatch.queryIdx];
    	cv::KeyPoint curKp = currFrame.keypoints[kpMatch.trainIdx];

    	bool onlyOneFound=false;
    	BoundingBox prevBox;
    	for (auto bb: prevFrame.boundingBoxes) {
    		if (bb.roi.contains(prevKp.pt)) {
    			if (!onlyOneFound) {
    				// keypoint is in the box and its the first one
    				prevBox = bb;
    				onlyOneFound = true;
    			} else {
    				// keypoint belong to more than one box
    				onlyOneFound = false;
    				break; // stop looking at other boxes
    			}
    		}
    	}

    	if (onlyOneFound) {
    		// found bounding box for prevKp and its the only one.

    		onlyOneFound = false;
    		BoundingBox curBox;
			for (auto bb: currFrame.boundingBoxes) {
				if (bb.roi.contains(curKp.pt)) {
					if (!onlyOneFound) {
						// keypoint is in the box and its the first one
						curBox = bb;
						onlyOneFound = true;
					} else {
						// keypoint belong to more than one box
						onlyOneFound = false;
						break; // stop looking at other boxes
					}
				}
			}

			if (onlyOneFound) {
				// both kp belong to unique boxes in their respective frames
				++connections[prevBox.boxID][curBox.boxID];
			}
    	}

    	//printConnections(connections);

    	// Find best matches
    	int nConnMax;
    	int bestCurBox;
    	std::set<int> assignedCurBoxes;
    	for (auto conn : connections) {
    		int prevBoxId = conn.first;
    		nConnMax = 0;
    		bestCurBox = -1;
    		for (auto boxConn : conn.second) { // @suppress("Symbol is not resolved")
    			int curBoxId = boxConn.first;
    			int nConn = boxConn.second;
    			if ( (nConn > nConnMax) && (assignedCurBoxes.find(curBoxId) == assignedCurBoxes.end())) {
    				// curBox has more connections than the best choice so far and isn't assigned yet
    				nConnMax = nConn;
    				bestCurBox = curBoxId;
    			}
    		}
    		if (bestCurBox != -1) {
    			bbBestMatches[prevBoxId] = bestCurBox;
    			assignedCurBoxes.insert(bestCurBox);
    		}
    	}


    }
}

void printConnections(std::map<int, std::map<int, int>> connections) {
	cout << endl << "Connections map printout:" << endl;
	for (auto it1=connections.begin(); it1!=connections.end(); ++it1){
		cout << "\tprevBoxId: " << it1->first << " | nAssociates = " << it1->second.size() << endl;
		for (auto it2=it1->second.begin(); it2!=it1->second.end(); ++it2)
			cout << "\t\tcurBoxId: " << it2->first << "| nconn = " << it2->second << endl;
	}
	cout << "" << endl;
}



