#ifndef HDLENGINE_H
#define HDLENGINE_H
#include "../ugv-share/defines.h"
#include "../ugv-share/config.h"
#include "HdlCorrection.h"
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>


class HdlEngine
{
public /*method*/:
    explicit HdlEngine();
    HdlEngine(const std::string hdlFileName);
    ~HdlEngine();
    bool initialize(const std::string hdlFileName);
    bool processNextFrame();

    //following two functions are mainly used for debuging
    bool saveFrame(const std::vector<Grid>& frame, int width, int height, const std::string& name);
#ifdef OFFLINE
    bool visualLocalMap(const std::string& name);
    void saveLocalMap(const std::string name);
#endif

    bool write3bPng(const std::string fileName = "unamed-map.png", MapType type = LOCALMAP);
    const std::vector<Grid>& getAccumMap();
    const Range& getAccumMapRange();

private /*method*/:
    //read points from file
    inline bool readPointsFromFile();
    //read points from shared memory. This method is used when running online
    inline bool readPointsFromShm();
    //XYZs are coordinates of each corresponding rawHdlPoints
    inline bool populateXYZ(RawHdlPoint *rawHdlPoints , HdlPointXYZ* hdlPointXYZs, int totalPointsNum);
    //Get the position of current frame on the global map (origin at Differential Station).
    //Returns a cv::Rect2f to represent left-bottom and top-right corner

    //given the interval between highest point and lowest point in the grid, calculate each grid's probability in dynamic map
    inline bool calcProbability();

    //following codes were inherited from XIAO KE, but they are not needed for the moment
    /*
    //using ray tracing algorithm to clean road area
    inline bool rayTracing(const Carpose& currentPose);
    //following two function were inherited from XIAO KE, mainly used by rayTracing()
    inline bool frontPoints(unsigned char angle, int axis, int MaxAngle, int MinAngle, std::map<int, std::set<double> > &anglemap);
    inline double likelihood(double d);
    inline unsigned char p2color(float p);
    */

    //update accumulated map
    inline bool updateAccumMap();

#ifdef OFFLINE
    //update local map
    inline bool updateLocalMap();
    //prepare local map. If current local map does not contain current accumulated map, expand it
    inline bool adjustLocalMapSize();
    //update the corresponding region of local map with accumulated map
    inline bool updateRegion(cv::Mat region, const std::vector<Grid>& accumMap);

    //write value to cv::Mat. We need this function because OpenCV treat top-left as origin
#endif
    //while our coordinate's orignin is on bottom-left
    inline bool writeOnMat(cv::Mat mat, int x, int y, unsigned char value);

    //3b format input & output
    inline Point3B get3b(unsigned short xx, unsigned short yy, MapType type);

private:
    //configuration object:
    UgvParam params;

    //file name without file type extension, can be used to refer to .hdl, .dgps etc after appending file name extension.
    std::string baseFileName;

    //input stream for handling hdl file and hdl_dgps file
    std::ifstream hdlInstream;
    std::ifstream carposeInstream;

    //LiDAR correction parameters. will be read from a txt file by initialize with that file
    HdlCorrection correction;

    //the size of current frame's points
    int totalPointsNum;
    //container for current frame's points, its size is sufficiently large
    RawHdlPoint rawHdlPoints[MAX_CLOUD_NUM];
    //container for coordinates (XYZs) of current frame's points, its size is sufficiently large
    HdlPointXYZ hdlPointXYZs[MAX_CLOUD_NUM];
    //car pose of current frame
    Carpose currentPose;

    //the counter to record how many frame have been processed, and indicate next frame's number
    size_t frameProcessedNum;

    //container to record all carposes
    std::vector<Carpose> carposes;

    //the range of dynamic map.
    Range dynamicMapRange;
    //dynamic map
    std::vector<Grid> dynamicMap;

    //accumulated map
    std::vector<Grid> accumMap;
    //the range of accumulated map.
    Range accumMapRange;

#ifdef OFFLINE
    //local map
    cv::Mat localMap;
    //the range of local map
    Range localMapRange;
#endif

};

#endif // HDLENGINE_H
