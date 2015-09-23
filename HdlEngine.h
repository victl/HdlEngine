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
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#define DEBUG

namespace victl {

class HdlEngine
{
public /*method*/:
    //if running online, initialize an instance of this class with empty parameter list
    explicit HdlEngine();
    //if running offline, initialize it with a HDL file pathname.
    //It is assumed that other related files ( like '.carposes', '.camerapoints' etc. ) were stored in the same dir with the '.hdl' file
    //don't break this rule.
    HdlEngine(const std::string hdlFileName);
    ~HdlEngine();
    bool initialize(const std::string hdlFileName);
    bool processNextFrame();

    //following two functions are mainly used for debuging
    bool saveFrame(Grid** frame, const Range& range, const std::string& name);
    bool visualLocalMap(const std::string& name);

    void saveLocalMap(const std::string name);


    bool write3bPng(const std::string fileName = "unamed-map.png", MapType type = LOCALMAP);
    //The following 3 methods are mainly designed for Wang Shi Yao's programs
    Grid** getAccumMap();
    const Range& getAccumMapRange();
    const Carpose& getCurrentPose();

private /*method*/:
    //read points from file
    inline bool readPointsFromFile();
#ifndef OFFLINE
    //read points from shared memory. This method is used when running online
    inline bool readPointsFromShm();
#endif
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


    //update local map
    inline bool updateLocalMap();
    //prepare local map. If current accumulated map can't fit into current local map, expand the local map
    inline bool adjustLocalMapSize();
    //update the corresponding region of local map with accumulated map
    inline bool updateRegion(cv::Mat region, Grid** accumMap);

    //write value to cv::Mat. We need this function because OpenCV treat top-left as origin
    //while our coordinate system's orignin is on bottom-left
    inline bool writeOOnMat(cv::Mat mat, int x, int y, unsigned char value);
    inline bool writeAOnMat(cv::Mat mat, int x, int y, unsigned char value);

    //for 3b format input & output
    inline Point3B get3b(unsigned short xx, unsigned short yy, MapType type);

private:
    //configuration parameters object:
    UgvParam params;

    //file name without file type extension, can be used to refer to .hdl, .dgps etc after appending file name extension.
    std::string baseFileName;

    //input stream for handling hdl file and carpose file, etc.
    std::ifstream hdlReader;
    std::ifstream carposeReader;
    std::ifstream cameraPointReader;

    //LiDAR correction parameters. will be read from a txt file by initialize with that file
    HdlCorrection correction;

    //the size of current frame's points
    int totalPointsNum;
    //container for current frame's points, its size is sufficiently large
    RawHdlPoint* rawHdlPoints;
    //container for coordinates (XYZs) of current frame's points, its size is sufficiently large
    HdlPointXYZ* hdlPointXYZs;
    //container for native hdl points. Used for online recording
    //and since 2015-9-10, offline data were also stored in this format.
    HdlPoint* hdlPointCloud;
    //car pose of current frame
    Carpose currentPose;

    ////Here, l..., r..., s... represent left lanemark, right lanemark, stopline points from camera vision.
    /// provided by Liu Xiao Long, via shared memory.
    bool lValid, rValid, sValid;
    int lnum, rnum, snum;
    SimpleCarpose* lpts;
    SimpleCarpose* rpts;
    SimpleCarpose* spts;

    //the counter to record how many frame have been processed, and indicate next frame's number
    size_t frameProcessedNum;

    //container to record all carposes
    std::vector<Carpose> carposes;

    //the range of dynamic map.
    Range dynamicMapRange;
    //dynamic map
    Grid** dynamicMap;
//    std::vector<Grid> dynamicMap;

    //accumulated map
    Grid** accumMap;
    Grid** newAccumMap; //a temporary storing area for merging dynamic map and accumulated map
//    std::vector<Grid> accumMap;
    //the range of accumulated map.
    Range accumMapRange;

    //local map
    cv::Mat localMap;
    //the range of local map
    Range localMapRange;

    //FOWLLOWING VARIABLES ARE FOR TESTING

};

}//end namespace victl
#endif // HDLENGINE_H
