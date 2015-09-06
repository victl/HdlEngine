/**
  * This header file contains Adjustable parameters that could change between different run.
  * These parameters could be read from a configuration file, so changes could be made within
  * that file and take effect without recompiling the whole program.
  *
  * Author: Zou Lu (victl@163.com)
  * Date: 2015-9-1
  */


#ifndef CONFIG_H
#define CONFIG_H

#include <string>

struct UgvParam {
//public:
    UgvParam(){
        restoreDefault();
        loadParam();
    }

    bool loadParam(std::string configFile = "/home/victor/workspace/qt/UgvMap/ugv.conf");
    void restoreDefault();
    //DivideCarTrack
    struct DivideCarTrack_t {
//    public:
        double EulrChangeThreshold;
        int DetectPoints;
        int DetectDistance;
        int ValidSegmentPointsNum;
    };
    struct DivideCarTrack_t DivideCarTrack;

    //LineParallel
    struct LineParallel_t {
        double SimilarEulrThreshold;
    };
    struct LineParallel_t LineParallel;

    struct SameSeg_t {
        double SameDirectionThreshold;
        double LateralDistanceThreshold;
    };
    struct SameSeg_t SameSeg;
};

#endif // CONFIG_H
