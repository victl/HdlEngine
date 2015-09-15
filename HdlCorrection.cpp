#include "HdlCorrection.h"
#include <glog/logging.h>
#include <cmath>

namespace victl {

HdlCorrection::HdlCorrection(const std::string fileName)
    : blockedByHipAngle(4500)
{
    //the dat file is converted from db.xml, only have the array info
    //used parse_xml_db solution to convert xml to dat file
    std::ifstream fdb(fileName.c_str());
    if (!fdb)
    {
        DLOG(FATAL) << "Error reading ladar calibration file.";
    }

    for (int i = 0; i < LASER_NUM; i++)
    {
        lasersort[i]=i;
        fdb >> rotAngle[i] >> vertAngle[i] >> dist[i]
            >> vertOffset[i] >> horizOffset[i] >> minIntensity[i]
            >> maxIntensity[i] >> distX[i] >> distY[i]
            >> focalDist[i] >> focalSlope[i];
    }
    fdb.close();


    //resort each laser. Reason uncertain for Zou - to be clarified.
    for (int i=0;i<LASER_NUM-1;i++)
        for (int j=i+1;j<LASER_NUM;j++)
            if (vertAngle[lasersort[i]]>vertAngle[lasersort[j]])
            {
                int ts=lasersort[i];lasersort[i]=lasersort[j];lasersort[j]=ts;
            }

    for (int i=0;i<LASER_NUM;i++) layermark[lasersort[i]]=i;

    //regulate unit to mm
    for (int i = 0; i < LASER_NUM; i++)
    {
        dist[i]  *= 10;vertOffset[i] *= 10;horizOffset[i] *= 10;distX[i] *= 10;distY[i] *= 10;
    }

    for (int i = 0; i < LASER_NUM; i++)
    {
        cos_rotAngle[i] = cos( rotAngle[i] / 180.0f * M_PI);
        sin_rotAngle[i] = sin( rotAngle[i] / 180.0f * M_PI);
        cos_vertAngle[i] = cos( vertAngle[i] / 180.0f * M_PI);
        sin_vertAngle[i] = sin( vertAngle[i] / 180.0f * M_PI);
    }

    for (int i = 0; i < ANGLE_NUM; i++)
    {
        cos_raw[i] = cos( i / 18000.0f * M_PI);
        sin_raw[i] = sin( i / 18000.0f * M_PI);
    }
}

}//end namespace victl
