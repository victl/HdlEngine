#ifndef HDLCORRECTION_H
#define HDLCORRECTION_H
#include "../ugv-share/defines.h"
#include <string>
#include <fstream>


struct HdlCorrection
{
    HdlCorrection(const std::string fileName);

    float rotAngle[LASER_NUM];  //Correction of rotation angle for each laser. Corresponse to original 'm_rot'
    float vertAngle[LASER_NUM];  //vertical angle correction.Corresponse to original 'm_vert'
    float dist[LASER_NUM];  //distance system error, Corresponse to original m_dist
    float vertOffset[LASER_NUM]; //vertical offset. Corresponse to original m_z_off
    float horizOffset[LASER_NUM]; //horizantal offset. Corresponse to original m_x_off
    //S2 new features:
    float minIntensity[LASER_NUM]; //minIntensity. Corresponse to original m_min_i
    float maxIntensity[LASER_NUM]; //maxIntensity. Corresponse to original m_max_i
    float distX[LASER_NUM]; //distCorrectionX. Corresponse to original m_distX
    float distY[LASER_NUM]; //distCorrectionY. Corresponse to original m_distY
    float focalDist[LASER_NUM];  //focalDistance. Corresponse to original m_f_d
    float focalSlope[LASER_NUM]; //focalSlope. Corresponse to original m_f_s
    //for param calculation
    float cos_rotAngle[LASER_NUM];//Corresponse to original m_cos_rot
    float sin_rotAngle[LASER_NUM];//Corresponse to original m_sin_rot
    float cos_vertAngle[LASER_NUM];//Corresponse to original m_cos_vert
    float sin_vertAngle[LASER_NUM];//Corresponse to original m_sin_vert
    float cos_raw[ANGLE_NUM];//Corresponse to original m_cos_raw
    float sin_raw[ANGLE_NUM];//Corresponse to original m_sin_raw
    //for sorting
    int lasersort[LASER_NUM];//Corresponse to original rasersort
    int layermark[LASER_NUM];//usage uncertain
};

#endif // HDLCORRECTION_H
