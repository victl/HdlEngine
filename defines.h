/**
  * This header file contains many preprocessor defines and some fundamental types.
  * Most of them do not change unless the physical model of HDL Lader changed.
  * Adjustable parameters should be put in config.h file.
  *
  * Author: Zou Lu (victl@163.com)
  * Date: 2015-9-1
  */


#ifndef DEFINES_H
#define DEFINES_H

//define some constant
#define ANGLE_NUM 36000 //divide the whole 360 degree to small pieces
#define LASER_NUM 64    //the total num of laser beam from the ladar is 64

//struct used to store raw hdl data
typedef struct
{
    unsigned short distance;    //corresponse to original 'dist'
    unsigned short rotAngle;   //corresponse to original 'rot'
    unsigned char intensity;    //corresponse to original 'i'
    unsigned char beamId;    //corresponse to original 'c'
}RawHdlPoint;

typedef struct{
    int x;
    int y;
    int z;
}HdlPointXYZ;

typedef struct{
    double x;
    double y;
    double eulr;
}Carpose;


#endif // DEFINES_H

