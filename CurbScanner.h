/**
  * This class was originally defined by XIAO KE LAI TI
  * with modification by Zou Lu
  * Modify date: 2015-9-3
*/
#include "../ugv-share/defines.h"

#define PT_TYPE_UNINDENTIFIED 0
#define PT_TYPE_ROAD 1
#define PT_TYPE_CURB 2

#define LEFT 0
#define RIGHT 1
#define SCANRANGE 50

#define FIRSTSEGMENT 1
#define SECONDSEGMENT 2
#define LEASTWIDTH 5.0
#define LANEWIDTH    3

typedef struct HDLScale{
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double x_scale;
	double y_scale;
}HDLScale_t;

typedef struct Point_laser
{
	int pt_count;
	LPoint_t pt[HDL_MAX_POINT_LASER];
	int road_type[HDL_MAX_POINT_LASER];
	int pt_type[HDL_MAX_POINT_LASER];
}Point_laser_t;

typedef struct Point_int
{
	int pt_count;
	double x_int[HDL_MAX_POINT_LASER];
	double y_int[HDL_MAX_POINT_LASER];
	double z_int[HDL_MAX_POINT_LASER];
	double x2_int[HDL_MAX_POINT_LASER];
	double xy_int[HDL_MAX_POINT_LASER];
	double y2_int[HDL_MAX_POINT_LASER];
}Point_int_t;


typedef struct Point3D
{
	int x;
	int y;
	int z;
}Point3D_t;

class CurbScanner
{
public:
	CurbScanner();
	~CurbScanner();
	void Initialize();
	void ProcessFrame();

private:
	void detection_curb();
	void detection_road();
	void calc_int();
	void GetHDLImage();


	void Get_preview(void);
	void updateWidth(void);
	void puttext(void);

public:
	LPoint_t *m_cloud;
	Point_laser_t *m_point;
	int m_cloud_count;
	vector<Point3D_t> m_pt_left, m_pt_right;
	vector<CvPoint> m_l,m_r;
	IplImage *m_img;// ,*m_temp,*m_tempcolor,*hist;

private:
	HDLScale_t m_scale;
	Point_int_t *m_point_int;	
	int m_laser_index[LASER_NUM], m_laser_last;

	CvSize m_size;
};
