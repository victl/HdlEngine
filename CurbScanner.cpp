/**
  * This class was originally defined by XIAO KE LAI TI
  * with modification by Zou Lu
  * Modify date: 2015-9-3
*/

#include "CurbScanner.h"
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/legacy/compat.hpp>
#include<vector>
#include<set>
#include <cstdlib>


ofstream ftemp("c_temp.txt");
ofstream fout("c_out.txt");
ofstream finit("c_init.txt");
ofstream fpreview("c_preview.txt");

extern Carpose m_vehicle_pos;
extern double m_preview_dist;
extern double m_vehicle_speed;
double CurbWidth;

int is_equal(const void* _a, const void* _b, void* userdata){
	CvScalar a = *(const CvScalar*)_a;
	CvScalar b = *(const CvScalar*)_b;
	double y = *(double*)userdata;

	double x_a = a.val[0] * y + a.val[1];
	double x_b = b.val[0] * y + b.val[1];

	double d_angle = fabs(a.val[0] - b.val[0]);
	double d_dist = fabs(x_a - x_b);

	return (d_angle < 0.1 && d_dist < 10);
}
CurbScanner::CurbScanner()
{
	m_cloud = NULL;
	m_point = NULL;
	m_point_int = NULL;
	m_img = NULL;	
}

CurbScanner::~CurbScanner()
{
	if (m_cloud)
	{
		delete[] m_cloud;
		m_cloud = NULL;
	}
	if (m_point)
	{
		delete[] m_point;
		m_point = NULL;
	}
	if (m_point_int)
	{
		delete[] m_point_int;
		m_point_int = NULL;
	}
	if (m_img)
	{
		cvReleaseImage(&m_img);
		m_img = NULL;
	}
}

void CurbScanner::Initialize()
{
	srand((unsigned)time(NULL));
	m_cloud = new LPoint_t[HDL_MAX_POINT_NUMBER];
	m_cloud_count = 0;
    m_point = new Point_laser_t[LASER_NUM];
    m_point_int = new Point_int_t[LASER_NUM];
	m_laser_last = 45;

	m_size = cvSize(500, 500);
	m_img = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
	m_scale.x_max = 12.5;
	m_scale.x_min = -12.5;
	m_scale.y_max = 25;
	m_scale.y_min = 0;
	m_scale.x_scale = m_size.width / (m_scale.x_max - m_scale.x_min);    // 水平方向每一米的像素数目
	m_scale.y_scale = m_size.height / (m_scale.y_max - m_scale.y_min);   // 垂直方向每一米的像素数目
	

	int temp[] = {39, 40, 43, 44, 33, 34, 37, 38, 41, 42,
	              47, 48, 51, 52, 55, 56, 45, 46, 49, 50,
	              53, 54, 59, 60, 63, 64, 35, 36, 57, 58,
	              61, 62, 7, 8, 11, 12, 1, 2, 5, 6,
	              9, 10, 15, 16, 19, 20, 23, 24, 13, 14,
	              17, 18, 21, 22, 27, 28, 31, 32, 3, 4,
	              25, 26, 29, 30
	             };

    for (int i = 0; i < LASER_NUM; i++)
		m_laser_index[i] = temp[i];
	
//	m_temp = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
}

void CurbScanner::ProcessFrame()
{
	int i;
    int c, count[LASER_NUM];

	m_pt_left.clear();
	m_pt_right.clear();

	//Store all the points according to their c
    for (i = 0; i < LASER_NUM; i++)
		count[i] = 0;

	for (i = 0; i < m_cloud_count; i++)
	{
		c = m_cloud[i].c;
		m_point[c].pt[count[c]].x = m_cloud[i].x;
		m_point[c].pt[count[c]].y = m_cloud[i].y;
		m_point[c].pt[count[c]].z = m_cloud[i].z;
		m_point[c].pt[count[c]].dist = m_cloud[i].dist;
		m_point[c].pt[count[c]].i = m_cloud[i].i;
		m_point[c].pt[count[c]].c = c;
		m_point[c].pt[count[c]].rot = m_cloud[i].rot;
		m_point[c].road_type[count[c]] = false;
        m_point[c].pt_type[count[c]] = UNKNOWN;
		count[c]++;
	}
    for (i = 0; i < LASER_NUM; i++)
	{
		m_point[i].pt_count = count[i];
	}

	//Sort the 64 lasers from near to far
    Point_laser* pl = new Point_laser[LASER_NUM];
    for (int i = 0; i < LASER_NUM; i++)
	{
		pl[i] = m_point[m_laser_index[i] - 1];
	}
    for (int i = 0; i < LASER_NUM; i++)
	{
		m_point[i] = pl[i];
	}
	delete[] pl;
	m_l.clear();
	m_r.clear();
	calc_int();
	detection_curb();
	detection_road();
	GetHDLImage();
}

void CurbScanner::GetHDLImage()
{
	double x, y;
	int col, row;

	cvZero(m_img);

	for (int i = 0; i < (int)m_pt_left.size(); i++)
	{
		x = (double)m_pt_left[i].x / 1000;
		y = (double)m_pt_left[i].y / 1000;
		col = (int)((x - m_scale.x_min) * m_scale.x_scale);
		row = m_size.height - (int)((y - m_scale.y_min) * m_scale.y_scale);
		if (col >= 0 && col < m_size.width && row >= 0 && row < m_size.height)
		{
			m_img->imageData[row * m_size.width + col] = 255;
		}
		m_l.push_back(cvPoint(col*2,row*2-m_size.height));
	}
	for (int i = 0; i < (int)m_pt_right.size(); i++)
	{
		x = (double)m_pt_right[i].x / 1000;
		y = (double)m_pt_right[i].y / 1000;
		col = (int)((x - m_scale.x_min) * m_scale.x_scale);
		row = m_size.height - (int)((y - m_scale.y_min) * m_scale.y_scale);
		if (col >= 0 && col < m_size.width && row >= 0 && row < m_size.height)
		{
			m_img->imageData[row * m_size.width + col] = 255;
		}
		m_r.push_back(cvPoint(col*2,row*2-m_size.height));
	}
}

void CurbScanner::calc_int()
{
	double x, y, z, x2, xy, y2;
	bool bStart;

	for (int i = 5; i < m_laser_last; i++)  // m_laser_last = 45
	{
		m_point_int[i].pt_count = m_point[i].pt_count;
		bStart = false;

		for (int j = 1; j < m_point_int[i].pt_count; j++)
		{
			if (!bStart)
			{
				if (m_point[i].pt[j-1].y < 0 && m_point[i].pt[j].y >= 0)
				{
					m_point_int[i].x_int[j] = (double)m_point[i].pt[j].x / 1000;
					m_point_int[i].y_int[j] = (double)m_point[i].pt[j].y / 1000;
					m_point_int[i].z_int[j] = (double)m_point[i].pt[j].z / 1000;
					m_point_int[i].x2_int[j] = (double)(m_point[i].pt[j].x * m_point[i].pt[j].x) / (1000 * 1000);
					m_point_int[i].xy_int[j] = (double)(m_point[i].pt[j].x * m_point[i].pt[j].y) / (1000 * 1000);
					m_point_int[i].y2_int[j] = (double)(m_point[i].pt[j].y * m_point[i].pt[j].y) / (1000 * 1000);
					bStart = true;
				}
				else
					continue;
			}

			if (m_point[i].pt[j].rot > 0 && m_point[i].pt[j].y < 0)
				break;

			x = (double)m_point[i].pt[j].x / 1000;
			y = (double)m_point[i].pt[j].y / 1000;
			z = (double)m_point[i].pt[j].z / 1000;
			x2 = (double)(m_point[i].pt[j].x * m_point[i].pt[j].x) / (1000 * 1000);
			xy = (double)(m_point[i].pt[j].x * m_point[i].pt[j].y) / (1000 * 1000);
			y2 = (double)(m_point[i].pt[j].y * m_point[i].pt[j].y) / (1000 * 1000);
			m_point_int[i].x_int[j] = m_point_int[i].x_int[j-1] + x;
			m_point_int[i].y_int[j] = m_point_int[i].y_int[j-1] + y;
			m_point_int[i].z_int[j] = m_point_int[i].z_int[j-1] + z;
			m_point_int[i].x2_int[j] = m_point_int[i].x2_int[j-1] + x2;
			m_point_int[i].xy_int[j] = m_point_int[i].xy_int[j-1] + xy;
			m_point_int[i].y2_int[j] = m_point_int[i].y2_int[j-1] + y2;
		}
	}
}

void CurbScanner::detection_curb()
{
	int nWindowLen = 6;
	bool bStart = false;
	int y_min = 4000, x_max = 20000;
	int min_intensity, max_intensity;
	double thresh_a = 0.2, thresh_diff = 0.001;
	double x_sum, y_sum, x2_sum, xy_sum, y2_sum, a, b, diff;
	Point3D_t pt;
	int index_right = 0, index_left = 0;

	for (int i = 5; i < m_laser_last; i++)
	{
		index_left = index_right = 0;
		for (int j = 0; j < m_point[i].pt_count - 1; j++)
		{
			if (m_point[i].pt[j].rot > m_point[i].pt[j+1].rot)
			{
				index_right = j+1;
				index_left = j;
				break;
			}
		}

		for (int j = index_right; j < m_point[i].pt_count - nWindowLen; j++)
		{
			m_point[i].pt_type[j] = PT_TYPE_ROAD;

			if (m_point[i].pt[j].y < y_min || m_point[i].pt[j].x > x_max)
				break;

			x_sum = m_point_int[i].x_int[j+nWindowLen] - m_point_int[i].x_int[j];
			y_sum = m_point_int[i].y_int[j+nWindowLen] - m_point_int[i].y_int[j];
			x2_sum = m_point_int[i].x2_int[j+nWindowLen] - m_point_int[i].x2_int[j];
			xy_sum = m_point_int[i].xy_int[j+nWindowLen] - m_point_int[i].xy_int[j];
			y2_sum = m_point_int[i].y2_int[j+nWindowLen] - m_point_int[i].y2_int[j];

			a = (y_sum * x_sum - nWindowLen * xy_sum) / (y_sum * y_sum - nWindowLen * y2_sum);
			b = (y_sum * xy_sum - y2_sum * x_sum) / (y_sum * y_sum - nWindowLen * y2_sum);
			diff = x2_sum - 2*a*xy_sum - 2*b*x_sum + a*a*y2_sum + 2*a*b*y_sum + nWindowLen*b*b;

			min_intensity = max_intensity = m_point[i].pt[j+1].i;
			for (int k = j+2; k < j+nWindowLen+1; k++)
			{
				if (m_point[i].pt[k].i < min_intensity)
					min_intensity = m_point[i].pt[k].i;

				if (m_point[i].pt[k].i > max_intensity)
					max_intensity = m_point[i].pt[k].i;
			}
			if (fabs(a) <= thresh_a && diff <= thresh_diff && (max_intensity - min_intensity) >= 10)
			{
				for (int k = j+1; k < j+nWindowLen+1; k++)
				{
					pt.x = m_point[i].pt[k].x;
					pt.y = m_point[i].pt[k].y;
					pt.z = m_point[i].pt[k].z;
					m_pt_right.push_back(pt);
					m_point[i].pt_type[k] = PT_TYPE_CURB;
				}
				break;
			}
		}

		bStart = false;
		for (int j = index_left; j >= nWindowLen; j--)
		{
			m_point[i].pt_type[j] = PT_TYPE_ROAD;

			if (m_point[i].pt[j].y < y_min || m_point[i].pt[j].x < -x_max)
				break;

			x_sum = m_point_int[i].x_int[j] - m_point_int[i].x_int[j-nWindowLen];
			y_sum = m_point_int[i].y_int[j] - m_point_int[i].y_int[j-nWindowLen];
			x2_sum = m_point_int[i].x2_int[j] - m_point_int[i].x2_int[j-nWindowLen];
			xy_sum = m_point_int[i].xy_int[j] - m_point_int[i].xy_int[j-nWindowLen];
			y2_sum = m_point_int[i].y2_int[j] - m_point_int[i].y2_int[j-nWindowLen];

			a = (y_sum * x_sum - nWindowLen * xy_sum) / (y_sum * y_sum - nWindowLen * y2_sum);
			b = (y_sum * xy_sum - y2_sum * x_sum) / (y_sum * y_sum - nWindowLen * y2_sum);
			diff = x2_sum - 2*a*xy_sum - 2*b*x_sum + a*a*y2_sum + 2*a*b*y_sum + nWindowLen*b*b;

			if (fabs(a) <= thresh_a && diff <= thresh_diff)
			{
				for (int k = j; k > j-nWindowLen; k--)
				{
					pt.x = m_point[i].pt[k].x;
					pt.y = m_point[i].pt[k].y;
					pt.z = m_point[i].pt[k].z;
					m_pt_left.push_back(pt);
					m_point[i].pt_type[k] = PT_TYPE_CURB;
				}
				break;
			}
		}
	}
}

void CurbScanner::detection_road()
{
	int count_noroad, nWindowLen = 20, nCurbLen = 5;
	double nAvgHeight, z, thresh_diff = 0.03;
	int y_min = 4000, x_max = 20000;
	Point3D_t pt;
	int index_left = 0, index_right = 0;
	int max_height, min_height;

	for (int i = 5; i < m_laser_last; i++)
	{
		index_left = index_right = 0;
		for (int j = 0; j < m_point[i].pt_count - 1; j++)
		{
			if (m_point[i].pt[j].rot > m_point[i].pt[j+1].rot)
			{
				index_right = j+1;
				index_left = j;
				break;
			}
		}

		while(1)
		{
			if(index_right<HDL_MAX_POINT_LASER)
			  max_height = min_height = m_point[i].pt[index_right].z;
			for (int j = index_right + 1; j < index_right + nWindowLen; j++)
			{
				//bool tic = m_point[i].pt[j].z > max_height;
				if ((j<HDL_MAX_POINT_LASER)&&(m_point[i].pt[j].z > max_height))
					max_height = m_point[i].pt[j].z;

				if ((j<HDL_MAX_POINT_LASER)&&(m_point[i].pt[j].z < min_height))
					min_height = m_point[i].pt[j].z;
			}
			if (max_height < -1800 && min_height > -2300 && (max_height - min_height) < 50)
				break;
			index_right++;
			if (index_right >= m_point[i].pt_count - nWindowLen - 1)
				break;
			if(index_right>HDL_MAX_POINT_LASER)
				break;
		}
		if (index_right == m_point[i].pt_count - nWindowLen - 1)
			continue;

		while(1)
		{
			if(index_left<HDL_MAX_POINT_LASER&&index_left>=0)
			    max_height = min_height = m_point[i].pt[index_left].z;
			for (int j = index_left - 1; j > index_left - nWindowLen; j--)
			{
				if ((j<HDL_MAX_POINT_LASER)&&(j>=0)&&(m_point[i].pt[j].z > max_height))
					max_height = m_point[i].pt[j].z;

				if ((j<HDL_MAX_POINT_LASER)&&(j>=0)&&(m_point[i].pt[j].z < min_height))
					min_height = m_point[i].pt[j].z;
			}
			if (max_height < -1800 && min_height > -2300 && (max_height - min_height) < 50)
				break;
			index_left--;
			if (index_left <= nWindowLen - 1)
				break;

			if(index_left>HDL_MAX_POINT_LASER ||index_left<0)
				break;
		}
		if (index_left == nWindowLen - 1)
			continue;

		//Detect right road surface
		count_noroad = 0;
		for (int j = index_right; j < index_right + nWindowLen; j++)
			m_point[i].road_type[j] = PT_TYPE_ROAD;

		for (int j = index_right + nWindowLen; j < m_point[i].pt_count; j++)
		{
			if (m_point[i].pt[j].y < y_min || m_point[i].pt[j].x > x_max)
				break;

			nAvgHeight = (m_point_int[i].z_int[j] - m_point_int[i].z_int[j-nWindowLen]) / nWindowLen;
			z = (double)m_point[i].pt[j].z / 1000;
			if (fabs(z - nAvgHeight) <= thresh_diff)
			{
				m_point[i].road_type[j] = PT_TYPE_ROAD;
				count_noroad = 0;
			}
			else
			{
				count_noroad++;
				if (count_noroad >= nCurbLen)
				{
					for (int k = 0; k < nCurbLen; k++)
					{
						pt.x = m_point[i].pt[j-k].x;
						pt.y = m_point[i].pt[j-k].y;
						pt.z = m_point[i].pt[j-k].z;
						m_pt_right.push_back(pt);
						m_point[i].road_type[j-k] = PT_TYPE_CURB;
					}
					break;
				}
			}
		}

		//Detect left road surface
		count_noroad = 0;
		for (int j = index_left; j > index_left - nWindowLen; j--)
			m_point[i].road_type[j] = PT_TYPE_ROAD;

		for (int j = index_left - nWindowLen; j >= 1; j--)
		{
			if (m_point[i].pt[j].y < y_min || m_point[i].pt[j].x < -x_max)
				break;

			nAvgHeight = (m_point_int[i].z_int[j+nWindowLen] - m_point_int[i].z_int[j]) / nWindowLen;
			z = (double)m_point[i].pt[j].z / 1000;
			if (fabs(z - nAvgHeight) <= thresh_diff)
			{
				m_point[i].road_type[j] = PT_TYPE_ROAD;
				count_noroad = 0;
			}
			else
			{
				count_noroad++;
				if (count_noroad >= nCurbLen)
				{
					for (int k = 0; k < nCurbLen; k++)
					{
						pt.x = m_point[i].pt[j+k].x;
						pt.y = m_point[i].pt[j+k].y;
						pt.z = m_point[i].pt[j+k].z;
						m_pt_left.push_back(pt);
						m_point[i].road_type[j+k] = PT_TYPE_CURB;
					}
					break;
				}
			}
		}
	}
}
