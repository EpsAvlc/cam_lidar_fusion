/*
 * Created on Tue Apr 16 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#ifndef POINTXYZPIXEL_H_
#define POINTXYZPIXEL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct PointXYZPIXEL{
	PCL_ADD_POINT4D
		; // quad-word XYZ
	int pixel_x; // pixel coor x
	int pixel_y; ///< laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
		PointXYZPIXEL, (float, x, x) (float, y, y) (float, z, z) (int, pixel_x, pixel_x) (int, pixel_y, pixel_y))

#endif // !POINTXYZPIXEL_H_
