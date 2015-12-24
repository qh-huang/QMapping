#ifndef DEBUGVIZ_H
#define DEBUGVIZ_H

#define _USE_MATH_DEFINES
#include "opencv2/opencv.hpp"
#include <cmath>
#include <gmapping/scanmatcher/smmap.h>

struct DebugViz
{
	inline DebugViz();
	inline ~DebugViz();

	inline void showGridMap(GMapping::ScanMatcherMap smmap);
	inline void showRobotPose(GMapping::ScanMatcherMap smmap, GMapping::OrientedPoint pose, int* colorRGB);
	inline void showLaserBeam(GMapping::ScanMatcherMap smmap, GMapping::OrientedPoint pose, double measure, int* colorRGB);

	inline void setGridResolution(double resolution) { _grid_resolution = resolution; }

	inline void resetGridMap();
	inline void resetGridMap(size_t size_x, size_t size_y) { _sizeX = size_x; _sizeY = size_y;	resetGridMap(); }

	inline void showGridMap(double ** gridmap);
	inline void showGridMap(size_t size_x, size_t size_y, double ** gridmap) { _sizeX = size_x; _sizeY = size_y; showGridMap(gridmap); }

	inline void showRobotPose(double* pose2D_XYA, int* colorRGB);
	inline void showLaserBeam(double* pRobotXYA, double* pIrXYA, double length, int* colorRGB);

	inline void setWorldCenter(double x, double y){ _world_center_x = x; _world_center_y = y; }

	size_t _grid_size;			// in pixel
	double _grid_resolution;	// meter in each grid
	size_t _line_width;			// in pixel

	cv::Mat _matMap;
	size_t _sizeX, _sizeY;		// in num of grids
	double _occ_thresh;

	double _world_center_x, _world_center_y;
};

inline DebugViz::DebugViz()
{
	// parameter default value
	_grid_size = 4;
	_line_width = 2;
	_grid_resolution = 0.1; // in meter

	_sizeX = 20;
	_sizeY = 20;

	_world_center_x = _world_center_y = 0;

	_occ_thresh = 0.5;
	_matMap = cv::Mat(_sizeY * (_grid_size + _line_width), _sizeX * (_grid_size + _line_width), CV_8UC3, cv::Scalar(0));
}

inline DebugViz::~DebugViz()
{
	_matMap.release();
	cv::destroyAllWindows();
}

inline void DebugViz::showGridMap(GMapping::ScanMatcherMap smmap)
{
	resetGridMap(smmap.getMapSizeX(), smmap.getMapSizeY());
	cv::Point pt1, pt2; // 2 endpoint of a grid
	for (size_t x = 0; x < _sizeX; x++) {
		for (size_t y = 0; y < _sizeY; y++) {
			pt1.x = x * (_grid_size + _line_width);
			pt1.y = y * (_grid_size + _line_width);
			pt2.x = pt1.x + _grid_size;
			pt2.y = pt1.y + _grid_size;
			GMapping::IntPoint p(x, (_sizeY -1) - y);
			double occ = smmap.cell(p);
			if (occ < 0) { // cell = -1 stands for unknown
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(127, 127, 127), CV_FILLED);
			}
			else if (occ > _occ_thresh) {
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(0, 0, 0), CV_FILLED);
			}
			else { // free
				double gray_level = 255.0*(1.0 - occ / _occ_thresh);
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(gray_level, gray_level, gray_level), CV_FILLED);
			}
		}
	}
	cv::imshow("DebugViz", _matMap);
}

inline void DebugViz::showRobotPose(GMapping::ScanMatcherMap smmap, GMapping::OrientedPoint pose, int* colorRGB)
{
	double pose2D_XYA[3];
	pose2D_XYA[0] = smmap.world2map(pose).x;
	pose2D_XYA[1] = smmap.world2map(pose).y;
	pose2D_XYA[2] = pose.theta;
	cv::Point2d __rcp, _rcp, rcp;	// robot center point (__world, _map, pixel)
	//__rcp.x = pose2D_XYA[0];
	//__rcp.y = pose2D_XYA[1];
	_rcp.x = pose2D_XYA[0]; 
	_rcp.y = (_sizeY-1)-pose2D_XYA[1]; 
	rcp.x = _rcp.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	rcp.y = _rcp.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	double robot_radius = 0.3; // in meter
	cv::circle(_matMap, rcp, (_grid_size + _line_width) * robot_radius / _grid_resolution,
		cv::Scalar(colorRGB[2], colorRGB[1], colorRGB[0]), 2);

	cv::Point2d __rhd; // robot head: meter 
	cv::Point2d _rhd; // robot head: pixel 
	cv::Point2d rhd; // robot head: pixel 
	//double theta = pose2D_XYA[2] + M_PI_2;
	double theta = pose2D_XYA[2]; // robot head toward X-axis
	_rhd.x = _rcp.x + (robot_radius + 0.02) * cos(theta) / _grid_resolution;
	_rhd.y = _rcp.y - (robot_radius + 0.02) * sin(theta) / _grid_resolution;
	//_rhd.x = (__rhd.x) / _grid_resolution;
	//_rhd.y = (__rhd.y) / _grid_resolution;
	rhd.x = _rhd.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	rhd.y = _rhd.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
	cv::line(_matMap, rhd, rcp,
		cv::Scalar(colorRGB[2], colorRGB[1], colorRGB[0]), 2);

	cv::imshow("DebugViz", _matMap);
}

inline void DebugViz::showLaserBeam(GMapping::ScanMatcherMap smmap, GMapping::OrientedPoint pose, double measure, int* colorRGB)
{
	double pose2D_XYA[3];
	pose2D_XYA[0] = smmap.world2map(pose).x;
	pose2D_XYA[1] = smmap.world2map(pose).y;
	pose2D_XYA[2] = pose.theta;
	cv::Point2d _bsp, bsp;	// beam start point (__world, _map, pixel)
	_bsp.x = pose2D_XYA[0];
	_bsp.y = (_sizeY - 1) - pose2D_XYA[1];
	bsp.x = _bsp.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	bsp.y = _bsp.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);

	GMapping::Point __bep; // beam end point (world)
	cv::Point2d _bep, bep;
	double theta = pose2D_XYA[2]; // robot head toward X-axis
	__bep.x = pose.x + measure * cos(theta);
	__bep.y = pose.y + measure * sin(theta);
	_bep.x = smmap.world2map(__bep).x;
	_bep.y = (_sizeY - 1) - smmap.world2map(__bep).y;
	bep.x = _bep.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	bep.y = _bep.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
	cv::line(_matMap, bsp, bep,	cv::Scalar(colorRGB[0], colorRGB[1], colorRGB[2]), 2);

	cv::imshow("DebugViz", _matMap);
	//cv::waitKey();
}

inline void DebugViz::resetGridMap()
{
	_matMap.create(_sizeY * (_grid_size + _line_width), _sizeX * (_grid_size + _line_width), CV_8UC3);
	_matMap.setTo(cv::Scalar(0, 0, 0));
}

inline void DebugViz::showGridMap(double** gridmap)
{
	cv::Point pt1, pt2; // 2 endpoint of a grid (pixel)
	for (size_t x = 0; x < _sizeX; x++) {
		for (size_t y = 0; y < _sizeY; y++){
			pt1.x = x * (_grid_size + _line_width);
			pt1.y = y * (_grid_size + _line_width);
			pt2.x = pt1.x + _grid_size;
			pt2.y = pt1.y + _grid_size;
			double occ = gridmap[x][(_sizeY - 1) - y];
			if (occ < 0) { // cell = -1 stands for unknown
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(127, 127, 127), CV_FILLED);
			} else if (occ > _occ_thresh) {
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(0, 0, 0), CV_FILLED);
			} else { // free
				double gray_level = 255.0*(1.0 - occ/_occ_thresh);
				cv::rectangle(_matMap, pt1, pt2, cv::Scalar(gray_level, gray_level, gray_level), CV_FILLED);
			}
		}
	}
	cv::imshow("DebugViz", _matMap);
}

inline void DebugViz::showRobotPose(double* pose2D_XYA, int* colorRGB)
{
	cv::Point2d __rcp, _rcp, rcp;	// robot center point (__world, _map, pixel)
	__rcp.x = pose2D_XYA[0];
	__rcp.y = pose2D_XYA[1];
	_rcp.x = ((double)_sizeX / 2.0) + (__rcp.x - _world_center_x) / _grid_resolution;
	_rcp.y = ((double)_sizeY / 2.0) - (__rcp.y - _world_center_y) / _grid_resolution;
	rcp.x = _rcp.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	rcp.y = _rcp.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	double robot_radius = 0.3; // in meter
	cv::circle(_matMap, rcp, (_grid_size + _line_width) * robot_radius / _grid_resolution,
		cv::Scalar(colorRGB[0], colorRGB[1], colorRGB[2]), 2);

	cv::Point2d __rhd; // robot head: meter 
	cv::Point2d _rhd; // robot head: pixel 
	cv::Point2d rhd; // robot head: pixel 
	//double theta = pose2D_XYA[2] + M_PI_2;
	double theta = pose2D_XYA[2]; // robot head toward X-axis
	__rhd.x = __rcp.x + (robot_radius + 0.02) * cos(theta);
	__rhd.y = __rcp.y + (robot_radius + 0.02) * sin(theta);
	_rhd.x = ((double)_sizeX / 2.0) + (__rhd.x - _world_center_x) / _grid_resolution;
	_rhd.y = ((double)_sizeX / 2.0) - (__rhd.y - _world_center_y) / _grid_resolution;
	rhd.x = _rhd.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0); 
	rhd.y = _rhd.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0); // pixel to show
	cv::line(_matMap, rhd, rcp, 
		cv::Scalar(colorRGB[0], colorRGB[1], colorRGB[2]), 2);

	cv::imshow("DebugViz", _matMap);
}

/**
 * Show IR Beam on Map
 * @param pRobotXYA robot pose to world frame
 * @param IrXYA IR pose to robot frame
 * @param length IR measurement value
 */
inline void DebugViz::showLaserBeam(double* pRobotXYA, double* pIrXYA, double length, int* colorRGB){
	cv::Point2d pRobot(pRobotXYA[0],pRobotXYA[1]);
	cv::Point2d __pIRa, __pIRb; // IR start, end point
	cv::Point2d _pIRa, _pIRb; // IR start, end point 
	cv::Point2d pIRa, pIRb; // IR start, end point (to show)
	double robot_radius = 0.3;// / (double)_grid_resolution;	// pixel
	__pIRa.x = pRobot.x + robot_radius * cos(pRobotXYA[2] + pIrXYA[2]);
	__pIRa.y = pRobot.y + robot_radius * sin(pRobotXYA[2] + pIrXYA[2]);
	_pIRa.x = ((double)_sizeX / 2.0) + __pIRa.x / _grid_resolution;
	_pIRa.y = ((double)_sizeY / 2.0) - __pIRa.y / _grid_resolution;
	pIRa.x = _pIRa.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	pIRa.y = _pIRa.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);

	__pIRb.x = pRobot.x + (robot_radius+length) * cos(pRobotXYA[2] + pIrXYA[2]);
	__pIRb.y = pRobot.y + (robot_radius+length) * sin(pRobotXYA[2] + pIrXYA[2]);
	_pIRb.x = ((double)_sizeX / 2.0) + __pIRb.x / _grid_resolution;
	_pIRb.y = ((double)_sizeY / 2.0) - __pIRb.y / _grid_resolution;
	pIRb.x = _pIRb.x * (_grid_size + _line_width) + ((double)_grid_size / 2.0);
	pIRb.y = _pIRb.y * (_grid_size + _line_width) + ((double)_grid_size / 2.0);

	cv::line(_matMap, pIRa, pIRb,
		cv::Scalar(colorRGB[0], colorRGB[1], colorRGB[2]), 2);

	cv::imshow("DebugViz", _matMap);
}

#endif // DEBUGVIZ_H