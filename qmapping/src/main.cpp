#include <gmapping/utils/autoptr.h>
#include <gmapping/utils/stat.h>
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/commandline.h>
#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <parameter.h>
#include <logutils.h>
#include "../src/openslam_gmapping/log/carmenconfiguration.h"
#include "../src/openslam_gmapping/log/sensorstream.h"
#include "../src/openslam_gmapping/scanmatcher/scanmatcherprocessor.h"
#include <debugviz.h>
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <ctime>

#define _USE_MATH_DEFINES
#include <cmath>

// The angles in the laser, going from -x to x (adjustment is made to get the laser between
// symmetrical bounds as that's what gmapping expects)
std::vector<double> laser_angles_;
GMapping::RangeSensor* gsp_laser_;
unsigned int gsp_laser_beam_count_;
GMapping::OdometrySensor* gsp_odom_;
GMapping::GridSlamProcessor* gsp_;
std::string odom_frame_;

bool got_first_scan_;
bool got_map_;

// Parameters used by GMapping
double maxRange_;
double minRange_;
double maxUrange_, maxrange_;
double minimum_score_;
double sigma_;
int kernelSize_;
double lstep_, astep_;
int iterations_;
double lsigma_;
double ogain_;
int lskip_;
double srr_, srt_, str_, stt_;
double linearUpdate_, angularUpdate_;
double temporalUpdate_;
double resampleThreshold_;
int particles_;
double xmin_, ymin_, xmax_, ymax_;
double delta_;
double occ_thresh_;
double llsamplerange_, llsamplestep_, lasamplerange_, lasamplestep_;
unsigned long int seed_;

double scan_angle_increment_;

DebugViz dbgviz_;

int global_start_time;

bool isX80 = false;

void initParamDefault()
{
	gsp_ = new GMapping::GridSlamProcessor();
	gsp_laser_ = NULL;
	gsp_odom_ = NULL;

	got_first_scan_ = false;
	got_map_ = false;
	// Parameters used by GMapping itself
	maxUrange_ = 0.0;  maxRange_ = 0.0; minRange_ = 0.0; // preliminary default, will be set in initMapper()
	minimum_score_ = 0;
	sigma_ = 0.05;
	kernelSize_ = 1;
	lstep_ = 0.05;	astep_ = 0.05;	iterations_ = 5; lsigma_ = 0.075;
	ogain_ = 3.0; lskip_ = 0;
	srr_ = 0.1;	srt_ = 0.2;	str_ = 0.1;	stt_ = 0.2;
	linearUpdate_ = 1.0; angularUpdate_ = 0.5;
	temporalUpdate_ = -1.0;	resampleThreshold_ = 0.5;
	particles_ = 30;
	xmin_ = -100.0;	ymin_ = -100.0;	xmax_ = 100.0;	ymax_ = 100.0;
	delta_ = 0.05;
	occ_thresh_ = 0.25;
	llsamplerange_ = 0.01;	llsamplestep_ = 0.01;	lasamplerange_ = 0.005;	lasamplestep_ = 0.005;

	seed_ = std::time(NULL);
}

bool loadParam(const std::string &filename)
{
	std::map<std::string, std::string> config;
	std::ifstream fin(filename.c_str());
	std::string line;

	if (!fin) {
		std::cout << "can't open file: " << filename.c_str() << std::endl;
		return false;
	}

	while (getline(fin, line)) {
		if (line[0] == '#') {
			continue;
		}
		// Trim comment #
		size_t posPound = line.find_first_of('#');
		if (posPound != std::string::npos) {
			line = line.substr(0, posPound);
		}
		// Remove blank
		line.erase(
			remove_if(line.begin(), line.end(), ::isspace),
			line.end()
			);
		// Skip if empty
		if (line.empty()) {
			continue;
		}
		std::stringstream ss(line);
		std::string key;
		std::string value;
		std::getline(ss, key, '=');
		std::getline(ss, value, '=');
		//		transform(key.begin(), key.end(), key.begin(), ::toupper);
		config[key] = value;
		// debug
		//std::cout << "[" << key << "]" << "=" << value << std::endl;
	}
	fin.close();

	// update parameters
	try { maxRange_ = stod(config.at("maxRange_")); }
	catch (const std::out_of_range) {};
	try { minRange_ = stod(config.at("minRange_")); }
	catch (const std::out_of_range) {};
	try { maxUrange_ = stod(config.at("maxUrange_")); }
	catch (const std::out_of_range) {};
	try { maxrange_ = stod(config.at("maxrange_")); }
	catch (const std::out_of_range) {};
	try { minimum_score_ = stod(config.at("minimum_score_")); }
	catch (const std::out_of_range) {};
	try { sigma_ = stod(config.at("sigma_")); }
	catch (const std::out_of_range) {};
	try { kernelSize_ = (size_t)stoi(config.at("kernelSize_")); }
	catch (const std::out_of_range) {};
	try { lstep_ = stod(config.at("lstep_")); }
	catch (const std::out_of_range) {};
	try { astep_ = stod(config.at("astep_")); }
	catch (const std::out_of_range) {};
	try { iterations_ = (size_t)stoi(config.at("iterations_")); }
	catch (const std::out_of_range) {};
	try { lsigma_ = stod(config.at("lsigma_")); }
	catch (const std::out_of_range) {};
	try { ogain_ = stod(config.at("ogain_")); }
	catch (const std::out_of_range) {};
	try { lskip_ = stoi(config.at("lskip_")); }
	catch (const std::out_of_range) {};
	try { srr_ = stod(config.at("srr_")); }
	catch (const std::out_of_range) {};
	try { srt_ = stod(config.at("srt_")); }
	catch (const std::out_of_range) {};
	try { str_ = stod(config.at("str_")); }
	catch (const std::out_of_range) {};
	try { stt_ = stod(config.at("stt_")); }
	catch (const std::out_of_range) {};
	try { linearUpdate_ = stod(config.at("linearUpdate_")); }
	catch (const std::out_of_range) {};
	try { angularUpdate_ = stod(config.at("angularUpdate_")); }
	catch (const std::out_of_range) {};
	try { temporalUpdate_ = stod(config.at("temporalUpdate_")); }
	catch (const std::out_of_range) {};
	try { resampleThreshold_ = stod(config.at("resampleThreshold_")); }
	catch (const std::out_of_range) {};
	try { particles_ = (size_t)stoi(config.at("particles_")); }
	catch (const std::out_of_range) {};
	try { xmin_ = stod(config.at("xmin_")); }
	catch (const std::out_of_range) {};
	try { ymin_ = stod(config.at("ymin_")); }
	catch (const std::out_of_range) {};
	try { xmax_ = stod(config.at("xmax_")); }
	catch (const std::out_of_range) {};
	try { ymax_ = stod(config.at("ymax_")); }
	catch (const std::out_of_range) {};
	try { delta_ = stod(config.at("delta_")); }
	catch (const std::out_of_range) {};
	try { occ_thresh_ = stod(config.at("occ_thresh_")); }
	catch (const std::out_of_range) {};
	try { llsamplerange_ = stod(config.at("llsamplerange_")); }
	catch (const std::out_of_range) {};
	try { llsamplestep_ = stod(config.at("llsamplestep_")); }
	catch (const std::out_of_range) {};
	try { lasamplerange_ = stod(config.at("lasamplerange_")); }
	catch (const std::out_of_range) {};
	try { lasamplestep_ = stod(config.at("lasamplestep_")); }
	catch (const std::out_of_range) {};
	try { seed_ = (unsigned int)stoi(config.at("seed_")); }
	catch (const std::out_of_range) {};
	try { scan_angle_increment_ = (size_t)stoi(config.at("scan_angle_increment_")); }
	catch (const std::out_of_range) {};

	return true;
}

bool initMapper(double* readings, size_t num_readings, GMapping::OrientedPoint initialPose)
{
	gsp_laser_beam_count_ = num_readings;
	// Note: in IrSlam, angles might not be symmeteric or keeping same angle distance from neighbor IRs.
	if (laser_angles_.empty()) { // not set yet
		// Compute the angles of the laser from -x to x, basically symmetric and in increasing order
		laser_angles_.resize(num_readings);
		// Make sure angles are started so that they are centered
		//	double theta = -std::fabs(scan.angle_min - scan.angle_max) / 2;
		double theta = -std::fabs(0 - M_PI) / 2;
		for (unsigned int i = 0; i < num_readings; ++i) {
			laser_angles_[i] = theta;
			theta += std::fabs(scan_angle_increment_);
		}
	}
	else { // already set
		//*/ debug only
		std::cout << "laser_angles_ already set: ";
		for (unsigned int i = 0; i < num_readings; ++i) {
			std::cout << laser_angles_[i] << " ";
		}
		std::cout << std::endl;
		//*/
	}
	// The laser must be called "FLASER".
	// We pass in the absolute value of the computed angle increment, on the
	// assumption that GMapping requires a positive angle increment.  If the
	// actual increment is negative, we'll swap the order of ranges before
	// feeding each scan to GMapping.
	GMapping::OrientedPoint gmap_pose(0, 0, 0);
	//GMapping::OrientedPoint gmap_pose(0, 0, M_PI_2);
	gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan_angle_increment_), gmap_pose, 0.0, maxRange_);
	GMapping::SensorMap smap;
	smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
	gsp_->setSensorMap(smap);
	gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

	gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, ogain_, lskip_);
	gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
	gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
	gsp_->setUpdatePeriod(temporalUpdate_);
	gsp_->setgenerateMap(false);
	gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_, delta_, initialPose);
	gsp_->setllsamplerange(llsamplerange_);
	gsp_->setllsamplestep(llsamplestep_);

	/// @todo Check these calls; in the gmapping gui, they use
	/// llsamplestep and llsamplerange intead of lasamplestep and
	/// lasamplerange.  It was probably a typo, but who knows.
	gsp_->setlasamplerange(lasamplerange_);
	gsp_->setlasamplestep(lasamplestep_);
	gsp_->setminimumScore(minimum_score_);

	// Call the sampling function once to set the seed.
	GMapping::sampleGaussian(1, seed_);

	return true;
}

bool addScan(double* readings, size_t num_readings, GMapping::OrientedPoint& gmap_pose, double timestamp)
{
	// GMapping wants an array of doubles...
	double* ranges_double = new double[num_readings];
	// If the angle increment is negative, we have to invert the order of the readings.
	for (unsigned int i = 0; i < num_readings; i++) {
		// Must filter out short readings, because the mapper won't
		if (readings[i] < minRange_) // minimal valid range reading
			ranges_double[i] = (double)maxrange_;
		else
			ranges_double[i] = (double)readings[i];
	}

	GMapping::RangeReading reading(num_readings, ranges_double, gsp_laser_, timestamp);
	// add offset for IRs (X80)
	if (isX80) {
		const double OFFSET_1 = 0.18;
		const double OFFSET_2 = 0.18;
		const double OFFSET_3 = 0.18;
		const double OFFSET_4 = 0.18;
		const double OFFSET_5 = 0.10;
		const double OFFSET_6 = 0.18;
		const double OFFSET_7 = 0.10;
		reading[0] += OFFSET_1;
		reading[1] += OFFSET_2;
		reading[2] += OFFSET_3;
		reading[3] += OFFSET_4;
		reading[4] += OFFSET_5;
		reading[5] += OFFSET_6;
		reading[6] += OFFSET_7;
	}

	// ...but it deep copies them in RangeReading constructor, so we don't
	// need to keep our array around.
	delete[] ranges_double;

	reading.setPose(gmap_pose);

	std::cout << "scanpose (" << timestamp << "): " << gmap_pose.x << " " << gmap_pose.y << " " << gmap_pose.theta << std::endl;
	std::cout << "processing scan" << std::endl;

	return gsp_->processScan(reading);
}

struct MapMetaData
{
	float resolution;
	size_t width, height;
	GMapping::OrientedPoint origin_pose;
};
struct OccupancyGrid
{
	MapMetaData info;
	std::vector<int> data;
};

OccupancyGrid map_;
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

void updateMap(double* readings, size_t num_readings)
{
	std::cout << "Update map" << std::endl;
	std::mutex map_mutex_;
	GMapping::ScanMatcher matcher;

	matcher.setLaserParameters(num_readings, &(laser_angles_[0]), gsp_laser_->getPose());

	matcher.setlaserMaxRange(maxRange_);
	matcher.setusableRange(maxUrange_);
	matcher.setgenerateMap(true);

	GMapping::GridSlamProcessor::Particle best = gsp_->getParticles()[gsp_->getBestParticleIndex()];

	if (!got_map_) {
		map_.info.resolution = (float)delta_;
		map_.info.origin_pose.x = 0.0;
		map_.info.origin_pose.y = 0.0;
		map_.info.origin_pose.theta = 0.0;
	}

	GMapping::Point center;
	center.x = (xmin_ + xmax_) / 2.0;
	center.y = (ymin_ + ymax_) / 2.0;

	GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_, delta_);

	std::cout << "Trajectory tree:" << std::endl;
	for (GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent) {
		//ROS_DEBUG("  %.3f %.3f %.3f", n->pose.x, n->pose.y, n->pose.theta);
		std::cout << n->pose.x << " " << n->pose.y << " " << n->pose.theta << std::endl;
		if (!n->reading) {
			std::cout << "Reading is NULL" << std::endl;
			continue;
		}
		matcher.invalidateActiveArea();
		matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
		matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
		/*
		std::cout << "world(0,0) to map: " << smap.world2map(0,0).x << "," << smap.world2map(0,0).y << std::endl;
		std::cout << "map center: " << smap.getMapSizeX() / 2 << "," << smap.getMapSizeY() / 2 << std::endl;
		GMapping::Point world_center = smap.map2world(smap.getMapSizeX() / 2, smap.getMapSizeY() / 2);
		std::cout << "map center to world: " << world_center.x << "," << world_center.y << std::endl;
		std::cout << "robot point to world: " << n->pose.x << "," << n->pose.y << std::endl;
		dbgviz_.showGridMap(smap);
		cv::waitKey(0);
		*/
	}

	dbgviz_.showGridMap(smap);

	int robot_color[3] = { 0, 0, 255 };
	dbgviz_.showRobotPose(smap, best.node->pose, robot_color);

	int beam_color[3] = { 10, 200, 28 };
	for (size_t i = 0; i < num_readings; i++) {
		GMapping::OrientedPoint beam_pose = best.node->pose;
		beam_pose.theta += laser_angles_[i];
		//dbgviz_.showIrBeam(smap, beam_pose, readings[i], beam_color);
		dbgviz_.showIrBeam(smap, beam_pose, (*best.node->reading)[i], beam_color);
	}
	cv::waitKey(10);
	// the map may have expanded, so resize ros message as well
	if (map_.info.width != (unsigned int)smap.getMapSizeX() || map_.info.height != (unsigned int)smap.getMapSizeY()) {

		// NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
		//       so we must obtain the bounding box in a different way
		GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
		GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
		xmin_ = wmin.x; ymin_ = wmin.y;
		xmax_ = wmax.x; ymax_ = wmax.y;

		std::cout << "map size is now "
			<< smap.getMapSizeX() << "x" << smap.getMapSizeY() << " pixels (" << xmin_ << "," << ymin_ << ")-(" << xmax_ << "," << ymax_ << ")"
			<< std::endl;

		map_.info.width = smap.getMapSizeX();
		map_.info.height = smap.getMapSizeY();
		map_.info.origin_pose.x = xmin_;
		map_.info.origin_pose.y = ymin_;
		map_.data.resize(map_.info.width * map_.info.height);

		std::cout << "map origin: (" << map_.info.origin_pose.x << "," << map_.info.origin_pose.y << ")" << std::endl;
	}

	std::cout << "updateMap: sizeX, sizeY = " << smap.getMapSizeX() << " " << smap.getMapSizeY() << std::endl;
	for (int x = 0; x < smap.getMapSizeX(); x++) {
		for (int y = 0; y < smap.getMapSizeY(); y++) {
			/// @todo Sort out the unknown vs. free vs. obstacle thresholding
			GMapping::IntPoint p(x, y);
			double occ = smap.cell(p);
			assert(occ <= 1.0);
			if (occ < 0) {
				map_.data[MAP_IDX(map_.info.width, x, y)] = -1;
			}
			else if (occ > occ_thresh_) {
				//map_.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
				map_.data[MAP_IDX(map_.info.width, x, y)] = 100;
			}
			else {
				map_.data[MAP_IDX(map_.info.width, x, y)] = 0;
			}
		}
	}
	got_map_ = true;
}

// qiao@2015.08.10: copy from csharp project
unsigned int AD2Dis(short IRValue)
{
	double temp = 0;
	double IRAD2Distance = 0;

	temp = 21.6 / ((double)IRValue * 3 / 4028 - 0.17);

	// IR range 10-80cm
	if ((temp > 80) || (temp < 0))
		IRAD2Distance = 81;
	else if ((temp < 10) && (temp > 0))
		IRAD2Distance = 9;
	else
		IRAD2Distance = temp;
	return IRAD2Distance;
}

int main()
{
	global_start_time = 0;
	GMapping::Point global_start_point(0, 0);
	initParamDefault();
	size_t laser_count_ = 0;
	double enc1_prev, enc2_prev;
	double timestamp = 0.0;

	//*/ qiao: set log ang param file here
	//std::ifstream file("csail-oldcarmen.log");
	//std::ifstream file("intel.clf");
	std::ifstream file("data\\toby_20151217_071843.clf");
	//loadParam("gmapping.cfg");
	loadParam("config\\toby.cfg");
	bool isCSV = false;
	bool isCarmen = true;
	isX80 = false;

	/*
	std::ifstream file("data17_sqare300x300_4_vel100.csv");
	//std::ifstream file("data16_sqare300x300_3_vel100.csv");
	//std::ifstream file("data3_rec200x150.csv");
	//std::ifstream file("data11_loop_width50_1.csv");
	//std::ifstream file("intel.clf");
	loadParam("gmapping_X80.cfg");
	bool isCSV = true;
	bool isCarmen = false;
	isX80 = true;
	enc1_prev = enc2_prev = 0.0;
	//*/


	LogIterator loop;
	if (isCSV) {
		loop = LogIterator(file, ',');
	}
	else {
		loop = LogIterator(file, ' ');
	}

	double* readings = 0;
	size_t num_readings;
	GMapping::OrientedPoint odom_pose;
	for (; loop != LogIterator(); ++loop) {
		// in old-format carmen-log, FLASER should followed by ODOM
		if (isCarmen && (*loop)[0] == "FLASER") {
			// # FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta
			num_readings = (size_t)std::stoi((*loop)[1]);
			// fill laser reading
			readings = (double *)malloc(sizeof(double) * num_readings);
			memset(readings, 0, num_readings);
			for (size_t i = 0; i < num_readings; i++) {
				readings[i] = std::stod((*loop)[i + 2]);
				// std::cout << readings[i] << " ";
			}
			// std::cout << std::endl;
			odom_pose.x = std::stod((*loop)[2 + num_readings + 3 + 0]); // odom_x
			odom_pose.y = std::stod((*loop)[2 + num_readings + 3 + 1]); // odom_y
			odom_pose.theta = std::stod((*loop)[2 + num_readings + 3 + 2]); // odom_theta

			timestamp = std::stod((*loop)[2 + num_readings + 3 + 3 + 0]) - (double)global_start_time;
			if (global_start_time == 0) { global_start_time = (int)timestamp; }
		}
		else if (isX80) {
			// X80 log format: encoder1 encoder2 IR1 ... IR7
			num_readings = 7;
			readings = (double *)malloc(sizeof(double) * num_readings);
			memset(readings, 0, num_readings);
			readings[0] = (double)AD2Dis(std::stod((*loop)[2])) / 100;
			readings[1] = (double)AD2Dis(std::stod((*loop)[3])) / 100;
			readings[2] = (double)AD2Dis(std::stod((*loop)[4])) / 100;
			readings[3] = (double)AD2Dis(std::stod((*loop)[5])) / 100;
			readings[4] = (double)AD2Dis(std::stod((*loop)[6])) / 100;
			readings[5] = (double)AD2Dis(std::stod((*loop)[7])) / 100;
			readings[6] = (double)AD2Dis(std::stod((*loop)[8])) / 100;

			readings[3] = 0.00001;// too noisy, ignore it
			// debug
			std::cout << "readings = ";
			for (size_t i = 0; i < num_readings; i++) {
				std::cout << readings[i] << " ";
			}
			std::cout << std::endl;
			// according to IR position, IR right, right-front, right-head, left-head, left-front, left
			// is IR 5,4,3,2,1,7
			const double ANGLE_2 = 0.4729383367949;
			const double ANGLE_3 = -ANGLE_2;
			const double ANGLE_1 = 0.71347261;
			const double ANGLE_4 = -ANGLE_1;
			const double ANGLE_5 = -M_PI_2;	// 0.0;	// rad, about 180-51.44 deg
			const double ANGLE_7 = M_PI_2;
			const double ANGLE_6 = M_PI;

			laser_angles_.resize(num_readings);
			laser_angles_[0] = ANGLE_1;
			laser_angles_[1] = ANGLE_2;
			laser_angles_[2] = ANGLE_3;
			laser_angles_[3] = ANGLE_4;
			laser_angles_[4] = ANGLE_5;
			laser_angles_[5] = ANGLE_6;
			laser_angles_[6] = ANGLE_7;
			//const double RES = 0.05; // angle resolution

			// encoder
			double enc1, enc2;
			enc1 = std::stod((*loop)[0]);
			enc2 = std::stod((*loop)[1]);
			if (!got_first_scan_) { // no previous encoder data
				enc1_prev = enc1;
				enc2_prev = enc2;
				odom_pose = GMapping::OrientedPoint(0, 0, 0);
			}
			else { // got first scan
				// encoder to odometry
				// TODO: modify ref to standard formula
				double delta_Encoder1 = enc1 - enc1_prev;
				double delta_Encoder2 = enc2 - enc2_prev;
				if (fabs(delta_Encoder1) < 5 && fabs(delta_Encoder2) < 5) {
					continue;
				}

				// check if encoder just over the maximum (default 32767)
				const int cFULL_COUNT = 32767;
				const double THRESHOLD = (double)cFULL_COUNT * 0.8;
				if (delta_Encoder1 > 0 && delta_Encoder1 > THRESHOLD) { // decreasing and just cross 0->cFULL_COUNT
					delta_Encoder1 = delta_Encoder1 - (double)cFULL_COUNT;
				}
				if (delta_Encoder1 < 0 && delta_Encoder1 < -THRESHOLD) { // increasing and just cross cFULL_COUNT->0
					delta_Encoder1 = delta_Encoder1 + (double)cFULL_COUNT;
				}
				if (delta_Encoder2 > 0 && delta_Encoder2 > THRESHOLD) { // decreasing and just cross 0->cFULL_COUNT
					delta_Encoder2 = delta_Encoder2 - (double)cFULL_COUNT;
				}
				if (delta_Encoder2 < 0 && delta_Encoder2 < -THRESHOLD) { // increasing and just cross cFULL_COUNT->0
					delta_Encoder2 = delta_Encoder2 + (double)cFULL_COUNT;
				}

				// for rotation, just calculate delta angle and apply
				const double FULL_COUNT_1 = 2400;
				const double FULL_COUNT_2 = 2400;
				if (delta_Encoder1 * delta_Encoder2 >= 0) {
					double dAngleD_by_Enc1 = 360.0 * delta_Encoder1 / FULL_COUNT_1;
					double dAngleD_by_Enc2 = 360.0 * delta_Encoder2 / FULL_COUNT_2;
					double dAngle_by_Enc1 = dAngleD_by_Enc1 * M_PI / 180.0;
					double dAngle_by_Enc2 = dAngleD_by_Enc2 * M_PI / 180.0;
					odom_pose.theta += (dAngle_by_Enc1 + dAngle_by_Enc2);
					if (odom_pose.theta > 2 * M_PI) {
						odom_pose.theta -= 2 * M_PI;
					}
					if (odom_pose.theta < -2 * M_PI) {
						odom_pose.theta += 2 * M_PI;
					}
				}
				else {
					// for transition, need to find distance and delta angle
					const double M_PER_COUNT = 1.5 / 3360;
					// forward: delta_Encoder1 < 0, delta_Encoder2 > 0
					double dAngle = 2 * M_PI * (delta_Encoder1 + delta_Encoder2) / ((FULL_COUNT_1 + FULL_COUNT_2) / 2.0);
					double deltaEncoder = (fabs(delta_Encoder1) < fabs(delta_Encoder2)) ? delta_Encoder1 : delta_Encoder2;
					double direction = (delta_Encoder2 > 0) ? 1.0 : -1.0;
					double dDistance = direction * fabs(deltaEncoder) * M_PER_COUNT;
					//odom_pose.theta += (dAngle / 2.0); //m_Pose.rotate(dAngle);
					odom_pose.x += dDistance * cos(odom_pose.theta);
					odom_pose.y += dDistance * sin(odom_pose.theta);
					//odom_pose.theta += (dAngle / 2.0); //m_Pose.rotate(dAngle);
					std::cout << "delta odom_pose = " << dDistance * cos(odom_pose.theta) << " " << dDistance * sin(odom_pose.theta) << " " << odom_pose.theta << std::endl;
				}
				// to global odom pose
				//odom_pose.x += global_start_point.x;
				//odom_pose.y += global_start_point.y;
				enc1_prev = enc1;
				enc2_prev = enc2;
			} // got first scan
		}
		else {
			continue;
		} // (*loop)[0] == "XXX"

		// to avoid too large offset
		if (GMapping::euclidianDist(GMapping::Point(0, 0), global_start_point) < 0.000001) {
			global_start_point.x = odom_pose.x;
			global_start_point.y = odom_pose.y;
		}
		else {
			//odom_pose.x = odom_pose.x - global_start_point.x;
			//odom_pose.y = odom_pose.y - global_start_point.y;
		}
		// debug
		//std::cout << "x y theta = " << odom_pose.x << " " << odom_pose.y << " " << odom_pose.theta << " reading[91] = " << readings[91] << std::endl;
		//system("pause");
		//continue;

		// to simulate IR, reduce beams number of laser measurement
		size_t div = 1;// 36;
		size_t ir_num_readings = num_readings / div;
		double* ir_readings = (double *)malloc(sizeof(double) * ir_num_readings);
		memset(ir_readings, 0, ir_num_readings);
		scan_angle_increment_ = M_PI / (double)ir_num_readings;
		for (size_t i = 0; i < ir_num_readings; i++) {
			ir_readings[i] = readings[i * div];
		}

		// We can't initialize the mapper until we've got the first scan
		if (!got_first_scan_) {
			if (!initMapper(ir_readings, ir_num_readings, odom_pose))
				continue;
			got_first_scan_ = true;
		}
		if (isX80) { timestamp += 0.01; }
		if (addScan(ir_readings, ir_num_readings, odom_pose, timestamp)) {
			std::cout << "scan processed" << std::endl;

			GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;
			std::cout << "new best pose: " << mpose.x << " " << mpose.y << " " << mpose.theta << std::endl;
			std::cout << "odom pose: " << odom_pose.x << " " << odom_pose.y << " " << odom_pose.theta << std::endl;
			std::cout << "correction: " << mpose.x - odom_pose.x << " " << mpose.y - odom_pose.y << " " << mpose.theta - odom_pose.theta << std::endl;
			//system("pause");
			if (!got_map_ || true)	{ // always update
				updateMap(ir_readings, ir_num_readings);
				std::cout << "Updated the map" << std::endl;
			}
		}
		else { std::cout << "cannot process scan" << std::endl; }
		free(readings);
	} // for loop++
	// same logic as ROS-GMapping, refer to SlamGMapping::laserCallback

	system("pause");

	return 0;
}
