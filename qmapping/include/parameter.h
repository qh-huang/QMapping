#ifndef PARAMETER_H
#define PARAMETER_H

#include <map>
#include <string>
#include <iostream>
#include <fstream>
#include <istream>
#include <sstream>
#include <algorithm>
#include <stdexcept>

struct ParamMgr
{
	// Robot
	double INIT_X;
	double INIT_Y;
	double INIT_A; // angle
	double RATE_POSE_UPDATE;
	double ENCODER_FULLCOUNT;

	// MAP
	double MAP_SIZE;
	double MAP_RESOLUTION;

	// OPENCV UI
	double RATE_REDRAW;

	std::map<std::string, std::string> config;

	inline ParamMgr();

	inline void loadConfig(const std::string &filename);
	inline void saveConfig(const std::string &filename);
};

inline ParamMgr::ParamMgr()
{
	// give all parameter default value
	INIT_X = 0.0;
	INIT_Y = 0.0;
	INIT_A = 0.0;
	RATE_POSE_UPDATE = 0.2;	// s
	ENCODER_FULLCOUNT = 3600;
	MAP_SIZE = 5;	// m
	MAP_RESOLUTION = 0.05;	// m
	RATE_REDRAW = 0.02; // s
}

inline void ParamMgr::loadConfig(const std::string &filename)
{
	std::ifstream fin(filename.c_str());
	std::string line;

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
		transform(key.begin(), key.end(), key.begin(), ::toupper);
		config[key] = value;
		// debug
		std::cout << "[" << key << "]" << "=" << value << std::endl;
	}
	fin.close();

	// update parameters
	try { INIT_X = stod(config.at("INIT_X")); } catch (const std::out_of_range) {};
	try { INIT_X = stod(config.at("INIT_Y")); }	catch (const std::out_of_range) {};
	try { INIT_X = stod(config.at("INIT_A")); }	catch (const std::out_of_range) {};
	try { RATE_POSE_UPDATE = stod(config.at("RATE_POSE_UPDATE")); } catch (const std::out_of_range) {};
	try { ENCODER_FULLCOUNT = stod(config.at("ENCODER_FULLCOUNT")); } catch (const std::out_of_range) {};
	try { MAP_SIZE = stod(config.at("MAP_SIZE")); } catch (const std::out_of_range) {};
	try { MAP_RESOLUTION = stod(config.at("MAP_RESOLUTION")); }	catch (const std::out_of_range) {};

}

inline void ParamMgr::saveConfig(const std::string &filename)
{
	std::ofstream fout(filename.c_str());
	fout << "# Robot" << std::endl;
	fout << "INIT_X" << "\t=\t" << INIT_X << "\t# init pose x" << std::endl;
	fout << "INIT_Y" << "\t=\t" << INIT_Y << "\t# init pose y" << std::endl;
	fout << "INIT_A" << "\t=\t" << INIT_A << "\t# init pose a" << std::endl;
	// TODO: complete the list
	fout.close();
}

#endif // PARAMETER_H