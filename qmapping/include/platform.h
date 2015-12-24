#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <string>
#include <vector>

using namespace std;

typedef enum { CARMEN_LOG_FILE, CSV } LogFormat;
typedef struct 
{
	float x;
	float y;
	float th;
} Odom;

struct Platform
{
public:
	Platform(){}
	Platform(string name) : _name(name) {}
	Platform(string name, string config, string log, LogFormat format) : _name(name) 
	{
		loadConfigFile(config);
		setLogFile(log, format);
	}
	inline bool loadConfigFile(string filename) 
	{
		// TODO: put platform-dependent params in Platform class
		return true;
	}
	inline bool setLogFile(string filename, LogFormat format) 
	{
		_file = ifstream(filename);
		if (_file.bad()) {
			_file.close();
			return false; 
		}
		if (format == CARMEN_LOG_FILE) {
			_loop = LogIterator(_file, ' ');
		}
		_format = format;
		return true;
	}
	inline bool getNextReading(vector<double>& reading, Odom& odom, double& timestamp) 
	{
		size_t num_readings;
		while (_loop != LogIterator()) {
			_loop++;
			if ((*_loop)[0] == "FLASER") {
				reading.clear();
				num_readings = (size_t)std::stoi((*_loop)[1]);
				for (size_t i = 0; i < num_readings; i++) {
					reading.push_back(stod((*_loop)[i + 2]));
				}
				odom.x = std::stod((*_loop)[2 + num_readings + 3 + 0]); // odom_x
				odom.y = std::stod((*_loop)[2 + num_readings + 3 + 1]); // odom_y
				odom.th = std::stod((*_loop)[2 + num_readings + 3 + 2]); // odom_theta
				timestamp = std::stod((*_loop)[2 + num_readings + 3 + 3 + 0]);
				return true;
			}
		}
		return false;
	}
private:
	string _name;
	string _config;
	string _log;
	LogFormat _format;
	LogIterator _loop;
	ifstream _file;
};

#endif