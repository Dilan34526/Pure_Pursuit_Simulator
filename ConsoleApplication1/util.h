#pragma once

#include <fstream>
#include <sstream>
#include <iostream>
#include <cstdarg>
#include <string>
#include <iomanip>
#include <vector>

extern float round_up(float value, int decimal_places);

extern void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset);
