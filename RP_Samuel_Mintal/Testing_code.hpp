#pragma once

#ifndef WX_PRECOMP
#include <wx/wx.h>
#endif

#include <iostream>
#include <fstream>

#include <ctime>
#include <chrono>
#include <vector>

#include <thread>

#include "simulation.hpp"



std::string get_current_time_string();
std::pair<std::string, std::string> get_input_output_file_paths();



void TEST___hypothesis_1();
void TEST___hypothesis_4();