#pragma once
#include "simulation.hpp"
#include <algorithm> // for std::sort
#include <limits> // for std::max

#include "Structs.hpp"

/* Interface which every collision detection method implements
*/
class ICollision_Detection {	

protected:

	int get_plan_length(const std::vector<plan_step>& plan);

public:

	ICollision_Detection();

	detection_result virtual execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents ,int from_time) = 0;
};


class Line_Detection : public ICollision_Detection {	

	std::vector<int> generate_time_points_from_plans(const std::vector<std::vector<plan_step>>& plans);

	std::vector<int> sort_and_unique_time_points(const std::vector<int>& time_points);

	std::vector<std::pair<int, int>> generate_time_scopes_from_time_points(const std::vector<int>& time_points);

	std::vector<std::vector<std::pair<pos, pos>>> get_agent_positions(std::vector<std::vector<plan_step>>& plans, const std::vector<std::pair<int, int>>& time_scopes, const std::vector<Agent>& agents);	

	bool do_two_agent_lines_intersect(std::pair<pos, pos> first_ray, std::pair<pos, pos> second_ray);
	

public:

	Line_Detection();

	detection_result execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time);

};


class Rectangle_Detection : public ICollision_Detection {

	int _distance = 0;

	int get_max_plan_span(const std::vector<std::vector<plan_step>>& plans);

	std::vector<std::pair<int, int>> generate_time_scopes_with_distance(std::vector<std::vector<plan_step>>& plans, int distance);

	std::vector<std::vector<std::pair<pos, pos>>> get_agent_positions(std::vector<std::vector<plan_step>>& plans, const std::vector<std::pair<int, int>>& time_scopes, const std::vector<Agent>& agents);

	bool do_two_agent_rectangles_intersect(std::pair<pos, pos> first_ray, std::pair<pos, pos> second_ray);


public:
	/* time distance for making rectangles
	*/
	Rectangle_Detection(int distance);

	detection_result execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time);

};


class Sampling_Detection : public ICollision_Detection {

	float const_error_distance = 0.2;
	int const_sampling_amplitude_ms = 10;
	int const_max_sampling = 1;
	int const_min_sampling = 10;
	float const_max_map_distance = 10.0;

	pos get_agents_pos_according_to(int time, const Agent& agent, const std::vector<plan_step>& plan);

	float get_distance_of(const pos& p1, const pos& p2);

	int get_max_plan_span(const std::vector<std::vector<plan_step>>& plans);

	float get_distance_to_the_closest_pos(const std::vector<pos>& positions, int index);	

	size_t get_sampling_rate(float closest_distance);

public:

	Sampling_Detection(const std::vector<std::vector<char>>& map, float error_distance = 0.2, int sampling_amplitude_ms = 10, int max_sampling = 1, int min_sampling = 10);

	detection_result execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time);

};

class Static_Sampling_Detection : public ICollision_Detection {

	float const_error_distance = 0.2;
	int const_sampling_amplitude_ms = 10;

	pos get_agents_pos_according_to(int time, const Agent& agent, const std::vector<plan_step>& plan);

	float get_distance_of(const pos& p1, const pos& p2);

	int get_max_plan_span(const std::vector<std::vector<plan_step>>& plans);

public:

	Static_Sampling_Detection(float error_distance = 0.2, int sampling_amplitude_ms = 10);

	detection_result execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time);

};