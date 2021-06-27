#pragma once
#include "simulation.hpp"
#include <algorithm> // for std::sort

struct detection_result {
	int agent1_index = 0;
	int agent2_index = 0;
	int at_time = 0;
	pos at_position;
};


class ICollision_Detection {	

protected:

	int get_plan_length(const std::vector<plan_step>& plan);

public:

	ICollision_Detection();

	detection_result virtual execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents ,int from_time) = 0;
};


class Line_Detection : public ICollision_Detection {	

	const double ERROR_RESRVE = 0.1;

	std::vector<int> generate_time_points_from_plans(const std::vector<std::vector<plan_step>>& plans);

	std::vector<int> sort_and_unique_time_points(const std::vector<int>& time_points);

	std::vector<std::pair<int, int>> generate_time_scopes_from_time_points(const std::vector<int>& time_points);

	std::vector<std::vector<std::pair<pos, pos>>> get_agent_positions(std::vector<std::vector<plan_step>>& plans, const std::vector<std::pair<int, int>>& time_scopes, const std::vector<Agent>& agents);	

	bool do_two_agent_lines_intersect(std::pair<pos, pos> first_ray, std::pair<pos, pos> second_ray);
	

public:

	Line_Detection();

	detection_result execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time);

};