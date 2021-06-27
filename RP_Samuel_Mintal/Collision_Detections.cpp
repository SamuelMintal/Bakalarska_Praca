#include "Collision_Detections.hpp"

//class ICollision_Detection::



	int ICollision_Detection::get_plan_length(const std::vector<plan_step>& plan) {
	
		int ret = 0;

		for (const plan_step& ps : plan)
			ret += ps.duration;

		return ret;
	}

	ICollision_Detection::ICollision_Detection() {}
	


//class Line_Detection : public ICollision_Detection
	
	std::vector<int> Line_Detection::generate_time_points_from_plans(const std::vector<std::vector<plan_step>>& plans) {

		std::vector<int> ret;

		//every plan starts at time 0
		ret.push_back(0);

		for (const std::vector<plan_step>& plan : plans) {
			int current_time = 0;

			for (const plan_step& ps : plan) {

				//for each plan_step add to time_points it's ending time.
				//The begining time is push_backed at the begining represented by ret.push_back(0);
				current_time += ps.duration;
				ret.push_back(current_time);
			}
		}

		return ret;
	}

	std::vector<int> Line_Detection::sort_and_unique_time_points(const std::vector<int>& time_points) {

		std::vector<int> ret = time_points;

		sort(ret.begin(), ret.end());
		ret.erase(unique(ret.begin(), ret.end()), ret.end());

		return ret;
	}

	std::vector<std::pair<int, int>> Line_Detection::generate_time_scopes_from_time_points(const std::vector<int>& time_points) {

		auto tmp_time_points = sort_and_unique_time_points(time_points);
		std::vector<std::pair<int, int>> ret;

		if(tmp_time_points.size() > 1)
			for (size_t i = 0; i < tmp_time_points.size() - 1; i++)
				ret.push_back(std::make_pair(tmp_time_points[i], tmp_time_points[i + 1]));
		//else return empty vector

		return ret;
	}


	std::vector<std::vector<std::pair<pos, pos>>> Line_Detection::get_agent_positions(std::vector<std::vector<plan_step>>& plans, const std::vector<std::pair<int, int>>& time_scopes, const std::vector<Agent>& agents) {

		std::vector<std::vector<std::pair<pos, pos>>> ret;

		for (size_t i = 0; i < plans.size(); i++) {

			std::vector<std::pair<pos, pos>> ith_agent_positions;

			for (size_t j = 0; j < time_scopes.size(); j++) {

				Agent_move_state first = agents[i].get_agents_move_state_in(plans[i], get_plan_length(plans[i]), time_scopes[j].first);
				Agent_move_state second = agents[i].get_agents_move_state_in(plans[i], get_plan_length(plans[i]), time_scopes[j].second);
				ith_agent_positions.push_back(std::make_pair(first.current, second.current));

			}

			ret.push_back(std::move(ith_agent_positions));
			ith_agent_positions.clear();

		}

		return ret;
	}


	Line_Detection::Line_Detection() {}

	bool Line_Detection::do_two_agent_lines_intersect(std::pair<pos, pos> first_ray, std::pair<pos, pos> second_ray) {

		//if (!(first_ray.first.x == first_ray.second.x || first_ray.first.y == first_ray.second.y || second_ray.first.x == second_ray.second.x || second_ray.first.y == second_ray.second.y)) {
		//	std::cout << "D";
		//}

		//TODO check tie 2 priamky a mam to
		

		return false;
	}

	detection_result Line_Detection::execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time) {

		//Holds time_scopes, from which agent lines will be generated
		std::vector<std::pair<int, int>> time_scopes = generate_time_scopes_from_time_points(generate_time_points_from_plans(plans));

		//agents_positions[i][j] is pair of poisitions of agent i in j-th time_scope
		std::vector<std::vector<std::pair<pos, pos>>> agents_positions = get_agent_positions(plans, time_scopes, agents);
	
		for (size_t ts_i = 0; ts_i < time_scopes.size(); ts_i++) {

			for (size_t i = 0; i < agents_positions.size(); i++) {
				for (size_t j = i; j < agents_positions.size(); j++) {
					if (do_two_agent_lines_intersect(agents_positions[i][ts_i], agents_positions[j][ts_i])) {

						//Collision Detected!
						detection_result ret;

						ret.agent1_index = i;
						ret.agent2_index = j;						
						ret.at_time = time_scopes[ts_i].first;

						return ret;
					}
				}
			}

		}


		//Should not occur!
		detection_result default_res;
		return default_res;
	}
