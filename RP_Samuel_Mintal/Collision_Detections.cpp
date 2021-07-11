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
		
		float small_x1, big_x1, small_x2, big_x2;
		float small_y1, big_y1, small_y2, big_y2;

		//x of first_rect sorted
		if (first_ray.first.x <= first_ray.second.x) {
			small_x1 = first_ray.first.x;
			big_x1 = first_ray.second.x;
		}
		else {
			small_x1 =first_ray.second.x;
			big_x1 = first_ray.first.x;
		}

		//x of second ray sorted
		if (second_ray.first.x <= second_ray.second.x) {
			small_x2 = second_ray.first.x;
			big_x2 = second_ray.second.x;
		}
		else {
			small_x2 = second_ray.second.x;
			big_x2 = second_ray.first.x;
		}

		//y of first_rect sorted
		if (first_ray.first.y <= first_ray.second.y) {
			small_y1 = first_ray.first.y;
			big_y1 = first_ray.second.y;
		}
		else {
			small_y1 = first_ray.second.y;
			big_y1 = first_ray.first.y;
		}

		//y of second ray sorted
		if (second_ray.first.y <= second_ray.second.y) {
			small_y2 = second_ray.first.y;
			big_y2 = second_ray.second.y;
		}
		else {
			small_y2 = second_ray.second.y;
			big_y2 = second_ray.first.y;
		}

		//Now that I have sorted intervals it is time to compare them
		if (small_x1 <= big_x2 && small_x2 <= big_x1 &&
			small_y1 <= big_y2 && small_y2 <= big_y1) {
			return true;
		}
	
		//else they do not intersect
		return false;
	}

	detection_result Line_Detection::execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time) {

		//Holds time_scopes, from which agent lines will be generated
		std::vector<std::pair<int, int>> time_scopes = generate_time_scopes_from_time_points(generate_time_points_from_plans(plans));

		//agents_positions[i][j] is pair of poisitions of agent i in j-th time_scope
		std::vector<std::vector<std::pair<pos, pos>>> agents_positions = get_agent_positions(plans, time_scopes, agents);
	


		for (size_t ts_i = 0; ts_i < time_scopes.size(); ts_i++) {

			//get to the correct time_scope index
			if (time_scopes[ts_i].second < from_time)
				continue;

			for (size_t i = 0; i < agents_positions.size() - 1; i++) {
				for (size_t j = i + 1; j < agents_positions.size(); j++) {
					//if (do_two_agent_lines_intersect(agents_positions[i][ts_i], agents_positions[j][ts_i])) {
					//	printf("HEH");
					//}
					if (do_two_agent_lines_intersect(agents_positions[i][ts_i], agents_positions[j][ts_i])) {

						//Collision Detected!
						detection_result ret;

						ret.collision_detected = true;
						ret.agent1_index = i;
						ret.agent2_index = j;						
						ret.from_time = time_scopes[ts_i].first;
						ret.to_time = time_scopes[ts_i].second;

						return ret;
					}
				}
			}

		}


		//If there is no collision this returns
		detection_result default_res;
		return default_res;
	}













	//class Rectangle_Detection : public ICollision_Detection

		//const double Const_error_distance = 0.1;

	int Rectangle_Detection::get_max_plan_span(const std::vector<std::vector<plan_step>>& plans) {

		int ret = 0;

		for (const std::vector<plan_step>& plan : plans) {
			
			int curr = get_plan_length(plan);

			if (curr > ret)
				ret = curr;			
		}

		return ret;
	}

		std::vector<std::pair<int, int>> Rectangle_Detection::generate_time_scopes_with_distance(std::vector<std::vector<plan_step>>& plans, int distance) {
			
			std::vector<std::pair<int, int>> ret;
			int max_plan_span = get_max_plan_span(plans);
			

			std::vector<int> time_points;
			for (size_t i = 0; i < max_plan_span + distance; i += distance)
				time_points.push_back(i);

			
			for (size_t i = 0; i < time_points.size() - 1; i++)
				ret.push_back(std::make_pair(time_points[i], time_points[i + 1]));
			
			return ret;
		}

		std::vector<std::vector<std::pair<pos, pos>>> Rectangle_Detection::get_agent_positions(std::vector<std::vector<plan_step>>& plans, const std::vector<std::pair<int, int>>& time_scopes, const std::vector<Agent>& agents) {

			std::vector<std::vector<std::pair<pos, pos>>> ret;

			//for every agent
			for (size_t i = 0; i < plans.size(); i++) {

				std::vector<std::pair<pos, pos>> ith_agent_positions;

				//for every time_scope of agent
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

		bool Rectangle_Detection::do_two_agent_rectangles_intersect(std::pair<pos, pos> first_rect, std::pair<pos, pos> second_rect) {

			float small_x1, big_x1, small_x2, big_x2;
			float small_y1, big_y1, small_y2, big_y2;

			//x of first_rect sorted
			if (first_rect.first.x <= first_rect.second.x) {
				small_x1 = first_rect.first.x;
				big_x1 = first_rect.second.x;
			}
			else {
				small_x1 = first_rect.second.x;
				big_x1 = first_rect.first.x;
			}

			//x of second ray sorted
			if (second_rect.first.x <= second_rect.second.x) {
				small_x2 = second_rect.first.x;
				big_x2 = second_rect.second.x;
			}
			else {
				small_x2 = second_rect.second.x;
				big_x2 = second_rect.first.x;
			}

			//y of first_rect sorted
			if (first_rect.first.y <= first_rect.second.y) {
				small_y1 = first_rect.first.y;
				big_y1 = first_rect.second.y;
			}
			else {
				small_y1 = first_rect.second.y;
				big_y1 = first_rect.first.y;
			}

			//y of second ray sorted
			if (second_rect.first.y <= second_rect.second.y) {
				small_y2 = second_rect.first.y;
				big_y2 = second_rect.second.y;
			}
			else {
				small_y2 = second_rect.second.y;
				big_y2 = second_rect.first.y;
			}

			//Now that I have sorted intervals it is time to compare them
			if (small_x1 <= big_x2 && small_x2 <= big_x1 &&
				small_y1 <= big_y2 && small_y2 <= big_y1) {
				return true;
			}

			//else they do not intersect
			return false;
		}


	//public:

		Rectangle_Detection::Rectangle_Detection(int distance) 
			: _distance(distance) {

		}

		detection_result Rectangle_Detection::execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time) {

			//Holds time_scopes, from which agent lines will be generated
			std::vector<std::pair<int, int>> time_scopes = generate_time_scopes_with_distance(plans, _distance);

			//agents_positions[i][j] is pair of poisitions of agent i in j-th time_scope
			std::vector<std::vector<std::pair<pos, pos>>> agents_positions = get_agent_positions(plans, time_scopes, agents);



			for (size_t ts_i = 0; ts_i < time_scopes.size(); ts_i++) {

				//get to the correct time_scope index
				if (time_scopes[ts_i].second < from_time)
					continue;

				for (size_t i = 0; i < agents_positions.size() - 1; i++) {
					for (size_t j = i + 1; j < agents_positions.size(); j++) {
//						if (do_two_agent_rectangles_intersect(agents_positions[i][ts_i], agents_positions[j][ts_i])) {
//							printf("HEH");
//						}
						if (do_two_agent_rectangles_intersect(agents_positions[i][ts_i], agents_positions[j][ts_i])) {

							//Collision Detected!
							detection_result ret;

							ret.collision_detected = true;
							ret.agent1_index = i;
							ret.agent2_index = j;
							ret.from_time = time_scopes[ts_i].first;
							ret.to_time = time_scopes[ts_i].second;

							return ret;
						}
					}
				}

			}


			//If there is no collision this returns
			detection_result default_res;
			return default_res;
		}











		//class Sampling_Detection : public ICollision_Detection

		//float const_error_distance = 0.1;
		//int const_sampling_amplitude_ms = 10;
		//int const_max_sampling = 1;
		//int const_min_sampling = 10;
		//float const_max_map_distance = 10.0;

		float Sampling_Detection::get_distance_of(const pos& p1, const pos& p2) {
		
			return static_cast<float>(	
				std::sqrtf(
					
					std::pow( 	std::abs(p1.x) - std::abs(p2.x)	, 2	)
					+
					std::pow(	std::abs(p1.y) - std::abs(p2.y) , 2	)
				)
			);
		}

		pos Sampling_Detection::get_agents_pos_according_to(int time, const Agent& agent, const std::vector<plan_step>& plan) {
			return agent.get_agents_move_state_in(plan, get_plan_length(plan), time).current;
		}

		int Sampling_Detection::get_max_plan_span(const std::vector<std::vector<plan_step>>& plans) {

			int ret = 0;

			for (const std::vector<plan_step>& plan : plans) {

				int curr = get_plan_length(plan);

				if (curr > ret)
					ret = curr;
			}

			return ret;
		}

		//returns distance to the closest position for positions[index]
		float Sampling_Detection::get_distance_to_the_closest_pos(const std::vector<pos>& positions, int index) {
		
			float ret = std::numeric_limits<float>::max();;

			for (size_t i = 0; i < positions.size(); i++) {

				//Distance from myself is 0 and I do not want to consider it
				if (i == index)
					continue;

				float dist = get_distance_of(positions[i], positions[index]);

				if (dist < ret)
					ret = dist;
			}

			return ret;
		}

		//public:
		/*
		* @Param error_distance - when agents gets closer than this the collision is considered to occur
		* @Param sampling_amplitude_ms - by how much ms the system moves at the time
		* @Param max_sampling/min_sampling - after how many ticks the coordinates are updated
		*/
		Sampling_Detection::Sampling_Detection(const std::vector<std::vector<char>>& map, float error_distance, int sampling_amplitude_ms, int max_sampling , int min_sampling)
			: const_error_distance(error_distance), 
			const_sampling_amplitude_ms(sampling_amplitude_ms), 
			const_max_sampling(max_sampling),
			const_min_sampling(min_sampling) {

			const_max_map_distance = 0.0;

			//If I have valid map
			if (map.size() > 0)
				if (map[0].size() > 0) {

					//upper left corner of map
					pos first;
					first.x = 0.0;
					first.y = 0.0;

					//lower right corner of map
					pos second;
					second.x = static_cast<float>(map[map.size() - 1].size()) - static_cast<float>(1.0);
					second.y = static_cast<float>(map.size()) - static_cast<float>(1.0);

					const_max_map_distance = get_distance_of(first, second);
				}					
		}

		size_t Sampling_Detection::get_sampling_rate(float closest_distance) {
			
			float quotient = closest_distance / const_max_map_distance;

			//const_max_sampling < const_min_sampling as numbers
			size_t sampling_rate = const_max_sampling + static_cast<size_t>(static_cast<float>(const_min_sampling - const_max_sampling) * quotient);

			return sampling_rate;
		}

		detection_result Sampling_Detection::execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time) {

			std::vector<pos> agents_positions(agents.size());
			//set there initial positions of agents
			for (size_t i = 0; i < agents.size(); i++) 
				agents_positions[i] = get_agents_pos_according_to(from_time, agents[i], plans[i]);
			

			std::vector<float> distances_to_the_closest_agents(agents.size());
			//set there initial closest distances of agents
			for (size_t i = 0; i < agents.size(); i++) {
				distances_to_the_closest_agents[i] = get_distance_to_the_closest_pos(agents_positions, i);
			}

			std::vector<size_t> sampling_rates(agents.size());
			//set there initial sampling rates of agents
			for (size_t i = 0; i < agents.size(); i++) {
				sampling_rates[i] = get_sampling_rate(distances_to_the_closest_agents[i]);
			}

			int max_plan_make_span = get_max_plan_span(plans);

			//Collision detection goes here
			for (size_t curr_time = from_time, current_tick = 1; curr_time < max_plan_make_span + const_sampling_amplitude_ms; curr_time += const_sampling_amplitude_ms, current_tick++) {



				if (false) {
					curr_time = 3990;
					curr_time = 5990;

					curr_time = 7700;

					curr_time = 9990;
					curr_time = 11990;
				}




				//update positions according to curr_time, if theyr sampling rate allows it
				for (size_t i = 0; i < sampling_rates.size(); i++) {

					size_t rest = current_tick % sampling_rates[i]; //so the compiler doesnt optimalize it away........ 

					if (!rest) // == 0
						agents_positions[i] = get_agents_pos_according_to(curr_time, agents[i], plans[i]);
				}

				//Detect collisions
				for (size_t i = 0; i < agents_positions.size() - 1; i++) {
					for (size_t j = i + 1; j < agents_positions.size(); j++) {
					
						if (get_distance_of(agents_positions[i], agents_positions[j]) <= const_error_distance) {

							//Collision Detected!
							detection_result ret;

							ret.collision_detected = true;
							ret.agent1_index = i;
							ret.agent2_index = j;
							ret.from_time = curr_time;
							ret.to_time = curr_time;
							ret.at_position = agents_positions[i];

							return ret;

						}
					}
				}


				//Update sampling_rates if the sampling_rates allow it
				for (size_t i = 0; i < sampling_rates.size(); i++) {

					size_t rest = current_tick % sampling_rates[i]; //so the compiler doesnt optimalize it away........ 
				
					if (!rest) 
						sampling_rates[i] = get_sampling_rate(get_distance_to_the_closest_pos(agents_positions, i));												
				}
			}


			//If there is no collision this returns
			detection_result default_res;
			return default_res;
		}










		//class Static_Sampling_Detection : public ICollision_Detection {

			//float const_error_distance = 0.2;
			//int const_sampling_amplitude_ms = 10;

		pos Static_Sampling_Detection::get_agents_pos_according_to(int time, const Agent& agent, const std::vector<plan_step>& plan) {
			return agent.get_agents_move_state_in(plan, get_plan_length(plan), time).current;
		}

		float Static_Sampling_Detection::get_distance_of(const pos& p1, const pos& p2) {
			return static_cast<float>(
				std::sqrtf(

					std::pow(std::abs(p1.x) - std::abs(p2.x), 2)
					+
					std::pow(std::abs(p1.y) - std::abs(p2.y), 2)
				)
				);
		}

		int Static_Sampling_Detection::get_max_plan_span(const std::vector<std::vector<plan_step>>& plans) {

			int ret = 0;

			for (const std::vector<plan_step>& plan : plans) {

				int curr = get_plan_length(plan);

				if (curr > ret)
					ret = curr;
			}

			return ret;
		}

		//public:

		Static_Sampling_Detection::Static_Sampling_Detection(float error_distance, int sampling_amplitude_ms) 
			: const_error_distance(error_distance),
			const_sampling_amplitude_ms(sampling_amplitude_ms){
			
		}

		detection_result Static_Sampling_Detection::execute_detection(std::vector<std::vector<plan_step>>& plans, std::vector<Agent>& agents, int from_time) {

			std::vector<pos> agents_positions(agents.size());
			int max_plan_make_span = get_max_plan_span(plans);


			//Collision detection goes here
			for (size_t curr_time = from_time; curr_time < max_plan_make_span + const_sampling_amplitude_ms; curr_time += const_sampling_amplitude_ms) {

				//update positions according to curr_time
				for (size_t i = 0; i < agents.size(); i++)
					agents_positions[i] = get_agents_pos_according_to(curr_time, agents[i], plans[i]);

				//Detect collisions
				for (size_t i = 0; i < agents_positions.size() - 1; i++) {
					for (size_t j = i + 1; j < agents_positions.size(); j++) {

						if (get_distance_of(agents_positions[i], agents_positions[j]) <= const_error_distance) {
							//Collision Detected!

							detection_result ret;

							ret.collision_detected = true;
							ret.agent1_index = i;
							ret.agent2_index = j;
							ret.from_time = curr_time;
							ret.to_time = curr_time;
							ret.at_position = agents_positions[i];

							return ret;

						}
					}
				}
			}

			//If there is no collision this returns
			detection_result default_res;
			return default_res;
		}		