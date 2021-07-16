#pragma once

struct pos {
	float x = 0.0;
	float y = 0.0;
};

struct detection_result {
	bool collision_detected = false;
	int agent1_index = 0;
	int agent2_index = 0;
	int from_time = 0;
	int to_time = 0;
	pos at_position; //unused
};