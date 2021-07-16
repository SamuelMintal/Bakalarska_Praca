#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>
#include <stdlib.h> 

#include "Structs.hpp"


/* Stuff that agent changes on istelf when it moves
*/
struct Agent_move_state {
    pos current;
    int rotation;
    bool succesfully_moved;

    Agent_move_state() {
        pos curr;
        current = curr;
        rotation = 0;
        succesfully_moved = true;
    }
};

class plan_step {
public:
    //where it begins
    pos position;
    //at what rotation it begins
    int rotation;
    //how long the step lasts
    int duration;    
    //what the step does
    std::string action;

    //if the step is rotation than this field tells how much to rotate
    int angle_to_rotate;

    //id used for calculating time_diffs correctly and may be later used for better visualization of altered vs original plan
    int id;

    plan_step(pos position, int rotation, std::string action, int duration, int id, int p_angle_to_rotate = 0) :
        position(position), rotation(rotation), action(action), duration(duration), id(id) {
        angle_to_rotate = p_angle_to_rotate;
    }
};

class Time_diff_error {
public:
    
    int  time_stamp; //Time when agent has time difference against original plan of size time_stamp
    int   time_diff; //time in miliseconds agent is behind or forward - sign depended

    Time_diff_error( int time_stamp, int time_diff): time_stamp(time_stamp), time_diff(time_diff) { }

    bool operator < (const Time_diff_error& second) const
    {
        return (time_stamp < second.time_stamp); //For sorting
    }
};

class Agent {

    //I use name as an ID of an Agent
    std::string name; 
    std::string color;
    pos start;
    pos finish;
    pos current;
    int rotation = 0;

    //time span of both plans
    int altered_max_time = 0;
    int original_max_time = 0;

    int agent_error_speed_max = 0;
    int agent_error_speed_sigma = 0;

    int agent_error_angle_max = 0;
    int agent_error_angle_sigma = 0;
    int agent_error_angle_fatal = 0;

    std::vector<plan_step> original_plan;
    std::vector<plan_step> altered_plan;

    //If the agent is late/soon or succesfully_moved
    // 0 == no error, 1 == error, 5 == succesfully_moved
    int errorness = 0;

public:

    /* Constructor
    * errors[0] == speed max, errors[1] == speed sigma, errors[2] == angle max, errors[3] == angle sigma, errors[4] == fatal angle
    */
    Agent(std::string name, std::string color, pos start, pos finish, const std::vector<int>& errors);

    /* Returns move_state that agent would be at time 'time' if he was following plan 'plan' with it's time span being 'plan_max_time'
    */
    Agent_move_state get_agents_move_state_in(const std::vector<plan_step>& plan, int plan_max_time, int time) const;

    /* moves agent to absolute time
    * parameter time is in miliseconds
    * returns true if agent successfully perfomed his action, false if not (got succesfully_moved)
    * This function changes agent's current and rotation variables.
    */
    bool move_to_time(int time);

    /* Returns move_state that represents where agent should be if he was following his plan perfectly (his original plann)
    */
    Agent_move_state where_should_I_be(int time) const;

    std::string get_color() const;
    std::string get_name() const;
    pos get_current_position();
    int get_rotation();
    auto get_altered_plan_length() const;
    auto get_original_plan_length() const;
    std::vector<plan_step> get_altered_plan() const;
    std::vector<int> get_err_vector();
    std::vector<plan_step> get_original_plan() const;
    int get_error_state();

    void set_error_state(int err);
    void set_errors(const std::vector<int>& errors);
    void set_original_plan(std::vector<plan_step>&& vct);
    void set_altered_plan(std::vector<plan_step>&& vct);
    void set_altered_plan(std::vector<plan_step>&& vct, int total_altered_plan);
    void set_altered_plan(const std::vector<plan_step>& vct, int total_altered_plan);
};






#include "Collision_Detections.hpp"





class Simulation {
    std::vector<std::vector<char>> map;
    std::vector<Agent> agents;
    std::vector<std::vector<Time_diff_error>> time_diffs_of_agents;    

    // ERROR IS CONSIDERED ONLY IF AGENT IS LATE OR SOONER THEN 1000 ms
    /*const*/ int ERROR_TRESHOLD = 1000;

    //time span of the longest plan out of all agents
    int agent_plan_max_length = 0;

    /**************************************************************************************************************************************************************************/
    /************************************************************   COLLISIONS   *********************************************************************************************/
public:
    /* Returns plan, which agent is expected to perform
    *  - altered plan until now, then original plan
    * Used to calculate potential collisions
    */
    std::vector<plan_step> get_expected_plan_from_time(int time, const Agent& agt) const;

    /* When collision is detected, this will set the simulation to do expected plans etc,
    * UNABLE TO ROLL BACK, make backup if you wish to do so
    */
    void set_to_expected_plans_state(int time);

    std::string chosen_detection_method = "";
    detection_result last_detection_result;
private:
    /**************************************************************************************************************************************************************************/
    /************************************************************   ALTER PLANS   *********************************************************************************************/
    
    bool is_position_out_of_map(const pos& position) const;

    /* Also checks if position is within the map
    * if it's not, it returns true, because the agent should be able to get there, same as with obstacle
    */
    bool is_obstacle_on_position(const pos& position) const;

    /* Transforms any angle to range of 0 < angle < 360
    */
    int normalize_angle(int angle) const;

    /* u == 0 always
    */
    float normal_distribution(float sigma, float x);

    float get_scaler(int sigma, int max);

    /* Populates specified map using normal distribution with parameters as specified
    * returns probablities*scaler sum
    */
    uint64_t init_prob_map(std::vector<std::pair<int, float>>& map, float precision, int max, int sigma);

    
    /* Rolls random pair with consideration of probabilities given by map
    * (That means if for example option a.) has 90% of happening it should choose it 9 out of 10 times)
    * returns index to said pair
    */
    size_t roll_pair_and_get_its_index(const std::vector<std::pair<int, float>>& prob_map, uint64_t prob_sum, int i, int j);

    /* Sets agent's timediffs vector accordingly to the 2 specified plans
    * returns time span of the altered plan
    */
    int set_time_diffs_of_agent_accordingly_to(int agent_index, const std::vector<plan_step>& altered, const std::vector<plan_step>& original);

    /*
    * returns -1 when succesfully_moved, or one of 0,90,180,270 if not
    */
    int get_closest_90angle_or_lost(int angle, int fatal_angle);

    /* goal_angle is normalized, actual_angle doesn't have to be
    * returns signed number representing how many more degrees the agent will have to rotate to correct itself
    */
    int get_angle_to_be_corrected(int actual_angle, int goal_angle);

    /* weird indexes imply that this function can be used to alter one or all of the agents plans
    */
    void alter_plans_of_agents(int from, int to);

    /* updates agent_plan_max_length.
    * Goes through all agents and compares theyrs plan_length to agent_plan_max_length
    */
    void update_agent_plan_max_length();

    /* Returns index to specified agent's timediff field relevant for time
    *  if returns -1, the agent havent even finished his 1st plan_step and therefore it cannot be late/soon
    */
    int get_index_of_timediff_for_agent_at_time(int agents_index, int time);


    /**************************************************************************************************************************************************************************/
    /************************************************************   LOAD PLANS   **********************************************************************************************/


    /* 
    * Puts the concrete Solver Option Predicat Name into the line
    */
    void load_plans_get_SolverOptionPredicatName(std::ifstream& myfile, std::string& line);

    /*
    * Puts the firsts agent definition into the line
    */
    void load_plans_get_FirstAgentDefinition(std::ifstream& myfile, std::string& line);

    void load_plans_load_all_agents(std::ifstream& myfile, std::string& line, std::vector<Agent>& agents_vector, const std::vector<int>& errors_vectors, const std::string& delimiter);

    void load_plans_load_map(std::ifstream& myfile, std::string& line, std::vector<std::vector<char>>& map);

    void load_plans_load_num_to_vertex_map(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex);


    /*
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    */
    void load_plan_load_and_normalize_agents_plans(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex,
        std::vector<Agent>& agents_vector, const std::string& solver_name, const std::vector<int>& plan_step_durations);

    /* Retuns vector of normalized plan_steps from plan_step step
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    * increments refference to plan_step_id
    */
    std::vector<plan_step> get_normalized_plan_steps_from(plan_step&& step, const std::string& solver_name, const std::vector<plan_step>& preceding_normalized_agent_plan_step, const std::vector<int>& plan_step_durations, int& plan_step_id);

    int cardinal_to_angle(const std::string& str);


public:

    Simulation();


    /*
    * plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
    */
    void load_plans(std::string path, const std::vector<int>& plan_step_durations);

    /* moves all agents of simulation to specified time
    * Returns true if At least one agent moved. When it returns False, there is no point in moving to bigger time.
    * if time < 0 it moves to time == 0 and returns false
    */
    bool move_to_time(int time);


    std::vector<std::string> get_agents_names();

    /* Sets Agent's errors and also alters his plan.
    *  Also updates agent_plan_max_length so it stays correct
    *  parameters: 0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma,4 = fatal angle
    */
    void set_errors_to_agent(std::string& agent_name, const std::vector<int>& parameters);

    /*
    * returns how late/soon the agent is at specified time
    */
    int get_time_diffs_of_agent_at_time(int agent_index, int time);

    /*
    * Gives only const reference
    */
    const std::vector<std::vector<char>>& show_map();

    /*
    * Gives only reference
    */
     std::vector<Agent>& show_agents();

    /*
    * returns the time span of the simulation
    */
    int get_agent_plan_max_length();

};