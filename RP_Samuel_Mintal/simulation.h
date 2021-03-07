#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <sstream>
#include <stdlib.h> 

struct pos {
    float x;
    float y;
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
    int   time_diff; //time in miliseconds agent is behind or forward (sign depended

    Time_diff_error(/*int agent_index,*/ int time_stamp, int time_diff) : /*agent_index(agent_index),*/ time_stamp(time_stamp), time_diff(time_diff) { }

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

    //time span of the altered plan
    int altered_max_time = 0;

    int agent_error_speed_max = 0;
    int agent_error_speed_sigma = 0;

    int agent_error_angle_max = 0;
    int agent_error_angle_sigma = 0;
    int agent_error_angle_fatal = 0;

    std::vector<plan_step> original_plan;
    std::vector<plan_step> altered_plan;

    //If the agent is late/soon or lost
    // 0 == no error, 1 == error, 5 == lost
    int errorness = 0;

public:

    /* Constructor
    * errors[0] == speed max, errors[1] == speed sigma, errors[2] == angle max, errors[3] == angle sigma, errors[4] == fatal angle
    */
    Agent(std::string name, std::string color, pos start, pos finish, const std::vector<int>& errors);

    /* moves agent to absolute time
    * parameter time is in miliseconds
    * returns true if agent successfully perfomed his action, false if not (got lost)
    * This function changes agent's current and rotation variables.
    */
    bool move_to_time(int time);

    std::string get_color() const;
    std::string get_name() const;
    pos get_current_position();
    int get_rotation();
    auto get_altered_plan_length();
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







class Simulation {
    std::vector<std::vector<char>> map;
    std::vector<Agent> agents;
    std::vector<std::vector<Time_diff_error>> time_diffs_of_agents;

    // ERROR IS CONSIDERED ONLY IF AGENT IS LATE OR SOONER THEN 1000 ms
    const int ERROR_TRESHOLD = 1000;

    //time span of the longest plan out of all agents
    int agent_plan_max_length = 0;


    class Plan_normalizer {

        std::string solver_name;
        std::vector<int> plan_step_durations;

    public:

        /* param_solver_name is the name of the solver from the .solr plan, that will be taken into account while normalizing the plan
        *  param_plan_step_durations[0] == go_duration, param_plan_step_durations[1] == turn_duration, param_plan_step_durations[2] == wait_duration
        *  if param_plan_step_durations.size() < 3 then plan_step_durations will be supplemented with pramater default_plan_step_length so it is size() == 3.
        */
        Plan_normalizer(const std::string& param_solver_name, const std::vector<int>& param_plan_step_durations, int default_plan_step_length = 2000) {

            set_solver_name(param_solver_name);
            set_plan_step_durations(param_plan_step_durations, default_plan_step_length);
        }

        /* param_solver_name is the name of the solver from the .solr plan, that will be taken into account while normalizing the plan
        */
        void set_solver_name(const std::string& param_solver_name) {
            solver_name = param_solver_name;
        }

        /* param_plan_step_durations[0] == go_duration, param_plan_step_durations[1] == turn_duration, param_plan_step_durations[2] == wait_duration
        *  if param_plan_step_durations.size() < 3 then plan_step_durations will be supplemented with pramater default_plan_step_length so it is size() == 3.
        */
        void set_plan_step_durations(const std::vector<int>& param_plan_step_durations, int default_plan_step_length = 2000) {
            plan_step_durations = param_plan_step_durations;

            while (plan_step_durations.size() < 3)
                plan_step_durations.push_back(default_plan_step_length);
        }

        /* Retuns vector of normalized plan_steps from plan_step step
        * increments refference to plan_step_id
        */
        std::vector<plan_step> get_normalized_plan_steps_from_plan_step(plan_step&& step, const std::vector<plan_step>& preceding_normalized_agent_plan_step, int& plan_step_id) {

            std::vector<plan_step> ret;

            const int go_duration = plan_step_durations[0];
            const int turn_duration = plan_step_durations[1];
            const int wait_duration = plan_step_durations[2];

            if (solver_name == "mapf_edge_split_rob" ||
                solver_name == "mapf_edge_split" ||
                solver_name == "mapf_edge_disjoint") {
                //in this case the plan doesn't use rotation field nor the turn instruction
                //So I have to add them


                if (step.action == "start") {
                    //no need to change anything except of duration
                    //here step.rotation == 0 always
                    ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }

                //so I can get the angle the agent was previosly to this step
                //would preceeding[-1] only when when I would be reading the first (start) plan_instruction, but when that is the case I wont get here so it is Ok
                plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];


                if (step.action == "end") {
                    //need to rotate to 0 rotation angle
                    if (previous_step.rotation == 90) {
                        //it will be faster to turnLeft
                        ret.emplace_back(plan_step(previous_step.position, previous_step.rotation, "turnLeft", turn_duration, plan_step_id, -90));
                        plan_step_id++;
                    }
                    else {
                        //it will be faster or equaly as fast to turnRight
                        for (size_t i = 0; ((previous_step.rotation + i) % 360) != 0; i += 90) {
                            ret.emplace_back(plan_step(previous_step.position, (previous_step.rotation + i) % 360, "turnRight", turn_duration, plan_step_id, 90));
                            plan_step_id++;
                        }
                    }


                    ret.emplace_back(plan_step(previous_step.position, 0, "end", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "waitB") {
                    //need to change the name to wait and correct the rotation field and duration
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "wait", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "goB") {
                    //change action name from goB to go and correct the rotation field and duration
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "leftGo") {
                    //need to split into turnLeft and go instruction
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnLeft", turn_duration, plan_step_id, -90));
                    plan_step_id++;
                    ret.emplace_back(plan_step(step.position, (previous_step.rotation + 270) % 360, "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "rightGo") {
                    //need to split into turnRight and go instruction
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration, plan_step_id, 90));
                    plan_step_id++;
                    ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "backwardGo") {
                    //need to split into 2x turnRight and go instruction
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration, plan_step_id, 90));
                    plan_step_id++;
                    ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "turnRight", turn_duration, plan_step_id, 90));
                    plan_step_id++;
                    ret.emplace_back(plan_step(step.position, (previous_step.rotation + 180) % 360, "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }


            }
            else if (solver_name == "mapf_vertex_split_rob" ||
                solver_name == "mapf_vertex_split") {
                //in this case the plan already has only rotation, go, start, end, wait instructions
                //Also it's already using Rotation field correctly

                if (step.action == "start") {
                    //here step.rotation == 0 always
                    ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "end") {
                    //no need to change anything except of duration
                    ret.emplace_back(plan_step(step.position, step.rotation, "end", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "waitC") {
                    //change action name from waitC to wait and duration                
                    ret.emplace_back(plan_step(step.position, step.rotation, "wait", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "goC") {
                    //change action name from goC to go and duration
                    ret.emplace_back(plan_step(step.position, step.rotation, "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "turnLeft") {
                    //no need to change anything except of duration
                    ret.emplace_back(plan_step(step.position, step.rotation, "turnLeft", turn_duration, plan_step_id, -90));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "turnRight") {
                    //no need to change anything except of duration
                    ret.emplace_back(plan_step(step.position, step.rotation, "turnRight", turn_duration, plan_step_id, 90));
                    plan_step_id++;
                    return ret;

                }
            }
            else if (solver_name == "mapf_simple") {
                //in this case the plan doesn't use rotation field nor the turn instruction
                //So I have to add them

                if (step.action == "start") {
                    //here step.rotation == 0 always
                    ret.emplace_back(plan_step(step.position, step.rotation, "start", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;
                }
                else if (step.action == "end") {
                    //need to rotate to 0 rotation angle
                    plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];

                    if (previous_step.rotation == 90) {
                        //it will be faster to turnLeft
                        ret.emplace_back(plan_step(previous_step.position, previous_step.rotation, "turnLeft", turn_duration, plan_step_id, -90));
                        plan_step_id++;
                    }
                    else {
                        //it will be faster or equaly as fast to turnRight
                        for (size_t i = 0; ((previous_step.rotation + i) % 360) != 0; i += 90) {
                            ret.emplace_back(plan_step(previous_step.position, (previous_step.rotation + i) % 360, "turnRight", turn_duration, plan_step_id, 90));
                            plan_step_id++;
                        }
                    }


                    ret.emplace_back(plan_step(previous_step.position, 0, "end", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "wait") {
                    plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];
                    ret.emplace_back(plan_step(step.position, previous_step.rotation, "wait", wait_duration, plan_step_id));
                    plan_step_id++;
                    return ret;

                }
                else if (step.action == "north" ||
                    step.action == "east" ||
                    step.action == "south" ||
                    step.action == "west") {
                    //make rotate and go

                    //so I can get the angle the agent was previosly to this step
                    //would preceeding[-1] only when when I would be reading the first (start) plan_instruction, but when that is the case I wont get here so it is Ok
                    plan_step previous_step = preceding_normalized_agent_plan_step[preceding_normalized_agent_plan_step.size() - 1];

                    int angle_delta = previous_step.rotation - cardinal_to_angle(step.action);

                    //rotate to the right angle
                    if (angle_delta == 90 || angle_delta == -270) {
                        ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnLeft", turn_duration, plan_step_id, -90));
                        plan_step_id++;
                    }
                    else if (angle_delta == -90 || angle_delta == 270) {
                        ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration, plan_step_id, 90));
                        plan_step_id++;
                    }
                    else if (std::abs(angle_delta) == 180) {
                        ret.emplace_back(plan_step(step.position, previous_step.rotation, "turnRight", turn_duration, plan_step_id, 90));
                        plan_step_id++;
                        ret.emplace_back(plan_step(step.position, (previous_step.rotation + 90) % 360, "turnRight", turn_duration, plan_step_id, 90));
                        plan_step_id++;
                    }
                    //else if (angle_delta == 0) {
                    //    //when no rotation occured, the agent will have to wait the turn_duration so the plan will stay synced
                    //    ret.emplace_back(plan_step(step.position, step.rotation, "wait", turn_duration, plan_step_id));
                    //    plan_step_id++;
                    //}

                    ret.emplace_back(plan_step(step.position, cardinal_to_angle(step.action), "go", go_duration, plan_step_id));
                    plan_step_id++;
                    return ret;
                }
            }



            //default, shouldn't happen
            return ret;

        }

        int cardinal_to_angle(const std::string& str) {
            if (str == "north") return 0;
            else if (str == "east") return 90;
            else if (str == "south") return 180;
            else if (str == "west") return 270;

            return 0;
        }

    };


    /**************************************************************************************************************************************************************************/
    /************************************************************   ALTER PLANS   *********************************************************************************************/
    
    bool is_position_out_of_map(const pos& position);

    /* Also checks if position is within the map
    * if it's not, it returns true, because the agent should be able to get there, same as with obstacle
    */
    bool is_obstacle_on_position(const pos& position);

    /* Transforms any angle to range of 0 < angle <= 360
    */
    int normalize_angle(int angle);

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
    * returns time spam of the altered plan
    */
    int set_time_diffs_of_agent_accordingly_to(int agent_index, const std::vector<plan_step>& altered, const std::vector<plan_step>& original);

    /*
    * returns -1 when lost, or one of 0,90,180,270 if not
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
    * Gives only const reference
    */
    const std::vector<Agent>& show_agents();

    /*
    * returns the time spam of the simulation
    */
    int get_agent_plan_max_length();

};