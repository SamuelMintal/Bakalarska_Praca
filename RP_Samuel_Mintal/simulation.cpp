#include "simulation.h"


/**************************************************************************************************************************************/
/*************************************************************************************************************************************/
/*********************************************************** AGENT  *******************************************************************/
/**************************************************************************************************************************************/
/**************************************************************************************************************************************/


 /* Constructor
 * errors[0] == speed max, errors[1] == speed sigma, errors[2] == angle max, errors[3] == angle sigma, errors[4] == fatal angle
 */
Agent::Agent(std::string name, std::string color, pos start, pos finish, const std::vector<int>& errors) :
    name(name), color(color), start(start), finish(finish) {
    agent_error_speed_max = errors[0];
    agent_error_speed_sigma = errors[1];

    agent_error_angle_max = errors[2];
    agent_error_angle_sigma = errors[3];
    agent_error_angle_fatal = errors[4];

    current = start;
}

/* moves agent to absolute time
* parameter time is in miliseconds
* returns true if agent successfully perfomed his action, false if not (got lost)
* This function changes agent's current and rotation variables.
*/
bool Agent::move_to_time(int time) {  

    //If I was supposed to move more than I can, I will stand at the end of my plan
    if (time > altered_max_time) {
        current = altered_plan[altered_plan.size() - 1].position;

        if (altered_plan[altered_plan.size() - 1].action == "endLost")
            return false;
        else
            return true;
    }

    int tmp_time = 0;
    for (size_t i = 0; i < altered_plan.size(); i++) {

        //If I am supposed to start from following plan_step and not this
        if (tmp_time + altered_plan[i].duration < time) { 
            tmp_time += altered_plan[i].duration;
        }
        //else movement started from i-th plan_step
        else {

            if (altered_plan[i].action == "start" || altered_plan[i].action == "end" || altered_plan[i].action == "wait") {
                //Go to the starting position of this plan_step
                current = altered_plan[i].position; 
                rotation = altered_plan[i].rotation;

                return true;
            }
            //Else I move to the location between the 2 plan_steps
            else if (altered_plan[i].action == "go") { 
                current.x = altered_plan[i].position.x + (static_cast<float>((time - tmp_time)) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i + 1].position.x - altered_plan[i].position.x); //There is always i+1 th action because if i-th action was end, i didnt get into this else branch
                current.y = altered_plan[i].position.y + (static_cast<float>((time - tmp_time)) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i + 1].position.y - altered_plan[i].position.y); //

                //to avoid "direction arrows" being badly orineted due to short turn time or fast simulation step time
                rotation = altered_plan[i].rotation;

                return true;
            }
            else if (altered_plan[i].action == "turnLeft" || altered_plan[i].action == "turnRight") {
                int signed_rotation = altered_plan[i].rotation + (static_cast<int>((static_cast<float>(time - tmp_time) / static_cast<float>(altered_plan[i].duration)) * (altered_plan[i].angle_to_rotate)));
                while (signed_rotation < 0)
                    signed_rotation += 360;

                rotation = signed_rotation % 360;

                return true;
            }
            else if (altered_plan[i].action == "endLost") {
                //Go to the starting position of this plan_step
                current = altered_plan[i].position; 
                rotation = altered_plan[i].rotation;

                return false;
            }
        }
    }

    //default -> should not occur
    return true;
}

std::string Agent::get_color() {
    return color;
}

std::string Agent::get_name() const {
    return name;
}

pos Agent::get_current_position() {
    return current;
}

int Agent::get_rotation() {
    return rotation;
}

auto Agent::get_altered_plan_length() {
    return altered_max_time;
}

std::vector<plan_step>  Agent::get_altered_plan() const {
    return altered_plan;
}

std::vector<int> Agent::get_err_vector() {
    std::vector<int> ret = { agent_error_speed_max, agent_error_speed_sigma, agent_error_angle_max, agent_error_angle_sigma, agent_error_angle_fatal };
    return ret;
}

std::vector<plan_step> Agent::get_original_plan() const {    
    return original_plan;
}

int Agent::get_error_state() {
    return errorness;
}


void Agent::set_error_state(int err) {
    errorness = err;
}

void Agent::set_errors(const std::vector<int>& errors) {
    agent_error_speed_max = errors[0];
    agent_error_speed_sigma = errors[1];

    agent_error_angle_max = errors[2];
    agent_error_angle_sigma = errors[3];
    agent_error_angle_fatal = errors[4];
}

void Agent::set_original_plan(std::vector<plan_step>&& vct) {
    original_plan = std::move(vct);
}

void Agent::set_altered_plan(std::vector<plan_step>&& vct) {
    altered_plan = std::move(vct);
    altered_max_time = 0;
    for (size_t i = 0; i < altered_plan.size(); i++)
        altered_max_time += altered_plan[i].duration;
}

void Agent::set_altered_plan(std::vector<plan_step>&& vct, int total_altered_plan) {
    altered_plan = std::move(vct);
    altered_max_time = total_altered_plan;
}

void Agent::set_altered_plan(const std::vector<plan_step>& vct, int total_altered_plan) {
    altered_plan = vct;
    altered_max_time = total_altered_plan;
}



/**************************************************************************************************************************************/
/**************************************************************************************************************************************/
/********************************************************* SIMULATION  ****************************************************************/
/**************************************************************************************************************************************/
/**************************************************************************************************************************************/

Simulation::Simulation() {}

/**************************************************************************************************************************************************************************/
/************************************************************   ALTER PLANS   *********************************************************************************************/

bool Simulation::is_position_out_of_map(const pos& position) {

    if (map.size() == 0)
        return true;


    if ((position.y < map.size()) && (position.y >= 0) && (position.x < map[0].size()) && (position.x >= 0))
        return false;
    else
        return true;
}

/* Also checks if position is within the map
* if it's not, it returns true, because the agent should be able to get there, same as with obstacle
*/
bool Simulation::is_obstacle_on_position(const pos& position) {

    if (is_position_out_of_map(position))
        return false;
    else
        return (map[static_cast<int>(position.y)][static_cast<int>(position.x)] == '@');
}


/* Transforms any angle to range of 0 < angle <= 360
*/
int Simulation::normalize_angle(int angle) {
    while (angle < 0)
        angle += 360;
    angle = angle % 360;

    return angle;
}

/* u == 0 always
*/
float Simulation::normal_distribution(float sigma, float x) { 

    return (1 / (sigma * sqrtf(2 * static_cast<float>(3.14159)))) * static_cast<float>(pow(2.71828, -0.5 * pow(x / sigma, 2)));
}

float Simulation::get_scaler(int sigma, int max) {

    float ret_scaler = 1;
    float prev_scaler = 1;

    float f_sigma = static_cast<float>(sigma);
    float f_max = static_cast<float>(max);

    //while the possibility of max error occuring is (due to casting to int) zero, increase the scaler so it wouldn't be
    while (!static_cast<int>(normal_distribution(f_sigma, f_max) * ret_scaler)) {
        prev_scaler = ret_scaler;
        ret_scaler *= 10;


        //defense against overflow
        //this will occur when the sigma is too low for the specified max
        //this implies that the max won't occur
        if (ret_scaler < prev_scaler) {
            ret_scaler = prev_scaler / 1000000;
            break;
        }


    }

    //*100 so the probabilities will be more accurate
    return ret_scaler * 100; 
}

/* Populates specified map using normal distribution with parameters as specified
* returns probablities*scaler sum
*/
uint64_t Simulation::init_prob_map(std::vector<std::pair<int, float>>& map, float precision, int max, int sigma) {

    std::pair<int, float> prob_pair;
    uint64_t ret_sum = 0;

    auto speed_scaler = get_scaler(sigma, max);
    for (float x = 0; x <= max; x += precision) {
        //first is probablity * scaler casted into int
        //second is value of x
        prob_pair = std::make_pair(static_cast<int>(normal_distribution(static_cast<float>(sigma), x) * speed_scaler), x);

        //if the overflow in get_scaler has occured I won't be able to register some of the values near the ned of the spectrum
        if (prob_pair.first == 0) {
            break;
        }
        else {
            ret_sum += prob_pair.first;
            map.push_back(prob_pair);
            if (x != 0) {
                ret_sum += prob_pair.first;
                prob_pair.second = prob_pair.second * (-1); //normal distribution is symetrical when u == 0
                map.push_back(prob_pair);
            }
        }
    }

    return ret_sum;
}


/* Rolls random pair with consideration of probabilities given by map
* (That means if for example option a.) has 90% of happening it should choose it 9 out of 10 times)
* returns index to said pair
*/
size_t Simulation::roll_pair_and_get_its_index(const std::vector<std::pair<int, float>>& prob_map, uint64_t prob_sum, int i, int j) {

    //rolling random prob_value
    srand(j + i * (11071998 * j * j));
    uint64_t r1 = rand();
    srand(static_cast<unsigned int>(j * (23072001 * r1 * j)));
    uint64_t r2 = rand();
    uint64_t chosen_prob_value = (r1 * r2) % prob_sum;

    uint64_t curr_prob_sum = 0;

    //Find the rolled pair
    for (size_t i = 0; i < prob_map.size(); i++) {
        curr_prob_sum += prob_map[i].first;

        //If I rolled this pair
        if (curr_prob_sum >= chosen_prob_value) {
            return i;
        }
    }

    return 0;
}

/* Sets agent's timediffs vector accordingly to the 2 specified plans
* returns time spam of the altered plan
*/
int Simulation::set_time_diffs_of_agent_accordingly_to(int agent_index, const std::vector<plan_step>& altered, const std::vector<plan_step>& original) {
    //Note-ing time differences of original vs altered plan of agent, so I can detect errors later

    //Reset it for pushabcking new items
    time_diffs_of_agents[agent_index].clear();

    int time_counter_original = 0;
    int time_counter_altered = 0;

    int i_original = 0;
    int i_altered = 0;

    bool i_original_changed = true;

    //Errors can be noted ONLY when the agent finishes the action. Motivation is that he could make up for his mistake until the end of the plan_step action
    for (size_t i_altered = 0; i_altered < altered.size(); i_altered++) {

        //I wont fall out of range because max(original.id) == max(altered.id)
        if (original[i_original].id < altered[i_altered].id) {

            time_diffs_of_agents[agent_index].emplace_back(Time_diff_error(time_counter_altered, time_counter_altered - time_counter_original));
            //move onto next
            i_original++;
            i_original_changed = true;
        }

        if (i_original_changed)
            time_counter_original += original[i_original].duration;
        time_counter_altered += altered[i_altered].duration;

        i_original_changed = false;
    }
    time_diffs_of_agents[agent_index].emplace_back(Time_diff_error(time_counter_altered, time_counter_altered - time_counter_original));

    return time_counter_altered;
}

/*
* returns -1 when lost, or one of 0,90,180,270 if not
*/
int Simulation::get_closest_90angle_or_lost(int angle, int fatal_angle) {

    //correct the angle so it spans 0 <= angle < 360
    angle = normalize_angle(angle);

    int delta_to_0 = std::min(360 - angle, angle);
    int delta_to_90 = std::abs(angle - 90);
    int delta_to_180 = std::abs(angle - 180);
    int delta_to_270 = std::abs(angle - 270);

    int min_delta;
    int angle_of_minimal_delta;

    min_delta = delta_to_0;
    angle_of_minimal_delta = 0;
    if (delta_to_90 < min_delta) {
        min_delta = delta_to_90;
        angle_of_minimal_delta = 90;
    }
    if (delta_to_180 < min_delta) {
        min_delta = delta_to_180;
        angle_of_minimal_delta = 180;
    }
    if (delta_to_270 < min_delta) {
        min_delta = delta_to_270;
        angle_of_minimal_delta = 270;
    }

    if (min_delta < fatal_angle) {
        return angle_of_minimal_delta;
    }
    else {
        return -1;
    }
}

/* goal_angle is normalized, actual_angle doesn't have to be
* returns signed number representing how many more degrees the agent will have to rotate to correct itself
*/
int Simulation::get_angle_to_be_corrected(int actual_angle, int goal_angle) {

    actual_angle = normalize_angle(actual_angle);
    int angle_to_be_corrected = 0;

    if (goal_angle == 0) {
        //Trouble caused by that top angle is both 0 and 360 but I take it as 0

        if (actual_angle < 180)
            angle_to_be_corrected = -actual_angle;
        else
            angle_to_be_corrected = 360 - actual_angle;
    }
    else /*closest_angle == 90/180/270*/ {

        angle_to_be_corrected = goal_angle - actual_angle;
    }

    return angle_to_be_corrected;
}

/* weird indexes imply that this function can be used to alter one or all of the agents plans
*/
void Simulation::alter_plans_of_agents(int from, int to) {

    const float speed_precision = 100;   // n in  miliseconds ,where agent can be faster/slower by x*n miliseconds and x*n <= max_error_speed
    const float angle_precision = 1;     // n in  degrees     ,where agent can be rotated badly by x*n degrees     and x*n <= max_error_agnle          

    //for every specified agent            
    for (int i = from; i < to; i++) {

        //Despite it's name, this original plan may get deformed while making the altered one 
        //Namely when the rotations change the plan
        std::vector<plan_step> original = agents[i].get_original_plan();

        //the plan I am making and eventually will set as the agent's altered one
        std::vector<plan_step> altered;

        auto err_vec = agents[i].get_err_vector();
        int tmp_error_speed_max = err_vec[0];
        int tmp_error_speed_sigma = err_vec[1];
        int tmp_error_angle_max = err_vec[2];
        int tmp_error_angle_sigma = err_vec[3];
        int tmp_error_angle_fatal = err_vec[4];

        std::vector<std::pair<int, float>> speed_prob_map; // maps probability*scale [int] to relative change to plan_step action duration or action rotation
        std::vector<std::pair<int, float>> angle_prob_map; // scale is chosen by init_prob_map and I don't care about its concrete value

        uint64_t speed_prob_sum = init_prob_map(speed_prob_map, speed_precision, tmp_error_speed_max, tmp_error_speed_sigma);
        uint64_t angle_prob_sum = init_prob_map(angle_prob_map, angle_precision, tmp_error_angle_max, tmp_error_angle_sigma);


        //for every plan_step
        for (size_t j = 0; j < original.size(); j++) {

            if ((original[j].action == "turnLeft" || original[j].action == "turnRight")) {

                if (tmp_error_angle_max == 0) {
                    //Do not change anything

                    //angle_to_rotate to be used by this plan_step instruction
                    int total_angle_to_rotate = original[j].angle_to_rotate;

                    plan_step curr = original[j];
                    altered.emplace_back(plan_step(curr.position, curr.rotation, curr.action, curr.duration, curr.id, curr.angle_to_rotate));

                    //As I could have changed my direction, i need to set j+1 rotation to be correct for the future instruction
                    original[j + 1].rotation = normalize_angle(total_angle_to_rotate + original[j].rotation);
                    original[j + 1].position = original[j].position;
                }
                else {
                    //for every step just roll the dices
                    size_t rolled_pair_index = roll_pair_and_get_its_index(angle_prob_map, angle_prob_sum, i, j);
                    int chosen_relative_angle = static_cast<int>(angle_prob_map[rolled_pair_index].second);

                    // future_uncorrected_rotation stands for the yet uncorrected starting rotation for the next plan_step
                    int future_uncorrected_starting_rotation = original[j].rotation + original[j].angle_to_rotate + chosen_relative_angle;

                    //stands for the closest 0/90/180/270 axis to the future_uncorrected_starting_rotation, or if -1 stands for that the agent has lost due to the current rotation
                    int closest_angle_to_altered = get_closest_90angle_or_lost(future_uncorrected_starting_rotation, tmp_error_angle_fatal);
                    if (closest_angle_to_altered == -1) {
                        //if the agent got lost
                        plan_step curr = original[j];

                        int total_angle_to_rotate = original[j].angle_to_rotate + chosen_relative_angle;

                        altered.emplace_back(plan_step(curr.position, curr.rotation, curr.action, curr.duration, curr.id, total_angle_to_rotate));
                        altered.emplace_back(plan_step(curr.position, normalize_angle(curr.rotation + total_angle_to_rotate), "endLost", 1000, curr.id));

                        //Do not want any more plan_steps once it is lost
                        break;
                    }
                    else { //else if the agent did not get lost                                                             

                        // angle_to_be_corrected is an angle that the agent will have to travell in order to be corrected (heading on 0/90/180/270 axis)
                        int angle_to_be_corrected = get_angle_to_be_corrected(future_uncorrected_starting_rotation, closest_angle_to_altered);

                        plan_step curr = original[j];

                        //first, uncorrected turn
                        altered.emplace_back(plan_step(curr.position, curr.rotation, curr.action, curr.duration, curr.id, curr.angle_to_rotate + chosen_relative_angle));

                        //optional turn in order to be corrected
                        if (angle_to_be_corrected != 0) {

                            int duration_of_correction_rotate = static_cast<int>((static_cast<float>(std::abs(angle_to_be_corrected)) / 90.0) * static_cast<float>(curr.duration));

                            altered.emplace_back(plan_step(curr.position, normalize_angle(curr.rotation + curr.angle_to_rotate + chosen_relative_angle), curr.action, duration_of_correction_rotate, curr.id, angle_to_be_corrected));
                        }


                        //As I could have changed my direction, i need to set j+1 rotation to be correct for the future instruction
                        int total_angle_to_rotate = original[j].angle_to_rotate + chosen_relative_angle + angle_to_be_corrected;
                        original[j + 1].rotation = normalize_angle(original[j].rotation + total_angle_to_rotate);
                        original[j + 1].position = original[j].position;
                    }
                }
            }
            else if (original[j].action == "go") {

                plan_step curr = original[j];

                if (tmp_error_speed_max != 0) {
                    //roll the dices
                    size_t rolled_pair_index = roll_pair_and_get_its_index(speed_prob_map, speed_prob_sum, i, j);
                    //alter duration
                    curr.duration = original[j].duration + static_cast<int>(speed_prob_map[rolled_pair_index].second);
                }

                altered.push_back(curr);

                //correct the next step's newly potentionally uncorrect position                    
                pos future_step_pos = curr.position;

                if (curr.rotation == 0)
                    future_step_pos.y -= 2;
                else if (curr.rotation == 90)
                    future_step_pos.x += 2;
                else if (curr.rotation == 180)
                    future_step_pos.y += 2;
                else if (curr.rotation == 270)
                    future_step_pos.x -= 2;

                //if I will get lost
                if (is_position_out_of_map(future_step_pos) || is_obstacle_on_position(future_step_pos)) {

                    altered.emplace_back(plan_step(future_step_pos, curr.rotation, "endLost", 1000, curr.id));

                    //Do not want any more plan_steps once it is lost
                    break;
                }
                else {

                    original[j + 1].position = future_step_pos;
                    original[j + 1].rotation = original[j].rotation;
                }
            }
            else if (original[j].action == "wait") {
                //promote the possible changes
                plan_step curr = original[j];
                original[j + 1].position = curr.position;
                original[j + 1].rotation = curr.rotation;

                altered.push_back(curr);
            }
            else {
                //start and end just copyy
                altered.push_back(original[j]);
            }

        //end of plan_steps altering for cycle    
        }


        int time_counter_altered = set_time_diffs_of_agent_accordingly_to(i, altered, agents[i].get_original_plan());

        agents[i].set_altered_plan(std::move(altered), time_counter_altered);
        altered.clear();

        //And onto the next Agent!
    }
}

/* updates agent_plan_max_length.
* Goes through all agents and compares theyrs plan_length to agent_plan_max_length
*/
void Simulation::update_agent_plan_max_length() {
    agent_plan_max_length = 0;

    for (auto& agent : agents)
        if (agent.get_altered_plan_length() > agent_plan_max_length)
            agent_plan_max_length = agent.get_altered_plan_length();
}

/* Returns index to specified agent's timediff field relevant for time
*  if returns -1, the agent havent even finished his 1st plan_step and therefore it cannot be late/soon
*/
int Simulation::get_index_of_timediff_for_agent_at_time(int agents_index, int time) {
    int idx = -1;
    for (size_t j = 0; j < time_diffs_of_agents[agents_index].size(); j++) {
        if (time_diffs_of_agents[agents_index][j].time_stamp <= time) //If I have already finished j-th step
            idx = j; //Remember it
        else
            break;
    }

    return idx;
}


/**************************************************************************************************************************************************************************/
/************************************************************   LOAD PLANS   *********************************************************************************************/


/*
* Puts the concrete Solver Option Predicat Name into the line
*/
void Simulation::load_plans_get_SolverOptionPredicatName(std::ifstream& myfile, std::string& line) {
    while (std::getline(myfile, line)) {
        if (line == "#SolverOptionPredicatName") { //The line before the 
            std::getline(myfile, line);
            return;
        }
    }

    //shouldn't get here
    return;
}

/*
* Puts the firsts agent definition into the line
*/
void Simulation::load_plans_get_FirstAgentDefinition(std::ifstream& myfile, std::string& line) {
    while (std::getline(myfile, line)) {
        if (line == "#Agents: AgentNO AgentName AgentColor (startX,startY) (endX,endY)") { //The line before the agents definitions
            std::getline(myfile, line); //to get the firsts agent definition line
            break;
        }
    }

    //shouldn't get here
    return;
}

void Simulation::load_plans_load_all_agents(std::ifstream& myfile, std::string& line, std::vector<Agent>& agents_vector, const std::vector<int>& errors_vectors, const std::string& delimiter) {

    load_plans_get_FirstAgentDefinition(myfile, line);
    //now I have 1st agent definition in line

    size_t idx = 0;
    std::string agt_name;
    std::string agt_color;
    pos agt_start;
    pos agt_finish;

    while (line != "") { //There is a blank line after the agents definitions

        idx = line.find(delimiter);
        line.erase(0, idx + delimiter.length()); //I dont care about "IDs"

        idx = line.find(delimiter);
        agt_name = line.substr(0, idx); //name of agent
        line.erase(0, idx + delimiter.length());

        idx = line.find(delimiter);
        agt_color = line.substr(0, idx); //color of agent
        line.erase(0, idx + delimiter.length());


        agt_start.x = std::stof(&line[1]);
        idx = line.find(',');
        agt_start.y = std::stof(&line[idx + 1]);

        idx = line.find(delimiter);
        line.erase(0, idx + delimiter.length());

        agt_finish.x = std::stof(&line[1]);
        idx = line.find(',');
        agt_finish.y = std::stof(&line[idx + 1]);

        agents_vector.emplace_back(Agent(agt_name, agt_color, agt_start, agt_finish, errors_vectors));


        std::getline(myfile, line);
    }
}

/*
* Loads the map
*/
void Simulation::load_plans_load_map(std::ifstream& myfile, std::string& line, std::vector<std::vector<char>>& map) {

    size_t map_x = 0;
    size_t map_y = 0;

    while (std::getline(myfile, line)) {
        if (line == "#MapSizeX,MapSizeY") {
            std::getline(myfile, line); //get the data

            map_x = static_cast<size_t>(std::stoi(&line[1])) * 2 - 1; //mapsize X
            map_y = static_cast<size_t>(std::stoi(&line[line.find(',') + 1])) * 2 - 1; //mapsize Y

            break;
        }
    }
    //now I have the correct map sizes


    //Set the map size accordingly
    map.resize(map_y);
    for (size_t i = 0; i < map_y; i++)
        map[i].resize(map_x, '.'); // '.' means travesible


    //Set untraversible spots which are there because the map is a grid
    for (size_t y = 0; y < map_y; y++)
        for (size_t x = 0; x < map_x; x++)
            if (y % 2 == 1 && x % 2 == 1)
                map[y][x] = '@'; //Due to the map format in use



    //Now load the untraversible spots which are there because the map has obstacles
    std::getline(myfile, line); //
    std::getline(myfile, line); // Skip the boring lines

    while (line != "") { //while I am loading obstacles
        map_x = std::stoi(&line[1]);
        map_y = std::stoi(&line[line.find(',') + 1]);

        map[map_y][map_x] = '@'; //'@' stands for obstacle

        std::getline(myfile, line);
    }
}

void Simulation::load_plans_load_num_to_vertex_map(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex) {

    //Get to the first usefull line
    while (std::getline(myfile, line))
        if (line == "#PicatIdToVertexNum: VertexNum->(PositionX,PositionY)") {
            std::getline(myfile, line);
            break;
        }

    //load all mappings
    pos act;
    while (line != "#AgentNO VertexNo Rotation Action Duration SomeText ") {
        act.x = std::stof(&line[line.find('(') + 1]);
        act.y = std::stof(&line[line.find(',') + 1]);
        num_to_vertex.emplace(std::stoi(&line[0]), act);

        std::getline(myfile, line);
    }
}


/*
* plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
*/
void Simulation::load_plan_load_and_normalize_agents_plans(std::ifstream& myfile, std::string& line, std::map<int, pos>& num_to_vertex,
    std::vector<Agent>& agents_vector, const std::string& solver_name, const std::vector<int>& plan_step_durations) {

    size_t agt_idx = 0;
    std::string token;

    pos agt_position;
    int agt_rotation;
    int agt_duration;
    std::string agt_action;

    int plan_step_id = 0;
    std::vector<plan_step> agt_plan;

    //loading plans
    while (std::getline(myfile, line)) {

        //If I have finished loading the agent
        if (line == "") {
            //Give agent its plan
            agents_vector[agt_idx].set_original_plan(std::move(agt_plan));

            //Get it ready for another round
            agt_plan.clear();
            ++agt_idx;
            continue;
        }

        std::istringstream iss(line);

        //Skip AgentNO
        std::getline(iss, token, ' ');

        //Get the vertex num
        std::getline(iss, token, ' ');
        agt_position = num_to_vertex[std::stoi(token)];

        // Get rotation
        std::getline(iss, token, ' ');
        if (token == "null")
            agt_rotation = 0;
        else
            agt_rotation = std::stoi(token);

        // Get action
        std::getline(iss, agt_action, ' ');

        // Get duration
        std::getline(iss, token, ' ');
        agt_duration = std::stoi(token);

        //And I ignore "SomeText"

        //id==0 is irrelevant here
        auto normalized_plan_steps = get_normalized_plan_steps_from(plan_step(agt_position, agt_rotation, agt_action, agt_duration, 0), solver_name, agt_plan, plan_step_durations, plan_step_id);

        //Merge them
        for (size_t i = 0; i < normalized_plan_steps.size(); i++)
            agt_plan.push_back(std::move(normalized_plan_steps[i]));


    }
}

/* Retuns vector of normalized plan_steps from plan_step step
* plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
* increments refference to plan_step_id
*/
std::vector<plan_step> Simulation::get_normalized_plan_steps_from(plan_step&& step, const std::string& solver_name, const std::vector<plan_step>& preceding_normalized_agent_plan_step, const std::vector<int>& plan_step_durations, int& plan_step_id) {

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

int Simulation::cardinal_to_angle(const std::string& str) {
    if (str == "north") return 0;
    else if (str == "east") return 90;
    else if (str == "south") return 180;
    else if (str == "west") return 270;

    return 0;
}


/*
* plan_step_durations[0] == go_duration, plan_step_durations[1] == turn_duration, plan_step_durations[2] == wait_duration
*/
void Simulation::load_plans(std::string path, const std::vector<int>& plan_step_durations) {

    //load vector of errors  
    std::vector<int> err = { 0, 0, 0, 0, 0 };

    //Reseting Stuff
    agents.clear();
    map.clear();
    time_diffs_of_agents.clear();
    agent_plan_max_length = 0;


    std::string line;
    std::ifstream myfile;

    std::string delimiter = " ";
    std::string solver_option_predicat_name;



    myfile.open(path);
    if (myfile.is_open()) {

        load_plans_get_SolverOptionPredicatName(myfile, line);
        solver_option_predicat_name = line;

        load_plans_load_all_agents(myfile, line, agents, err, delimiter);
        //Now I have loaded all the agents

        load_plans_load_map(myfile, line, map);
        //Now I have my map loaded

        std::map<int, pos> num_to_vertex;
        load_plans_load_num_to_vertex_map(myfile, line, num_to_vertex);
        //now I have map from vertices IDs to theyr real positions


        //And finally it is time to load and normalize agents plans
        load_plan_load_and_normalize_agents_plans(myfile, line, num_to_vertex, agents, solver_option_predicat_name, plan_step_durations);

        myfile.close();
    }
    else {

        std::cout << "Cannot find file " + path;
    }



    if (time_diffs_of_agents.size() != agents.size()) //Time diffs always have the right size
        time_diffs_of_agents.resize(agents.size());

    alter_plans_of_agents(0, agents.size());
    update_agent_plan_max_length();
}



/****************************************************************************************************************************************/
/****************************************************************************************************************************************/


/* moves all agents of simulation to specified time
* Returns true if At least one agent moved. When it returns False, there is no point in moving to bigger time.
* if time < 0 it moves to time == 0 and returns false
*/
bool Simulation::move_to_time(int time) {

    if (time < 0) {
        //Move to the closest possible state -> zero
        move_to_time(0);

        //But return false anyway
        return false;
    }


    int idx = 0;

    for (size_t i = 0; i < agents.size(); i++) {
        bool not_lost = agents[i].move_to_time(time);

        if (not_lost) {
            //Check if the time difference for agent[i] at time 'time' is too big

            idx = get_index_of_timediff_for_agent_at_time(i, time);
            //Now in the idx is the index of the last step I have finished (-1 == I havent finished even the first one)

            if (idx == -1) //If I havent even finished first step (index 0), I cannot be too late/soon
                agents[i].set_error_state(0);
            else if (std::abs(time_diffs_of_agents[i][idx].time_diff) >= ERROR_TRESHOLD) //If I am too late/soon
                agents[i].set_error_state(1);
            else
                agents[i].set_error_state(0); //Otherwise I am ok
        }
        else {
            //5 means lost
            agents[i].set_error_state(5);
        }
    }

    if (time > agent_plan_max_length) {
        return false;
    }
    else {
        return true;
    }
}




std::vector<std::string> Simulation::get_agents_names() {
    std::vector<std::string> ret;

    for (size_t i = 0; i < agents.size(); i++) {
        ret.push_back(agents[i].get_name());
    }
    return ret;
}

/* Sets Agent's errors and also alters his plan.
*  Also updates agent_plan_max_length so it stays correct
*  parameters: 0 = speed max, 1 = speed sigma, 2 = angle max, 3 = angle sigma,4 = fatal angle
*/
void Simulation::set_errors_to_agent(std::string& agent_name, const std::vector<int>& parameters) { 

    for (size_t i = 0; i < agents.size(); i++) {
        if (agents[i].get_name() == agent_name) {
            agents[i].set_errors(parameters);
            alter_plans_of_agents(i, i + 1);

            update_agent_plan_max_length();
            break;
        }
    }

}

/*
* returns how late/soon the agent is at specified time
*/
int Simulation::get_time_diffs_of_agent_at_time(int agent_index, int time) {
    int timediff_idx = get_index_of_timediff_for_agent_at_time(agent_index, time);

    if (timediff_idx == -1)
        return 0;
    else
        return time_diffs_of_agents[agent_index][timediff_idx].time_diff;
}

/*
* Gives only const reference
*/
const std::vector<std::vector<char>>& Simulation::show_map() {
    return map;
}

/*
* Gives only const reference
*/
const std::vector<Agent>& Simulation::show_agents() {
    return agents;
}

/*
* returns the time spam of the simulation
*/
int Simulation::get_agent_plan_max_length() {
    return agent_plan_max_length;
}