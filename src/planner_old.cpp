/**
 * Name: Planner
 * Author: Prithvi Balaram , Divya Sharma 
 *
 * This planner receives input from the global map as well as a PeopleFinder, and
 * accordingly makes and executes plans that send commands through a MoveBaseAction.
 *
 *NO : Update this... Heuristic Value = (1/(Number of States)) * (Observation Weight) * (Transition Weight)
 *
 * The planner keeps track of every possible state in the current world, assigning it
 * an observation weight and a transition weight.
 *
 * The observation weight is initialized to zero, and is then updated online as the
 * robot senses new objects near the given state.
 *
 * The transition weight between two states is (for now) calculated via the simple
 * Euclidean distance in two dimensions between the two states on the x and y axes.
 *
 * The planner executes its planning in belief space; it keeps track of only states
 * that have a belief or goal weight associated with them.
 *
 * Initially, we have the planner generate random beliefs about states that might be
 * a goal, and explore those in order to find a state that might actually be a goal.
 *
 * Once we have some states that might actually be a goal, we should travel to those
 * states and check them out, and then continue.
 *
 * So, at every given iteration, the planner must find out where it is, choose the next
 * believed goal state to travel to (the one with highest heuristic value relative to
 * the current state) and then travel to it.
 *
 * TODO: Fix fake observation probability.
 * TODO: Implement random walking before we find people.
 * TODO: Fix state heuristics being initialized to zero.
 * TODO: Keep track of a down sampled version of the map.
 * TODO: Change existing algorithms to work with down sampling.
 *
 */

#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <string>
#include <cmath>

#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <hide_n_seek/PeopleFinder.h>
#include <move_base_msgs/MoveBaseAction.h>

// MoveBaseClient takes care of providing interaction capability with the base.
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
		MoveBaseClient;

// State takes care of storing the believed goal information associated with a
// particular location on the map.
class State {
public:
	State() {
	}
	State(const geometry_msgs::Pose& p, const double obs) {
		this->pose.position.x = p.position.x;
		this->pose.position.y = p.position.y;
		this->obs_weight = obs;
	}

	State(const State& other) {
		this->pose.position.x = other.pose.position.x;
		this->pose.position.y = other.pose.position.y;
		obs_weight = other.obs_weight;
	}

	virtual ~State() {
	}

	double obs_weight;
	geometry_msgs::Pose pose;
	geometry_msgs::Pose state_space_pose;
	bool unreachable;

	bool operator!=(const State &other) {
		return (this->state_space_pose.position.x
				!= other.state_space_pose.position.x
				&& this->state_space_pose.position.x
						!= other.state_space_pose.position.x);
	}

	State& operator=(const State &other) {
		if (this != &other) {
			this->pose = other.pose;
			this->state_space_pose = other.state_space_pose;
			obs_weight = other.obs_weight;
		}
		return *this;
	}

};

// POMDP data structure, including our list of goal states as well as the
// current state and the calculation functions operating upon those pieces
// of data.
class POMDP {
public:
	int num_states;
	int num_lookahead;
	bool states_inited;
	State current_state;
	std::vector<State> states;
};

POMDP pomdp;
nav_msgs::OccupancyGridConstPtr map;
std::vector<geometry_msgs::Pose> people;
nav_msgs::OdometryConstPtr odometry;
bool odometry_inited = false;

// Faking an observation probability for now.
double fake_observation_probability = 0.8;
double value_iteration_decay_factor = 0.5;
double explored_decay_factor = 0.8;

ros::Publisher vis;

geometry_msgs::Pose stateToRealPose(const geometry_msgs::Pose s) {
	for (std::vector<State>::iterator state = pomdp.states.begin(); state
			!= pomdp.states.end(); state++) {
		if (state->state_space_pose.position.x == s.position.x
				&& state->state_space_pose.position.y == s.position.y) {
			return state->pose;
		}
	}
}

geometry_msgs::Pose realToStatePose(const geometry_msgs::Pose real) {
	//	int state_space_width = sqrt(pomdp.num_states);
	//	int state_space_height = sqrt(pomdp.num_states);
	geometry_msgs::Pose statePose;
	statePose.position.x = (real.position.x - map->info.origin.position.x) / 2;
	statePose.position.y = (real.position.y - map->info.origin.position.y) / 2;

	visualization_msgs::Marker marker;
	marker.header.frame_id = map->header.frame_id;
	marker.id = 67889;
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = real.position.x;
	marker.pose.position.y = real.position.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 1.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = 2.0;
	marker.scale.x = 2.0;
	marker.scale.y = 2.0;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	vis.publish(marker);

	marker.header.frame_id = map->header.frame_id;
	marker.id = 67845;
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = statePose.position.x;
	marker.pose.position.y = statePose.position.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = -1.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.z = 2.0;
	marker.scale.x = 2.0;
	marker.scale.y = 2.0;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 1.0;
	vis.publish(marker);

	ROS_INFO("Real pose: (%f, %f) State Pose: (%f, %f)", real.position.x, real.position.y, statePose.position.x, statePose.position.y);
	return statePose;
}

void updateRobotPose(const nav_msgs::OdometryConstPtr &odom) {
	odometry = odom;
	odometry_inited = true;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.id = 20000;
	marker.header.stamp = ros::Time();
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = odometry->pose.pose.position.x;
	marker.pose.position.y = odometry->pose.pose.position.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = -1.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	vis.publish(marker);
	return;
}

// Select some random states from the initial map so that we can randomly explore.

void normaliseDistribution(double normSum){

	//normalisation of the weights
	if(normSum !=0.0){
			for (unsigned int i = 0; i < pomdp.states.size(); i++) {
						State s = pomdp.states.at(i);
						s.obs_weight = s.obs_weight/normSum;

			}
	}else{

		ROS_INFO(" we have bad data from map or normal distribution");
	}

}
void updateStates() {
	ROS_INFO("Updating states.");

	//add states from Map, intially add all of them , later age them with time and multiply with the one received from map [0,100]
	//int real_space_width = map->info.width;
	//int real_space_height = map->info.height;
	int state_space_width = sqrt(pomdp.num_states);
	int state_space_height = sqrt(pomdp.num_states);
  double normSum = 0.0;
	// if the states haven't been inited, init them
	if (!pomdp.states_inited) {

		int count = 0;
		// initially add all the states
		for (int i = 0; i < state_space_width; i++) {
			for (int j = 0; j < state_space_height; j++) {
				State s;
				//	s.obs_weight = 1/pomdp.num_states;
				if(map->data[count] == -1 || map->data[count+1] == -1 || map->data[count] >80 || map->data[count+1] >80 )//|| map->data[count+2] == -1 ||map->data[count+3] == -1)
				{
				s.obs_weight=1/pomdp.num_states;
				s.unreachable=true;
				count+=2;
				}
				else{
				s.obs_weight =(1/map->data[count] * 1/map->data[count++]) ;//*1/map->data[count++]*1/map->data[count++]);
				s.unreachable=false;
				}
				s.pose.position.x = map->info.origin.position.x + 2 * i;
				s.pose.position.y = map->info.origin.position.y + 2 * j;

				s.state_space_pose.position.x = i;
				s.state_space_pose.position.y = j;
				pomdp.states.push_back(s);
				normSum=normSum +s.obs_weight;

			}
		}

		normaliseDistribution(normSum);
		pomdp.states_inited = true;
		ROS_INFO("Initialized states vector with size of %d", (int) pomdp.states.size());
	}

	// really the states should always be inited by this point, but
	if (pomdp.states_inited) {
		//do not access the pomdp since it is getting updated
		normSum=0.0;
		int count = 0;
		// update the states with the new probabilities from the map, poses remain the same
		for (unsigned int i = 0; i < pomdp.states.size(); i++) {
			State s = pomdp.states.at(i);
			if(map->data[count] == -1 || map->data[count+1] == -1 || map->data[count] > 80 || map->data[count+1] > 80)//|| map->data[count+2] == -1 ||map->data[count+3] == -1)
			{
				s.obs_weight=1/pomdp.num_states;
							count+=2;
							s.unreachable=true;
							}
			else{
				s.obs_weight = (1 / map->data[count] * 1/ map->data[count++]) ;//* 1 / map->data[count++] * 1/ map->data[count++]);

				s.unreachable=false;
			}
			normSum=normSum +s.obs_weight;
			// convert the x, y coordinates to get the obstacle probability at that location
		}
		normaliseDistribution(normSum);

	}

	ROS_INFO("Norm sum is %f.", normSum);
	//		for (unsigned int i = 0; i < map->data.size(); i++) {
	//
	//			// figure out which state we're modifying
	//			real_pose.position.x = i % state_space_width;
	//			real_pose.position.y = i / state_space_height;
	//			state_space_pose = realToStatePose(real_pose);
	//
	//			// modify the obstacle weight of that state.
	//			for (std::vector<State>::iterator state = pomdp.states.begin(); state != pomdp.states.end(); state++) {
	//				if (state->state_space_pose.position.x == state_space_pose.position.x
	//						&& state->state_space_pose.position.y == state_space_pose.position.y) {
	//					// age the previous obstacle weight
	//					state->obs_weight = state->obs_weight * value_iteration_decay_factor; // set to 0.5 at top of file
	//					// multiply by the new obstacle weight. if it's -1, it's unreachable, so effectively remove this state from calculation by setting to zero.
	//					if (map->data.at(i) == -1) {
	//						state->obs_weight = 0.1 * state->obs_weight;
	//					} else {
	//						state->obs_weight = state->obs_weight + 1/map->data[i];
	//					}
	//					break;
	//				}
	//			}
	//		}
	//	ROS_INFO("Done with map. State size is %d.", (int)pomdp.states.size());
	//	} else {
	//	ROS_ERROR("Why are we here?");
	//}
	return;
}

void printGoals(ros::Publisher vis) {
	std::vector<visualization_msgs::Marker> markers;
	ROS_INFO("Printing goals, %d total.", (int)pomdp.states.size());
	for (unsigned int i = 0; i < pomdp.states.size(); i++) {
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.id = i;
		marker.header.stamp = ros::Time();
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::MODIFY;
		marker.pose.position.x = pomdp.states.at(i).pose.position.x;
		marker.pose.position.y = pomdp.states.at(i).pose.position.y;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = -1.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		if (pomdp.states.at(i).obs_weight == 0.0f) {
			marker.scale.y = 1.0;
			marker.pose.orientation.y = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 0.0;
			marker.color.r = 0.0;
		} else {

			marker.scale.y = 1.0;
			if (pomdp.states.at(i).obs_weight < 0.1) {
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;
				marker.pose.orientation.y = 1.0;
			} else {
				marker.color.r = 0.0;
				marker.color.b = 1.0;
				marker.color.g = 0.0;
			}
			marker.scale.z = pomdp.states.at(i).obs_weight;

		}

		vis.publish(marker);
		//ROS_INFO("Goal: X(%f), Y(%f) W(%f)", state->pose.position.x , state->pose.position.y, state->obs_weight);
	}
	return;
}

//void addPerson(const geometry_msgs::Pose pose) {
//	ROS_INFO("Adding person to goal states.");
//
//	geometry_msgs::Pose state_pose = realToStatePose(pose);
//	// check to see if this pose is already in there
//	for (std::vector<State>::iterator state = pomdp.states.begin(); state != pomdp.states.end(); state++) {
//		if (state->state_space_pose.position.x == pose.position.x
//				&& state->state_space_pose.position.y == pose.position.y) {
//			state->obs_weight = state.obs_weight ;
//		}
//	}
//	// add the state
//	State s(pose, fake_observation_probability);
//	pomdp.states.push_back(s);
//	return;
//}

void updatePeople(const hide_n_seek::PeopleFinderConstPtr &finder) {
	//ROS_INFO("Updating people. Amount of new people found is %d.", (int)people.size());
	for (unsigned int i = 0; i < finder->people.size(); i++) {
		people.push_back(finder->people.at(i));
	}
	return;
}

void updateMap(const nav_msgs::OccupancyGridConstPtr &og) {
	ROS_INFO("Updating the map.");

	//ROS_INFO("Map width: %d height: %d", og->info.width, og->info.height);
	map = og;
	updateStates();

	//map_inited = true;
	return;
}

double euclideanDistance2D(State s1, State s2) {
	double delta_x = s1.pose.position.x - s2.pose.position.x;
	double delta_y = s2.pose.position.y - s2.pose.position.y;
	return sqrt(pow(delta_x, 2) + pow(delta_y, 2));
}

State getMostLikelyState() {
	ROS_INFO("Getting most likely state.");
	if (!pomdp.states_inited) {
		State s;
		ROS_ERROR("We're trying to get the most likely state when states have not been initialized.");
		return s;
	}

	float prob = 0;

	State goal;
	int index=0;
	int goalIndex = 0;
	for (std::vector<State>::iterator state = pomdp.states.begin(); state != pomdp.states.end(); state++) {
		index++;
		if (state->unreachable) {
			ROS_INFO("state is unreachable");
		}

		ROS_INFO("State obs_weight is %f", state->obs_weight);
		if (!state->unreachable && state->obs_weight > prob) {
			prob = state->obs_weight;
			goal = *state;
			goalIndex=index;
		}
	}
	ROS_INFO("Goal: X(%f), Y(%f) W(%f)", goal.pose.position.x , goal.pose.position.y, goal.obs_weight);
//age the prob of the current goal by  0.5

	pomdp.states.at(goalIndex).obs_weight = 0.5*pomdp.states.at(goalIndex).obs_weight;
	return goal;
}

double calculateReward(State goal, int lookahead) {
	ROS_INFO("Calculating reward for state. Lookahead is %d", lookahead);
	double reward = -1;
	//get the data from /goal , may be store it in a list and use
	//assuming the goal gets with only one person
	double normSum=0.0;
	if (lookahead == 1) {
		for (std::vector<State>::iterator state = pomdp.states.begin(); state != pomdp.states.end(); state++) {
			if (goal.pose.position.x != state->pose.position.x
					&& goal.pose.position.y != state->pose.position.y) {
				//only only 95 out of 100 times there is a person actually
				if (state->obs_weight == 1/pomdp.num_states) {
					ROS_INFO("State observation weight is 1/pomdp.num_states!");
					//as per map thats unreachable/unexplored  so keep it zero
					continue;
				} else {
					state->obs_weight = state->obs_weight + (1
							/ euclideanDistance2D(goal, *state) *.95 );
					//ROS_INFO("euc dist %f",euclideanDistance2D(goal, *state));
					if (state->obs_weight > reward) {
						reward = state->obs_weight;
					}
				}
			} else {
				state->obs_weight=0.8;
				ROS_INFO("goal is an internal state, reward set to very high");
			}
			normSum=normSum+state->obs_weight;

		}

		normaliseDistribution(normSum);
		ROS_INFO("Norm sum is %f.", normSum);
	} /*else {
	 for (std::vector<State>::iterator state = pomdp.states.begin(); state
	 != pomdp.states.end(); state++) {
	 if (goal != *state) {
	 if (state->obs_weight == 0) {
	 continue;
	 }
	 state->obs_weight = calculateReward(*state, lookahead - 1) + (1
	 / euclideanDistance2D(goal, *state) * 0.95)
	 + state->obs_weight;
	 if (state->obs_weight > reward) {
	 reward = state->obs_weight;
	 }
	 } else {
	 }
	 }
	 }*/
	ROS_INFO("Calculated a reward of %f.", reward);
	return reward;
}

State makePlan(State realSpaceGoal) {
	ROS_INFO("Making a plan. State amount is %d.", (int) pomdp.states.size());

	// Calculate the best next state to get to that goal.
	calculateReward(realSpaceGoal, pomdp.num_lookahead);


	return getMostLikelyState();
}

void sendGoal(State goalState) {
	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map"; //"base_link";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = goalState.pose.position.x;
	goal.target_pose.pose.position.y = goalState.pose.position.y;
	goal.target_pose.pose.position.z = 0;
	goal.target_pose.pose.orientation.w = 1.0;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	int decay_factor = 1;
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Hooray, the base moved to (%f, %f).", goalState.pose.position.x, goalState.pose.position.y);
		decay_factor = explored_decay_factor;
	} else {
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		decay_factor = 0.4;
	}
	//	for (std::vector<State>::iterator state = pomdp.states.begin(); state
	//			!= pomdp.states.end(); state++) {
	//		if (goalState.state_space_pose.position.x == state->state_space_pose.position.x
	//				&& goalState.state_space_pose.position.y == state->state_space_pose.position.y) {
	//			state->obs_weight = state->obs_weight * decay_factor;
	//		}
	//	}
	ac.cancelGoal();
	return;
}

void updateCost(const nav_msgs::GridCellsConstPtr &gc) {
	ROS_INFO("Updating Cost.");
	for (unsigned int i = 0; i < gc->cells.size(); i++) {
		ROS_INFO("Obstacle found at (%f, %f)", gc->cells.at(i).x, gc->cells.at(i).y);
	}
}

int main(int argc, char** argv) {

	// Initialize ourselves as a ROS node.
	ros::init(argc, argv, "planner");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.5); //hz
	vis = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 0);

	// Subscribe to the map and people finder data.
	ros::Subscriber map_sub;
	ros::Subscriber people_finder_sub;
	ros::Subscriber robot_pose_sub;
	//ros::Subscriber costmap_sub;
	//costmap_sub = nh.subscribe<nav_msgs::GridCells>("/move_base_node/local_costmap/obstacles", 1, updateCost);
	map_sub = nh.subscribe<nav_msgs::OccupancyGrid> ("/map", 1, updateMap);
	people_finder_sub = nh.subscribe<hide_n_seek::PeopleFinder> (
			"fake_people_finder/people_finder", 1, updatePeople);
	robot_pose_sub = nh.subscribe<nav_msgs::Odometry> (
			"/base_pose_ground_truth", 1, updateRobotPose);

	// map_inited = false;
	pomdp.num_states = 400;
	pomdp.num_lookahead = 1;
	pomdp.states_inited = false;
	//pomdp.current_state.pose.position.x = 0;
	//pomdp.current_state.pose.position.y = 0;


	ROS_INFO("States initialized.");

	// spin the main loop
	while (ros::ok()) {
		ROS_INFO("service loop...");

		 {

			State internalGoal,personGoal;
			State s;
			if (people.size() != 0) {
				// If we have people to explore, exploring them is first priority.
				personGoal.pose = people.at(0);
				//this  is a real world pose, now convert it into one of the pompdp state to calculate the euc dist from all poss states
				personGoal.state_space_pose  = realToStatePose(personGoal.pose);
				s = makePlan(personGoal);
				ROS_INFO("Person found at real(%f, %f)! Using them as the next goal.", personGoal.pose.position.x, internalGoal.pose.position.y);
			} else if (pomdp.states_inited){
				// get goal from POMDp with highest probability, if its an internal state no conversion needed

				internalGoal = getMostLikelyState();
				s = makePlan(internalGoal);

			}
			//internalGoal.state_space_pose = realToStatePose(internalGoal.pose);
			// once we've initialized some states...
			// make a plan

			ROS_INFO("State received from makePlan: (%f, %f)", s.pose.position.x, s.pose.position.y);

			if (people.size() != 0) {
				people.erase(people.begin());
			}
			// print current goals
			printGoals(vis);
			// send a goal
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";//->header.frame_id;
			marker.id = 67845;
			marker.header.stamp = ros::Time();
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = s.pose.position.x;
			marker.pose.position.y = s.pose.position.y;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = -1.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.z = 2.0;
			marker.scale.x = 2.0;
			marker.scale.y = 2.0;

			marker.color.r = 1.0;
			marker.color.g = 1.0;
			marker.color.b = 1.0;
			vis.publish(marker);
			sendGoal(s);
			ROS_INFO("Goal we're sending: (%f, %f)", s.pose.position.x, s.pose.position.y);

			// set the current state
			pomdp.current_state = s;

		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
