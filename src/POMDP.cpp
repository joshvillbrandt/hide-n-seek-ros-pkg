/*
 * File: hide_n_seek/src/fake_people_finder.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>,
 * 		Prithvi Balaram <balaram@usc.edu>,
 * 		Divya Sharma <divyasha@usc.edu>
 * Date: April 2012
 * Description:
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>
#define PI 3.14159265

class POMDP {
	private:
		tf::TransformListener* tf_listener;
		bool beliefInitialized;
		bool beliefUpdated;
		geometry_msgs::Pose robotPose;
		move_base_msgs::MoveBaseGoal navGoal;
		float beliefResolution;
		uint beliefWidth;
		uint beliefHeight;
		geometry_msgs::Pose beliefOrigin;
		std::vector< std::vector<float> > belief;
		std::vector< std::vector<float> > policy;
		// TODO: make use of the parameter server!
		const static int wallOccupancy = 50; // from -1 to 100, what constitutes a wall in the map occupancy grid
		const static float observationDecayRate = 0.5; // this percent of the current belief is replaced upon new observation
		const static float reductionPerMeter = 0.1; // used to convert the belief state into nav goals
		const static int downsampleFactor = 10;

		void initializeBelief(const nav_msgs::OccupancyGridConstPtr &map) {
			this->beliefOrigin = map->info.origin;
			this->beliefResolution = map->info.resolution * POMDP::downsampleFactor;
			this->beliefWidth = map->info.width / POMDP::downsampleFactor;
			this->beliefHeight = map->info.height / POMDP::downsampleFactor;

			ROS_INFO_STREAM("initializing belief with state space " << this->beliefWidth*this->beliefHeight);

			std::vector<float> column(this->beliefHeight, -1);
			for(uint i = 0; i < this->beliefWidth; i++) {
				this->belief.push_back(column);
				this->policy.push_back(column);
			}

			this->beliefInitialized = true;
		}

	public:
		POMDP () :
			beliefInitialized(false),
			beliefUpdated(false)
		{
			this->tf_listener = new (tf::TransformListener);
		}

		~POMDP () {}

		bool isBeliefUpdated() {
			return this->beliefUpdated;
		}

		void calculatePolicy() {
			this->navGoal.target_pose.header.frame_id = "map";
			this->navGoal.target_pose.header.stamp = ros::Time::now();

			// find position
			uint bestI = 0;
			uint bestJ = 0;
			float bestProbability = 0;
			for(uint i = 0; i < this->beliefWidth; i++) {
				for(uint j = 0; j < this->beliefHeight; j++) {
					// start with the belief state as our policy
					this->policy[i][j] = this->belief[i][j];

					// add some noise for good measure - easy way to make sure the robot keeps moving and does get stuck
					if(this->policy[i][j] >= 0) this->policy[i][j] += rand() / double(RAND_MAX) * 0.15;

					// the most attractive nav goals are locations in range of the sensor
					float x = i * this->beliefResolution + this->beliefOrigin.position.x + this->beliefResolution / 2;
					float relX = x - this->robotPose.position.x;
					float y = j * this->beliefResolution + this->beliefOrigin.position.y + this->beliefResolution / 2;
					float relY = y - this->robotPose.position.y;
					float distance = fabs(sqrt(pow(relX,2) + pow(relY,2)) - 0.75*3.5); // 3.5m is the kinect sensor range
					this->policy[i][j] -= distance * POMDP::reductionPerMeter;

					// bound the policy
					if(this->policy[i][j] < 0) this->policy[i][j] = 0;

					if(this->policy[i][j] >= bestProbability) {
						bestProbability = this->policy[i][j];
						bestI = i;
						bestJ = j;
					}
				}
			}
			this->navGoal.target_pose.pose.position.x = bestI * this->beliefResolution + this->beliefOrigin.position.x + this->beliefResolution / 2;
			this->navGoal.target_pose.pose.position.y = bestJ * this->beliefResolution + this->beliefOrigin.position.y + this->beliefResolution / 2;
			this->navGoal.target_pose.pose.position.z = this->beliefOrigin.position.z;

			// find orientation between current and goal location
			this->navGoal.target_pose.pose.orientation.w = 1.0;

			ROS_INFO_STREAM("new nav goal is: " << this->navGoal.target_pose.pose.position.x << ", "
				<< this->navGoal.target_pose.pose.position.y << ", " << this->navGoal.target_pose.pose.position.z );

			this->beliefUpdated = false;
		}

		move_base_msgs::MoveBaseGoal getNavGoal() {
			return this->navGoal;
		}

		visualization_msgs::MarkerArray getBeliefMarkers() {
			return this->getMarkers(true);
		}

		visualization_msgs::MarkerArray getPolicyMarkers() {
			return this->getMarkers(false);
		}

		visualization_msgs::MarkerArray getMarkers(bool showBelief) {
			// base marker
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time();
			marker.ns = "belief";
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.z = this->beliefOrigin.position.z;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = -1.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.color.a = 1.0;

			// build array
			visualization_msgs::MarkerArray ma;
			for(uint i = 0; i < this->beliefWidth; i++) {
				for(uint j = 0; j < this->beliefHeight; j++) {
					// only show states which we know exist
					if(this->belief[i][j] >= 0) {
						if(showBelief) {
							marker.color.r = 1.0 - this->belief[i][j];
							marker.color.g = this->belief[i][j];
							marker.color.b = 0.0;
							marker.scale.z = this->belief[i][j];
						}
						else {
							marker.color.r = 0.0;
							marker.color.g = 1.0;
							marker.color.b = 0.0;
							marker.scale.z = this->policy[i][j];
						}

						marker.id = i * this->beliefWidth + j;
						marker.pose.position.x = i * this->beliefResolution + this->beliefOrigin.position.x + this->beliefResolution / 2;
						marker.pose.position.y = j * this->beliefResolution + this->beliefOrigin.position.y + this->beliefResolution / 2;
						ma.markers.push_back(marker);
					}
				}
			}

			return ma;
		}

		void updateBeliefFromMap(const nav_msgs::OccupancyGridConstPtr &map) {
			ROS_INFO("updating belief from map");
			if(!this->beliefInitialized) this->initializeBelief(map);

			// update belief
			for(uint i = 0; i < this->beliefWidth; i++) {
				for(uint j = 0; j < this->beliefHeight; j++) {
					// we only care about new parts of the map
					if(this->belief[i][j] < 0) {
						// down-sample
						float sum = 0;
						float wallCount = 0;
						for(uint m = i*POMDP::downsampleFactor; m < i*POMDP::downsampleFactor+POMDP::downsampleFactor; m++) {
							for(uint n = j*POMDP::downsampleFactor; n < j*POMDP::downsampleFactor+POMDP::downsampleFactor; n++) {
								sum += map->data[m + n * map->info.width];
								if(map->data[m + n * map->info.width] > POMDP::wallOccupancy) wallCount++;
							}
						}

						// give this space an initial probability if at least half of the space has been viewed
						if(sum / POMDP::downsampleFactor / POMDP::downsampleFactor >= -0.50) {
							// give this a much lower probability if we see a wall (if 3% of squares are walls, call this a wall)
							if(wallCount / POMDP::downsampleFactor / POMDP::downsampleFactor >= 0.02)
								this->belief[i][j] = 0.01;
							else this->belief[i][j] = 0.5;
							this->beliefUpdated = true;
						}
					}
				}
			}
		}

		void updateBeliefFromSensor(const hide_n_seek::PeopleFinderConstPtr &finder) {
			ROS_INFO("updating belief from sensor");

			// convert people to
			std::vector<geometry_msgs::PoseStamped> people;
			for(uint m = 0; m < finder->people.size(); m++) {
				geometry_msgs::PoseStamped personBase;
				personBase.header = finder->header;
				personBase.pose = finder->people[m];

				geometry_msgs::PoseStamped personMap;
				this->tf_listener->transformPose("map", personBase, personMap);
				people.push_back(personMap);
			}


			// update belief
			int startI = (this->robotPose.position.x - this->beliefOrigin.position.x - finder->sensor_range)/this->beliefResolution;
			int startJ = (this->robotPose.position.y - this->beliefOrigin.position.y - finder->sensor_range)/this->beliefResolution;
			if(startI < 0) startI = 0;
			if(startJ < 0) startJ = 0;
			for(uint i = (uint)startI; i <= startI+2*finder->sensor_range/this->beliefResolution; i++) {
				for(uint j = (uint)startJ; j <= startJ+2*finder->sensor_range/this->beliefResolution; j++) {
					// make sure the indexes are valid
					if(i < this->beliefWidth && j < this->beliefHeight) {
						// cell pose in map frame
						geometry_msgs::PoseStamped cellPoseMap;
						cellPoseMap.header.stamp = ros::Time();
						cellPoseMap.header.frame_id = "/map";
						cellPoseMap.pose.position.x = i * this->beliefResolution + this->beliefOrigin.position.x + this->beliefResolution / 2;
						cellPoseMap.pose.position.y = j * this->beliefResolution + this->beliefOrigin.position.y + this->beliefResolution / 2;
						cellPoseMap.pose.position.z = 0;
						cellPoseMap.pose.orientation.w = 1.0;

						// cell pose in base frame
						geometry_msgs::PoseStamped cellPoseBase;
						this->tf_listener->transformPose("base_link", cellPoseMap, cellPoseBase);

						// if within range
						// TODO: check for walls here - right now we see through walls!
						float range = sqrt(pow(cellPoseBase.pose.position.x,2) + pow(cellPoseBase.pose.position.y,2));
						if(cellPoseBase.pose.position.x > 0 && range <= finder->sensor_range) {
							bool inView = false;
							if(cellPoseBase.pose.position.y == 0) inView = true; // prevents division by zero
							else {
								float bearing = PI/2 - atan(cellPoseBase.pose.position.x/fabs(cellPoseBase.pose.position.y));
								if(bearing < finder->sensor_fov/2) inView = true;
							}

							if(inView) {
								// this point is in range of the sensor, check for people
								float minDistance = finder->sensor_range;
								for(uint m = 0; m < people.size(); m++) {
									float relX = people[m].pose.position.x - cellPoseMap.pose.position.x;
									float relY = people[m].pose.position.y - cellPoseMap.pose.position.y;
									float distance = fabs(sqrt(pow(relX,2) + pow(relY,2)));
									if(distance < minDistance) minDistance = distance;
								}

								// calculate new probability
								float newProbability;
								if(minDistance < finder->sensor_range) newProbability = 0.95 - minDistance * POMDP::reductionPerMeter;
								else newProbability = 0.05;

								// keep some of the history
								if(this->belief[i][j] >= 0.6)
									this->belief[i][j] = (1 - POMDP::observationDecayRate/2) * this->belief[i][j] + POMDP::observationDecayRate/2 * newProbability;
								else if(this->belief[i][j] >= 0)
									this->belief[i][j] = (1 - POMDP::observationDecayRate) * this->belief[i][j] + POMDP::observationDecayRate * newProbability;
							}
						}
					}
				}
			}

			this->beliefUpdated = true;
		}

		void updateRobotPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &odom_combined) {
			this->robotPose = odom_combined->pose.pose;
		}
};
