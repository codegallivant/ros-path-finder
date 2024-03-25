#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>


using namespace std;

int get_index(int row, int col, int ncols) { // Row and col go from 1 to n
    return ((row-1)*(ncols))+col-1;
}

void get_row_col(int* node_dim, int index, int ncols) { // Row and col go from 1 to n
    int row = ceil((index+1.0)/ncols);
    int col = (index+1)-((row-1)*(ncols));
    node_dim[0] = row;
    node_dim[1] = col;
}

float euclidean_heuristic(int node_index, int goal_node_index, int ncols) {
    int node_dim[2];
    int goal_dim[2];
    get_row_col(node_dim, node_index, ncols);
    get_row_col(goal_dim, goal_node_index, ncols);
    return sqrt(pow(node_dim[0]-goal_dim[0],2) + pow(node_dim[1]-goal_dim[1],2));
}

vector<int> astarsearch(vector<int> obstacles, int start_node_index, int goal_node_index, int nrows, int ncols) {
    
    vector<int> g_cost(nrows*ncols, -1);
    g_cost[start_node_index] = 0;
    
    vector<int> source_nodes(nrows*ncols, -1);
    source_nodes[start_node_index] = 0;
    
    int current_node_index = start_node_index;
    
    vector<int> unvisited;// List of all nodes whose neighbours have not been checked
    vector<int> visited; // closed_list List of all nodes whose neighbours have been checked

    unvisited.push_back(current_node_index);

    int index_minimum_h;
    int unvisited_node_h;

    vector<int> neighbour_node_indexes;
    int neighbours[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
    int node_dim[2];
    int newrow, newcol;
        
    int found = 0;
    
    // int i = 0;
    while(true) {
        // i = i+1;
        // if(i==20) {
        //     break;
        // }

        // Finding visited node with least cost
        current_node_index = unvisited[0];
        for(int &node_index : unvisited) {
            index_minimum_h = euclidean_heuristic(current_node_index, goal_node_index, ncols);
            unvisited_node_h = euclidean_heuristic(node_index, goal_node_index, ncols);
            if (g_cost[node_index] + unvisited_node_h < g_cost[current_node_index] + index_minimum_h) {
                current_node_index = node_index;
            }
        }
        get_row_col(node_dim, current_node_index, ncols);
        // cout << "CNI" << node_dim[0] << " " << node_dim[1];
        visited.push_back(current_node_index);
        for(int i = 0;i < unvisited.size(); i++) {
            if (unvisited[i] == current_node_index) {
                unvisited.erase(unvisited.begin()+i);
            }
        }

        neighbour_node_indexes.clear();        
        for(int i = 0;i<4;i++) {
            newrow = node_dim[0]+neighbours[i][0];
            newcol = node_dim[1]+neighbours[i][1];
            if (0<newrow && 0<newcol && newrow<=nrows && newcol<=ncols) {
                neighbour_node_indexes.push_back(get_index(newrow, newcol, ncols));
            }
        }
        // Setting g for neighbour nodes
        for(int &neighbour_node_index : neighbour_node_indexes) {
            // cout << "nni: "<< neighbour_node_index << endl;
            if (0<=neighbour_node_index && neighbour_node_index<=(nrows*ncols-1)) {
                if (obstacles[neighbour_node_index] == 0 ) {

                    // cout << "nni: "<< neighbour_node_index << endl;
                    // cout << "gc" << g_cost[neighbour_node_index] << endl;
                    if (g_cost[neighbour_node_index] == -1) {
                        unvisited.push_back(neighbour_node_index);
                    }
                    if ((g_cost[neighbour_node_index] == -1) || ((g_cost[current_node_index]+1)<g_cost[neighbour_node_index])) {
                        g_cost[neighbour_node_index] = g_cost[current_node_index] + 1;  
                        source_nodes[neighbour_node_index] = current_node_index;
                        // cout << "h:" << euclidean_heuristic(neighbour_node_index, goal_node_index, dim) << endl;
                        if(euclidean_heuristic(neighbour_node_index, goal_node_index, ncols)==0.0) {
                            found = 1;
                            break;
                        }
                    }
                }
            }
        }
        if(found == 1) {
            break;
        }
        
    }

    // Backtracking and compiling path
    vector<int> path;
    int source = goal_node_index;
    path.push_back(goal_node_index);
    while(source != start_node_index) {
        source = source_nodes[source];
        path.push_back(source);
    }
    reverse(path.begin(), path.end());
    return path;
}


class SubscribeAndPublish {
	public:
		SubscribeAndPublish() {
			path_pub = n.advertise<nav_msgs::Path>("/path",1, true);
			map_sub = n.subscribe("/map",1, &SubscribeAndPublish::mapCallback, this);
		}

		void mapCallback(const nav_msgs::OccupancyGrid &msg){
			std_msgs::Header header = msg.header;
			nav_msgs::MapMetaData info = msg.info;
			ROS_INFO("Got map %d %d", info.height, info.width);
			int start[2] = {1, 1};
			int goal[2] = {(int)info.height, (int)info.width};
			
			vector<int> obstacles;
			for(int i = 0;i<info.height*info.width;i++) {
				obstacles.push_back(msg.data[i]);
                cout << obstacles[i] << " ";
            }
            cout << endl;

			vector<int> path = astarsearch(obstacles, get_index(start[0], start[1], info.width), get_index(goal[0], goal[1], info.width), info.height, info.width);
			
			nav_msgs::Path gui_path;
			
			geometry_msgs::PoseStamped pose;
			std::vector<geometry_msgs::PoseStamped> plan;
			int node_dim[2];
            cout << "size" << path.size() << endl;
            for (int &pathnode : path) {
				pose.header.stamp = ros::Time::now();
				// pose.header.frame_id = "base_link";
				get_row_col(node_dim, pathnode, info.width);
				pose.pose.position.x = node_dim[1];
				pose.pose.position.y = node_dim[0];
                cout << node_dim[0] << " " << node_dim[1] << endl;   
                plan.push_back(pose);            
            }
			// for (int i = path.size() - 1; i >= 0; --i){
			// 	pose.header.stamp = ros::Time::now();
			// 	// pose.header.frame_id = "base_link";
			// 	get_row_col(node_dim, path[i], info.width);
			// 	pose.pose.position.x = node_dim[0];
			// 	pose.pose.position.y = node_dim[1];
            //     cout << node_dim[0] << " " << node_dim[1] << endl;
			// 	// pose.pose.position.z = 0.0;
			// 	// pose.pose.orientation.x = 0.0;
			// 	// pose.pose.orientation.y = 0.0;
			// 	// pose.pose.orientation.z = 0.0;
			// 	// pose.pose.orientation.w = 1.0;
			// 	// gui_path.poses.push_back(pose);
			// 	plan.push_back(pose);
			// }
			gui_path.poses.resize(plan.size());

			if(!plan.empty()){
				gui_path.header.frame_id = "map";
				gui_path.header.stamp = ros::Time::now();
			}

			for(unsigned int i=0; i < plan.size(); i++){
				gui_path.poses[i] = plan[i];
			}
			ros::Rate rate(5);
			while(ros::ok()) {
				path_pub.publish(gui_path);
				ros::spinOnce();
				rate.sleep();
			}
		}
	private:
		ros::NodeHandle n; 
		ros::Publisher path_pub;
		ros::Subscriber map_sub;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}