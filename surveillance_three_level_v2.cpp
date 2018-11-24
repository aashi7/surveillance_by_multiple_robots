#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <queue>

#define NUM_OF_ROBOTS 2
#define NUM_OF_WAYPOINTS 2

using namespace std;

// In G_T, for each ith robot - have a cost corresponding to each robot i 
// If i matches the robot_id, update its cost else remains same

// Add cost functions now 
unordered_map<int, pair<int, int> > waypoint_id_to_grid_location; // 0 should map to start position of all robots and M + 1 their start positions 

float distance_between_two_waypoints(int wp1, int wp2)
{
	float cost = 0.0;
	pair<int, int> loc1 = waypoint_id_to_grid_location[wp1];
	pair<int, int> loc2 = waypoint_id_to_grid_location[wp2];
	cost = sqrt(pow(loc1.first - loc2.first, 2.0) + pow(loc1.second - loc2.second, 2.0)); 
	return cost; 
}

// Define a function B
vector<bool> B(vector<int>& waypoints) // sequence of waypoints - integer numbers  // B_5(2,4,5)= 11010
{ 
	int M = NUM_OF_WAYPOINTS;
	vector<bool> ans(M,0);
	for (int i = 0; i < waypoints.size(); i++)
	{
		int index = M - waypoints[i]; // 5 - 2 = 3 
		ans[index] = 1;
	}
	return ans; 
}

class G_M
{
	public:
		struct nodeM 
		{
			vector<bool> alpha;	// alpha - vector of M bits where a 0 bit indicates that the corresponding waypoint is unvisited  
			int omega;	// omega - index of the waypoint where the robot is currently at // at start = 0; at goal = M + 1 
			// order of visiting the waypoints does not matter : B_5(2,4,5) = B_5(4,2,5) 
			// Two vertices in G_M are connected iff they only differ by a single bit in alpha 
			// Edge direction is from a vertex with a lower value of alpha to that of a higher alpha (IT'S A DIRECTED GRAPH)
			float cost; 
		};

		nodeM* initialize_graph();
		nodeM* add_vertex(nodeM* parent, vector<bool>& new_alpha, int new_omega);
		vector<nodeM*> generate_successors(nodeM* parent);
		float construct_graph_upto_goal(vector<int>& allocated_waypoints);
};

G_M::nodeM* G_M::initialize_graph()
{
	int M = NUM_OF_WAYPOINTS;
	nodeM* new_node = new nodeM [1];
	vector<bool> start(M, 0);
	new_node->alpha = start; // all M waypoints are unvisited and robot is at 0
	new_node->omega = 0;
	new_node->cost = 0.0;
	return new_node; 
}

G_M::nodeM* G_M::add_vertex(nodeM* parent, vector<bool>& new_alpha, int new_omega)
{
	nodeM* new_node = new nodeM [1];
	new_node->alpha = new_alpha;
	new_node->omega = new_omega;
	new_node->cost = distance_between_two_waypoints(parent->omega, new_omega); // Euclidean distance between the two waypoints - previous (integer) and current (integer)  
	//cout << "Previous location: " << parent->omega  << " Current location: " << new_omega << " Euclidean Distance: " << new_node->cost << endl; 
	return new_node;
}

vector<G_M::nodeM*> G_M::generate_successors(nodeM* parent)
{
	vector<nodeM*> ans;
	vector<bool> prev = parent->alpha;
	int prev_loc = parent->omega;
	int M = NUM_OF_WAYPOINTS;
	for (int i = M-1; i >= 0; i--)
	{
		if (prev[i] == 0) // change in only one bit - to higher value only 
		{
			vector<bool> new_alpha = prev;
			new_alpha[i] = 1;
			// updating the new omega from (i+1) to (M-i)
			ans.push_back(add_vertex(parent, new_alpha, M-i));  // M - 1 position corresponds to Mth waypoint 
		}
	}
	// One successor is that it reaches the goal
	if (parent->omega < M+1) // only if robot is not currently at goal 
	{ 
		vector<bool> new_alpha = prev;
		nodeM* newNode = add_vertex(parent, new_alpha, M+1); // robot reaches the goal 
		ans.push_back(newNode);
	}
	return ans; 
}

void print_GM_node(G_M::nodeM* node_to_print)
{
	vector<bool> alpha = node_to_print->alpha; 
	for (int i=0; i < alpha.size(); i++)
		cout << alpha[i];
	cout << " " << node_to_print->omega << " Cost: " << node_to_print->cost <<  endl; 
	return; 
}

void print_vector(vector<int>& v)
{
	for (int i = 0; i < v.size(); i++)
		cout << v[i] << " ";
	cout << endl;  
}

void print_boolean(vector<bool>& v)
{
	for (int i = 0; i < v.size(); i++)
		cout << v[i] << " ";
	cout << endl;  
}

float G_M::construct_graph_upto_goal(vector<int>& allocated_waypoints)
{
	float optimal_cost = 0.0; 
	int M = NUM_OF_WAYPOINTS; 
	vector<bool> goal_notation = B(allocated_waypoints);
	cout << "Printing desired bool array: ";
	print_boolean(goal_notation);
	nodeM* root = initialize_graph();
	// The nodes are expanded using a queue 
	bool goal_expanded = 0;
	// Change it to priority queue storing g-value also 
	priority_queue<pair<float, nodeM*>, vector<pair<float, nodeM*> >, greater<pair<float, nodeM*> > > to_expand; 
	to_expand.push({0, root});
	float g_parent; 

	while (not goal_expanded && not to_expand.empty())
	{
		nodeM* parent = to_expand.top().second;
		// check if goal is expanded 
		g_parent = to_expand.top().first; 

		if (goal_notation == parent->alpha && parent->omega == M+1)
		{
			goal_expanded = 1;
			continue; 
		} 
		to_expand.pop();
		// should return nodes of children to put into queue and check if child is a goal 
		vector<nodeM*> successors = generate_successors(parent);
		for (int i=0; i < successors.size(); i++)
		{
			float g = g_parent + successors[i]->cost;  
			to_expand.push({g,successors[i]});
			//print_GM_node(successors[i]);
		}
	}

	if (goal_expanded)
	{
		cout << "Optimal Cost from mid level graph: " << g_parent << endl; 
		optimal_cost = g_parent; 
	}
	else
		cout << "Queue empty without finding the goal " << endl; 

	return optimal_cost; 
}

// cost for top level graph 
float search_in_GM(vector<int>& new_q, int robot_id) // New allocation of waypoints, 
{
	float cost = 0.0;
	int M = NUM_OF_WAYPOINTS; // should replace with const int M = 2 at start of code
	// from robot id, find out what waypoints have to be visited by that robot 
	// construct the goal for robot 
	vector<int> waypoints_to_visit;
	for (int i = new_q.size() - 1; i >= 0; i--)
	{
		if (new_q[i] == robot_id)
			waypoints_to_visit.push_back(M-i); 
	}
	G_M my_gm;
	//G_M::nodeM* root = my_gm.initialize_graph();
	//my_gm.generate_successors(root);
	cout << "G_M called to visit these waypoints ";
	print_vector(waypoints_to_visit);
	cost = my_gm.construct_graph_upto_goal(waypoints_to_visit);
	return cost; 
}

// Have to write ostream functions like symbolic planner to visualize the nodes

// I am changing the G_T class accordingly to keep constructing
// the graph until the goal expands 

class G_T
{
	public:

		//int N = NUM_OF_ROBOTS;

		struct node 
		{
			vector<int> q; // vector of M elements where M is the number of waypoints // The start node is always q_start={(0,0,..)}
			vector<node*> successors; 
			node* parent;
			// Changed cost to cost corresponding to each robot 
			vector<float> cost; // cost of the edge between this node and its parent   
			int robot_id; // intialization for root node  
			//edge corresponds to which robot who is assigned additional waypoint
		};

		int size = 0;
		node* initialize_graph();
		node* add_vertex(node* parent, vector<int>& new_q, int robot_id);
		vector<node*> generate_successors(node* parent);
		float construct_GT_upto_goal();
};

G_T::node* G_T::initialize_graph() // I do not want to fix the maximum number of nodes in graph 
{
	int M = NUM_OF_WAYPOINTS;
	int N = NUM_OF_ROBOTS;
	vector<int> start(M, 0);
	node* new_node = new node [1];
	new_node->q = start;
	new_node->parent = NULL;
	
	vector<float> cost(N,0.0);
	new_node->cost = cost; 
	//new_node->cost = 0.0; // should get initialized to 0 by default 
	// robot_id should get assigned as -1 
	size++; 
	return new_node; 
}

G_T::node* G_T::add_vertex(node* parent, vector<int>& new_q, int robot_id)
{
	int N = NUM_OF_ROBOTS;
	node* new_node = new node [1];
	new_node->q = new_q;
	new_node->parent = parent;
	new_node->robot_id = robot_id; // added here 
	new_node->cost = parent->cost; 
	new_node->cost[robot_id-1] = search_in_GM(new_q, robot_id);
	// for (int i = 0; i < N; i++)
	// {
	// 	if (i == robot_id-1)
	// 		new_node->cost[robot_id-1] = search_in_GM(new_q, robot_id);     // which robot was allocated   // construct a copy 
	// 	else
	// 		new_node->cost[i] = parent->cost[i];
	// }	
	parent->successors.push_back(new_node);
	return new_node; 
}

vector<G_T::node*> G_T::generate_successors(node* parent)
{
	vector<node*> ans; 
	vector<int> allocation = parent->q;
	int first_waypoint_unassigned = -1;
	for (int i = allocation.size() - 1; i >= 0; i--)
	{
		if (parent->q[i] == 0)
		{
			first_waypoint_unassigned = i;
			break;			
		}
	} 
	int N = NUM_OF_ROBOTS;
	for (int i = 1; i <= N; i++)
	{
		vector<int> new_q = parent->q; 
		new_q[first_waypoint_unassigned] = i; 
		node* newNode = add_vertex(parent, new_q, i);
		ans.push_back(newNode);
	}
	return ans;
}

void print_GT_node(G_T::node* node_to_print)
{
	int N = NUM_OF_ROBOTS;
	vector<int> q = node_to_print->q;
	for (int i = 0; i < q.size(); i++)
		cout << q[i] << " ";
	for (int i=0; i < N; i++)
	{
		cout << "Cost: " << node_to_print->cost[i] << endl;
	} 
	return; 
}

float G_T::construct_GT_upto_goal()
{
	float optimal_cost = 0.0;
	node* root = initialize_graph();
	print_GT_node(root);
	bool goal_expanded = 0;
	priority_queue<pair<float, node*>, vector<pair<float, node*> >, greater<pair<float, node*> > > to_expand; 
	// have to change the queue to priority queue for A* search  
	to_expand.push({0, root});
	float g_parent; 
	while (not goal_expanded && not to_expand.empty())
	{
		node* parent = to_expand.top().second;
		g_parent = to_expand.top().first; 
		// check if goal is expanded 
		// If all the waypoints are visited, goal reached
		vector<int> allocation = parent->q; 
		if (find(allocation.begin(), allocation.end(), 0) == allocation.end())
		{
			goal_expanded = 1;
			continue; 
		} 
		to_expand.pop();
		// should return nodes of children to put into queue and check if child is a goal 
		vector<node*> successors = generate_successors(parent);
		for (int i=0; i < successors.size(); i++)
		{
			//float g = successors[i]->cost - g_parent;
			//Which cost to select from successors[i]->cost
			//Edge corresponds to which robot - that I also have to add to node  
			int r = successors[i]->robot_id;
			float g = g_parent + (successors[i]->cost[r-1] - parent->cost[r-1]); 
			to_expand.push({g, successors[i]});
			print_GT_node(successors[i]);
		}
	}

	if (goal_expanded)
	{
		optimal_cost = g_parent; 
	}

	return optimal_cost; 
}

int main()
{
	// In what form should I give the input - text file makes sense - how to parse it then; 
	//first try to feed in an input array 
	// Location of waypoints 
	pair<int, int> start = {0,0}; // 2 Robots 
	pair<int, int> wp1 = {3,7};
	pair<int, int> wp2 = {7,7};

	int M = NUM_OF_WAYPOINTS;
	waypoint_id_to_grid_location[0] = start;
	waypoint_id_to_grid_location[1] = wp1;
	waypoint_id_to_grid_location[2] = wp2;
	waypoint_id_to_grid_location[3] = start; // goal is same as start  

	G_T my_gt; // Class constructor 
	float optimal_cost = my_gt.construct_GT_upto_goal(); 
	cout << optimal_cost << endl; 
	// G_T::node* root = my_gt.initialize_graph(); 
	// vector<G_T::node*> succ = my_gt.generate_successors(root);

	// Print the successors of G_T 
	// for (int i = 0; i < succ.size(); i++)
	// 	print_GT_node(succ[i]);

	return 0; 
}