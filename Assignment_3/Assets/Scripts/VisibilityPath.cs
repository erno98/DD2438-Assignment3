using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.Rendering;

public class VisibilityPath : MonoBehaviour
{
	public float min_x; // meters
	public float max_x; // meters
	public int x_n; // number of x grid elements
	public float x_res; // (max - min)/n
	    
	public float min_z; // meters
	public float max_z; // meters
	public int z_n; // number of z grid elements
	public float z_res; // (max - min)/n
	   
	public float[,] occupancy_grid;
	public float vertex_offset;

	public List<node> nodes;
	public List<List<int>> graph_edge_ids;
	public List<Vector3> vis_path;

	public bool path_status = false;
	private int path_goal_id = -1;

	public GameObject terrain_manager_game_object;
	TerrainManager terrain_manager;

	// Start is called before the first frame update
    void Start()
    {
		terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
	    // Map Info
	    float[,] O_map = terrain_manager.myInfo.traversability;
	    // X
	    int x_n = terrain_manager.myInfo.x_N;
	    float x_min = terrain_manager.myInfo.x_low;
	    float x_max = terrain_manager.myInfo.x_high;
	    float x_res = (x_max - x_min)/x_n;
			
	    // Z (Y)
	    int z_n = terrain_manager.myInfo.z_N;
	    float z_min = terrain_manager.myInfo.z_low;
	    float z_max = terrain_manager.myInfo.z_high;
	    float z_res = (z_max - z_min)/z_n;
			
	    // Start and goal
	    Vector3 start_pos = terrain_manager.myInfo.start_pos;
	    Vector3 goal_pos = terrain_manager.myInfo.goal_pos;
			
	    // Super sampling needed?
	    // will need to be more robust for varied map resolutions
	    int desired_res = 1;
	    int x_scale = (int)(x_res / desired_res);
	    int z_scale = (int)(z_res / desired_res);

	    float[,] new_grid = do_grid_upsample(O_map, x_scale, z_scale);

	    // Form the config grid
	    int desired_border = 2;
	    float[,] config_grid = do_grid_config(new_grid, desired_border);

	    // JVs visibilty
	    VisibilityPath vis_path_find = new VisibilityPath(config_grid, x_min, x_max, z_min, z_max);
	    vis_path_find.perform_planning(start_pos.x, start_pos.z, goal_pos.x, goal_pos.z, (float)5.0);

	    vis_path = new List<Vector3>();
	    if (vis_path_find.path_status)
	    {
		    vis_path = vis_path_find.generate_path();
	    }
	    
	    // Plot your vertices to see if it makes sense
	    Vector3 old_vt = start_pos;
	    foreach (var vt in vis_path)
	    {
		    Debug.DrawLine(old_vt, vt, Color.blue, 100f);
		    old_vt = vt;
		    Console.WriteLine(vt);
	    }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    
    public class node
	    {
		    public float x;
		    public float z;
		    public float cost;
		    public int parent;
		    public List<int> edge_ids;

		    public node(float x, float z, float cost = 0, int parent = -1)
		    {
			    this.x = x;
			    this.z = z;
			    this.cost = cost;
			    this.parent = parent;
			    this.edge_ids = new List<int>();
		    }
	    }

		// Constructor
	    public VisibilityPath(float[,] occupancy_grid, float min_x, float max_x, float min_z, float max_z)
	    {
		    // Grid
		    this.occupancy_grid = occupancy_grid;

		    // X
		    this.min_x = min_x;
		    this.max_x = max_x;
		    this.x_n = this.occupancy_grid.GetLength(0);
		    this.x_res = (max_x - min_x) / this.x_n;

		    // Z (Y)
		    this.min_z = min_z;
		    this.max_z = max_z;
		    this.z_n = this.occupancy_grid.GetLength(1);
		    this.z_res = (max_z - min_z) / this.z_n;
		    
		    // Initialize
		    this.nodes = new List<node>();
		    this.graph_edge_ids = new List<List<int>>();
	    }
		
	    // Main methods
	    public bool perform_planning(float start_x, float start_z, float goal_x, float goal_z, float offset)
	    {
		    // Performs path plannning 
		    
		    // Add the start and end points to the node list, graph
		    node start_node = new node(start_x, start_z);
		    node goal_node = new node(goal_x, goal_z);
		    this.nodes.Add(start_node);
		    this.nodes.Add(goal_node);
		    
		    
		    // find the vertices that will make up the nodes of the visibility graph
		    List<Vector3> Vertices = this.find_grid_convex_verts(offset);
			
		    // converts the list of Vector3 to nodes in a list
		    this.add_verts_to_graph(Vertices);
			
		    // Check for collisions between vertices, nodes, only valid edges are maintained
		    // the returned array contains the cost of an edge going from node_a to node_b
		    float[,] costs = this.add_edges_to_graph();
		    
		    // The above stores all edge info in a list of list, below that info is distributed to the nodes.
		    // each node has a list of valid edges sored as a list.
		    for (int node_id = 0; node_id < this.nodes.Count; node_id++)
		    {
			    this.nodes[node_id].edge_ids = this.graph_edge_ids[node_id];
		    }

		    // Dijkstra's Algo
		    // Initialize the open and closed sets
		    List<int> open_list = new List<int>();
		    List<int> closed_list = new List<int>();
		    
		    int current_id = 0;
		    int start_id = this.find_node_id(start_node);
		    int goal_id = this.find_node_id(goal_node);
		    
		    open_list.Add(start_id);

		    while (true)
		    {
			    // Check if the goal is in the closed set -> DONE!
			    if (closed_list.Exists(element => element == goal_id))
			    {
				    // At this point we are done but the below is not the correct thing to do!
				    this.path_status = true;
				    this.path_goal_id = goal_id;
				    return true;
			    }
			    else if (open_list.Count < 1)
			    {
				    // Failure!! no open nodes and goal is not yet reached
				    this.path_status = true;
				    return false;
			    }
				
			    // Expand about open node with the lowest cost
			    current_id = this.lowest_cost_node_id(open_list);

			    open_list.Remove(current_id);
			    closed_list.Add(current_id);
			    
			    // loop over the nodes that are connected to the current node
			    foreach (var visit_id in this.nodes[current_id].edge_ids)
			    {
				    // cost to visit this node
				    float new_cost = this.nodes[current_id].cost + costs[current_id, visit_id];
				   
				    // visit existing valid neighbor node
				    if (open_list.Exists(element => element == visit_id))
				    {
					    if (new_cost < this.nodes[visit_id].cost)
					    {
						    this.nodes[visit_id].cost = new_cost;
						    this.nodes[visit_id].parent = current_id;
					    }
				    }
				    // visit valid new and non closed node
				    else if (!closed_list.Exists(element => element == visit_id))
				    {
					    // Update nodes cost and parent
					    this.nodes[visit_id].cost = new_cost;
					    this.nodes[visit_id].parent = current_id;
					    
					    // Add this newly opened node to the open list
					    open_list.Add(visit_id);
				    }
			    }
		    }
	    }

	    public List<Vector3> generate_path()
	    {
		    // Returns the path as a list of Vector3s, list is formed from the goal to the start and then reversed
		    
		    List<Vector3> path = new List<Vector3>();

		    if (this.path_status)
		    {
			    path.Add(new Vector3(this.nodes[this.path_goal_id].x, 0, this.nodes[this.path_goal_id].z));
			    int parent = this.nodes[this.path_goal_id].parent;

			    while (parent != -1)
			    {
				    // Add parent to list as Vector3d
				    path.Add(new Vector3(this.nodes[parent].x, 0, this.nodes[parent].z));
				    
				    // Update to find next parent
				    parent = this.nodes[parent].parent;
			    }
		    }
		    // reverse and return list
		    path.Reverse();

		    return path;
	    }
	    
	    // Methods of constructing nodes from vertices
	    public List<Vector3> find_grid_convex_verts(float offset)
        {
	        // Using the grid, perferably the configuration grid, vertices are found. The continuous coordinates are returned in a list of vector3
	        // The offset allows for the vertices to be further displaced from the obstacles indicated by the grid, offset >= 0
	        
	        // i: x
	        // j: z <- z instead of y because of unity!
	        int i_n = this.occupancy_grid.GetLength(0);
	        int j_n = this.occupancy_grid.GetLength(1);

	        List<Vector3> vertices = new List<Vector3>();
	        
	        // loop over non outer elements of grid
	        for (int i = 1; i < i_n - 1; i++)
	        {
		        for (int j = 1; j < j_n - 1; j++)
		        {
			        if (this.occupancy_grid[i,j] == (float)1.0)
			        {
				        // There are 4 cases of vertices: UpperLeft, UpperRight, DownLeft, and DownRight
				        // Case 1: UpperLeft
				        if (   this.occupancy_grid[i - 1,j] == 0 
				            && this.occupancy_grid[i + 1,j] == 1
				            && this.occupancy_grid[i,j - 1] == 1
				            && this.occupancy_grid[i,j + 1] == 0 )
				        {
					        Vector3 vert = new Vector3(this.min_x + (i * this.x_res) - offset, 0, this.min_z + (j + 1) * this.z_res + offset);
					        vertices.Add(vert);
				        }
				        // Case 2: UpperRight
				        else if (   this.occupancy_grid[i - 1,j] == 1 
				                 && this.occupancy_grid[i + 1,j] == 0
				                 && this.occupancy_grid[i,j - 1] == 1
				                 && this.occupancy_grid[i,j + 1] == 0 )
				        {
					        Vector3 vert = new Vector3(this.min_x + (i + 1) * this.x_res + offset, 0, this.min_z + (j + 1) * this.z_res + offset);
					        vertices.Add(vert);
				        }
				        // Case 3: DownLeft
				        else if (   this.occupancy_grid[i - 1,j] == 0 
				                    && this.occupancy_grid[i + 1,j] == 1
				                    && this.occupancy_grid[i,j - 1] == 0
				                    && this.occupancy_grid[i,j + 1] == 1 )
				        {
					        Vector3 vert = new Vector3(this.min_x + (i * this.x_res) - offset, 0, this.min_z + (j * this.z_res) - offset);
					        vertices.Add(vert);
				        }
				        // Case 4: DownRight
				        else if (   this.occupancy_grid[i - 1,j] == 1 
				                    && this.occupancy_grid[i + 1,j] == 0
				                    && this.occupancy_grid[i,j - 1] == 0
				                    && this.occupancy_grid[i,j + 1] == 1 )
				        {
					        Vector3 vert = new Vector3(this.min_x + (i + 1) * this.x_res + offset, 0, this.min_z + (j * this.z_res) - offset);
					        vertices.Add(vert);
				        }
			        }
		        }
	        }

	        return vertices;
        }
	    
	    public void add_verts_to_graph(List<Vector3> convex_verts)
	    {
		    // Adds vertices stored in a list of vector3 to the list of nodes
		    foreach (var vec_3 in convex_verts)
		    {
			    this.nodes.Add(new node(vec_3.x, vec_3.z));
		    }
	    }
		
	    // Methods for checking and adding edges between nodes
	    public float[,] add_edges_to_graph()
	    {
		    // loop over all nodes and look for valid edges, if edges are valid
		    // this will also return NxN array with the edge costs

		    float[,] costs = new float[this.nodes.Count, this.nodes.Count];
		    
		    for (int node_id = 0; node_id < this.nodes.Count; node_id++)
		    {
			    List<int> new_edge_ids = new List<int>();
			    for (int target_node_id = 0; target_node_id < this.nodes.Count; target_node_id++)
			    {
				    if (node_id != target_node_id) // only add edges bewteen different nodes
				    {
					    if (this.Check_grid_bewteen_nodes(node_id, target_node_id)) // checks if path is clear of obstacles
					    {
						    // Add node id to list
						    new_edge_ids.Add(target_node_id);
						    
						    // cost is just euclidean distance
						    float cost = (float) Math.Sqrt(Math.Pow(this.nodes[node_id].x - this.nodes[target_node_id].x, 2.0f) +
															Math.Pow(this.nodes[node_id].z - this.nodes[target_node_id].z, 2.0f));
						    
						    // Add this cost to the NxN costs array
						    // This array should be symetric
						    costs[node_id, target_node_id] = cost;
					    }
				    }
			    } 
			    // After looping over other nodes add the ids of valid ones to the graph_edge_ids list
			    this.graph_edge_ids.Add(new_edge_ids);
		    }

		    return costs;

	    }
	    
	    public List<int> Convert_coords_to_indices(float x, float z)
	    {
		    // Converts the continuous coords to indices, used for checking collisions on the config grid
		    List<int> indices = new List<int>();
		    
		    // x -> i_index
		    int i_index = (int) Math.Floor((x - this.min_x) / this.x_res);
		    indices.Add(i_index);
		    
		    // z -> j_index
		    int j_index = (int) Math.Floor((z - this.min_z) / this.z_res);
		    indices.Add(j_index);

		    return indices;
	    }

	    public bool Check_grid_between_indices(int x0, int z0, int x1, int z1)
	    {
		    // Used to check for collision between to grid positions on the configuration grid
		    
		    // There is some problems with this approach, when the line passes through a grid point it will move vertically instead of both veritcally and horizontally
		    // For example line going from cell C -> D will return A, B, and C as visited. Ideally I think all cells should count as visited to be safe but maybe not...
		    // A B
		    // C D
	        
		    int i_n = this.occupancy_grid.GetLength(0);
		    int j_n = this.occupancy_grid.GetLength(1);

		    // Check if input is valid
		    if (x0 < 0 || z0 < 0 || x1 < 0 || z1 < 0)
		    {
			    return false;
		    }

		    if (x0 >= i_n || x1 >= i_n || z0 >= j_n || z1 >= j_n)
		    {
			    return false;
		    }

		    // grid raytracing from: http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
		    int dx = Math.Abs(x1 - x0);
		    int dz = Math.Abs(z1 - z0);
		    int x = x0;
		    int z = z0;
		    int n = 1 + dx + dz;
		    int x_inc = (x1 > x0) ? 1 : -1;
		    int z_inc = (z1 > z0) ? 1 : -1;
		    int error = dx - dz;
		    dx *= 2;
		    dz *= 2;

		    for (; n > 0; --n)
		    {

			    // Check
			    if (this.occupancy_grid[x, z] == 1)
			    {
				    return false;
			    }

			    if (error > 0)
			    {
				    x += x_inc;
				    error -= dz;
			    }
			    else
			    {
				    z += z_inc;
				    error += dx;
			    }
		    }

		    return true;
	    }

	    public bool Check_grid_bewteen_nodes(int node_a_id, int node_b_id)
	    {
		    if (node_a_id == node_b_id)
		    {
			    return false;
		    }

		    List<int> a_indices = this.Convert_coords_to_indices(this.nodes[node_a_id].x, this.nodes[node_a_id].z);
		    List<int> b_indices = this.Convert_coords_to_indices(this.nodes[node_b_id].x, this.nodes[node_b_id].z);

		    return this.Check_grid_between_indices(a_indices[0], a_indices[1], b_indices[0], b_indices[1]);
	    }
	    
	    // Working with nodes
	    public int find_node_id(node node_a)
	    {
		    float threshold = 0.5f;
		    for (int i = 0; i < this.nodes.Count; i++)
		    {
			    if (this.node_dist(node_a, this.nodes[i]) <= threshold)
			    {
				    return i;
			    }
		    }
		    // This is a problem
		    return -1;
	    }

	    public float node_dist(node node_a, node node_b)
	    {
		    float distance = (float) Math.Sqrt(Math.Pow(node_a.x - node_b.x, 2.0f) +
		                                   Math.Pow(node_a.z - node_b.z, 2.0f));

		    return distance;
	    }

	    public int lowest_cost_node_id(List<int> id_list)
	    {
		    // id_list is list of node indices
		    
		    float min_val = -1;
		    int min_id = 0;

		    foreach (var node_id in id_list)
		    {
			    if (min_val == -1 || this.nodes[node_id].cost < min_val)
			    {
				    min_val = this.nodes[node_id].cost;
				    min_id = node_id;
			    }
		    }

		    return min_id;
	    }
    
		float[,] do_grid_upsample(float[,] original_grid, int i_scale, int j_scale)
		{
		    // upsample grid
		    // i: x
		    // j: z, or y
		    int i_n = original_grid.GetLength(0);
		    int j_n = original_grid.GetLength(1);
		    
		    float[,] new_grid = new float[i_n * i_scale, j_n * j_scale];
		    
		    for (int i = 0; i < i_n; i++)
		    {
		        for (int j = 0; j < j_n; j++)
		        {
		            float val = original_grid[i, j];
		            for (int iMod = 0; iMod < i_scale; iMod++)
		            {
			            for (int jMod = 0; jMod < j_scale; jMod++)
			            {
				            new_grid[i * i_scale + iMod, j * j_scale + jMod] = val;
			            }
		            }
		        }
		    }

		    return new_grid;
		}

		float[,] do_grid_config(float[,] original_grid, int border)
		{
		    // add a border arround fill grid
		    // i: x
		    // j: z, or y
		    int i_n = original_grid.GetLength(0);
		    int j_n = original_grid.GetLength(1);
		    
		    float[,] new_grid = new float[i_n, j_n];
		    
		    for (int i = 0; i < i_n; i++)
		    {
		        for (int j = 0; j < j_n; j++)
		        {
			        if (original_grid[i,j] == 1.0)
			        {
				        for (int iMod = i - border; iMod < i + border + 1; iMod++)
				        {
					        if (iMod >= 0 && iMod < i_n) // VALID i index
					        {
						        for (int jMod = j - border; jMod < j + border + 1; jMod++)
						        {
							        if (jMod >= 0 && jMod < j_n) // VALID j index
							        {
								        new_grid[iMod, jMod] = (float)1.0;
							        }
						        }  
					        }
				        }
			        }
		        }
		    }

		    return new_grid;
		}

		List<Vector3> find_grid_convex_verts(float[,] original_grid, float x_res, float z_res, float offset)
		{
		    // i: x
		    // j: z
		    int i_n = original_grid.GetLength(0);
		    int j_n = original_grid.GetLength(1);

		    List<Vector3> vertices = new List<Vector3>();
		    
		    // loop over non outer elements of grid
		    for (int i = 1; i < i_n - 1; i++)
		    {
		        for (int j = 1; j < j_n - 1; j++)
		        {
			        if (original_grid[i,j] == (float)1.0)
			        {
				        // There are 4 cases of vertices: UpperLeft, UpperRight, DownLeft, and DownRight
				        // Case 1: UpperLeft
				        if (   original_grid[i - 1,j] == 0 
				            && original_grid[i + 1,j] == 1
				            && original_grid[i,j - 1] == 1
				            && original_grid[i,j + 1] == 0 )
				        {
					        Vector3 vert = new Vector3(i * x_res - offset, 0, (j + 1) * z_res + offset);
					        vertices.Add(vert);
				        }
				        // Case 2: UpperRight
				        else if (   original_grid[i - 1,j] == 1 
				                 && original_grid[i + 1,j] == 0
				                 && original_grid[i,j - 1] == 1
				                 && original_grid[i,j + 1] == 0 )
				        {
					        Vector3 vert = new Vector3((i + 1) * x_res + offset, 0, (j + 1) * z_res + offset);
					        vertices.Add(vert);
				        }
				        // Case 3: DownLeft
				        else if (   original_grid[i - 1,j] == 0 
				                    && original_grid[i + 1,j] == 1
				                    && original_grid[i,j - 1] == 0
				                    && original_grid[i,j + 1] == 1 )
				        {
					        Vector3 vert = new Vector3(i * x_res - offset, 0, j * z_res - offset);
					        vertices.Add(vert);
				        }
				        // Case 4: DownRight
				        else if (   original_grid[i - 1,j] == 1 
				                    && original_grid[i + 1,j] == 0
				                    && original_grid[i,j - 1] == 0
				                    && original_grid[i,j + 1] == 1 )
				        {
					        Vector3 vert = new Vector3((i + 1) * x_res + offset, 0, j * z_res - offset);
					        vertices.Add(vert);
				        }
			        }
		        }
		    }

		    return vertices;
		}

		bool Check_grid(float[,] grid, int x0, int z0, int x1, int z1)
		{
		    // There is some problems with this approach, when the line passes through a grid point it will move vertically instead of both veritcally and horizontally
		    // For example line going from cell C -> D will return A, B, and C as visited. Ideally I think all cells should count as visited to be safe but maybe not...
		    // A B
		    // C D
		    
		    int i_n = grid.GetLength(0);
		    int j_n = grid.GetLength(1);

		    // Check if input is valid
		    if (x0 < 0 || z0 < 0 || x1 < 0 || z1 < 0)
		    {
		        return false;
		    }

		    if (x0 >= i_n || x1 >= i_n || z0 >= j_n || z1 >= j_n)
		    {
		        return false;
		    }

		    // grid raytracing from: http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
		    int dx = Math.Abs(x1 - x0);
		    int dz = Math.Abs(z1 - z0);
		    int x = x0;
		    int z = z0;
		    int n = 1 + dx + dz;
		    int x_inc = (x1 > x0) ? 1 : -1;
		    int z_inc = (z1 > z0) ? 1 : -1;
		    int error = dx - dz;
		    dx *= 2;
		    dz *= 2;

		    for (; n > 0; --n)
		    {

		        // Check
		        if (grid[x, z] == 1)
		        {
			        return false;
		        }

		        if (error > 0)
		        {
			        x += x_inc;
			        error -= dz;
		        }
		        else
		        {
			        z += z_inc;
			        error += dx;
		        }
		    }

		    return true;
		}
}

