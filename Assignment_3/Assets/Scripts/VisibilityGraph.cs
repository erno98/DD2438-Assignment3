using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using UnityEngine;
using PathCreation;
using UnityEngine.Analytics;
using Vector3 = UnityEngine.Vector3;

public class VisibilityGraph : MonoBehaviour
{
	// Ignore initial Orientation - Use for drone
	// todo: not sure how to pass this value to visibilityGraph
	public bool ignoreStartOrientation = false;
	
	// Visualization controls
    public bool showGraph = true;
    public bool showPath = true;
    public bool showNodeOrientations = true;
    public bool showNodeWeights = true;
    public bool showBezierPath = true;
    
    // Configuration space offset
    public float offset = 0.2f;
    
    // config space padding
    public float desired_res = .25f;
    public float x_res_config = 0; // (max - min)/n_config
    public float z_res_config = 0; // (max - min)/n_config
    public int config_padding = 3;
    
    // Terrain
    public GameObject terrain;
    TerrainManager terrainManager;
    
    // Path
    public PathCreator PathCreator;
    public Path path;
    public float vertexSpacing = .1f;
    public bool individualControlPointOptimization = false;
    public bool asymmetricIndividualControlPointOptimization = false;
    public float maxControlPointSpacing = 0.4f;
    public int kCentralMovingAverage = 2;

    // Map related
    // X, this size is fixed for both occupancy and config
    float min_x; // meters
    float max_x; // meters
			
    int x_n; // number of x grid elements, ORIGINAL OCCUPANCY
    float x_res; // (max - min)/n
    int x_n_config; // number of x grid elements, CONFIG

    // Z, this size is fixed for both occupancy and config	
    float min_z; // meters
    float max_z; // meters
			
    int z_n; // number of z grid elements
    float z_res; // (max - min)/n
    int z_n_config; // number of x grid elements, CONFIG

    float[,] occupancy_grid;
    float[,] config_grid;

    public Vector3 startPos;
    public Vector3 goalPos;

    public node StartNode;
    public node GoalNode;

    public float[,] EdgeCosts;
    
    public List<node> nodes = new List<node>();
    List<List<int>> graph_edge_ids = new List<List<int>>();

    // Start is called before the first frame update
    void Start ()
    {
	    terrainManager = terrain.GetComponent<TerrainManager>();
	    
	    // Map Info
	    occupancy_grid = terrainManager.myInfo.traversability;
			    
	    // X
	    min_x = terrainManager.myInfo.x_low; // todo: was x_min
	    max_x = terrainManager.myInfo.x_high; // todo: was x_max
	    x_n = terrainManager.myInfo.x_N;
	    x_res = (max_x - min_x)/x_n;
					
	    // Z (Y)
	    min_z = terrainManager.myInfo.z_low; // todo: was z_min
	    max_z = terrainManager.myInfo.z_high; // todo: was z_max
	    z_n = terrainManager.myInfo.z_N;
	    z_res = (max_z - min_z)/z_n;

	    // Upsample
	    // Select scaling factor
	    int x_scale = (int)Mathf.Max((x_res / desired_res), 1);
	    int z_scale = (int)Mathf.Max((z_res / desired_res), 1);
				
	    // Perform upsampling
	    float[,] new_grid = do_grid_upsample(occupancy_grid, x_scale, z_scale);
			    
	    // Update the size of the upsampled config grid
	    x_n_config = x_n * x_scale;
	    z_n_config = z_n * z_scale;
			    
	    // Update the resolution of the config grid
	    x_res_config = (max_x - min_x) / x_n_config;
	    z_res_config = (max_z - min_z) / z_n_config;

	    // Form the config grid <- Same size as new_grid above
	    config_grid = do_grid_config(new_grid, config_padding);

	    // Start and goal
	    startPos = terrainManager.myInfo.start_pos;
	    goalPos = terrainManager.myInfo.goal_pos;
	    
	    EdgeCosts = CreateVisibilityGraph();

	    path = new Path(this);

	    // Visualization
	    if (showGraph)
	    {
		    foreach (var node in nodes)
		    {
			    Vector3 current_pos = new Vector3(node.x, 0, node.z);
			    // Visit the connecting nodes
			    foreach (var visit_id in node.edge_ids)
			    {
				    Vector3 visit_pos = new Vector3(nodes[visit_id].x, 0, nodes[visit_id].z);
				    Debug.DrawLine(current_pos, visit_pos, Color.yellow, 100f);
			    }
		    }
	    }
	    if (showPath)
	    {
		    if (path.path_status)
		    {
			    Vector3 old_vt =  new Vector3(nodes[0].x, 0, nodes[0].z);
			    foreach (var vt in path.path)
			    {
				    for (int i = 0; i < 3; i++)
				    {
					    var offset = 0.1f * i;
					    Debug.DrawLine(old_vt + new Vector3(offset, 0f, offset), 
						    vt + new Vector3(offset, 0f, offset), Color.blue, 100f);
							    
				    }
				    old_vt = vt;
				    Console.WriteLine(vt);
			    }
		    }
	    }
	    if (showNodeOrientations)
	    {
		    int orient_scale = 10;
		    foreach (var node in nodes)
		    {
			    Vector3 start_vt = new Vector3(node.x, 0, node.z);
			    // Vector of orientation after travel
			    Vector3 travel_end_vt = start_vt + new Vector3((float)(orient_scale * Math.Cos(node.orientation)), 
				    0, 
				    (float)(orient_scale * Math.Sin(node.orientation)));
			            
			    // vector of parents orientation of parent
			    Vector3 parent_end_vt = start_vt + new Vector3((float)(orient_scale * Math.Cos(node.parent_orientation)), 
				    0, 
				    (float)(orient_scale * Math.Sin(node.parent_orientation)));
			            
			    // draw orientation after travel
			    Debug.DrawLine(start_vt, travel_end_vt, Color.red, 100f);
			            
			    // Draw parents orientation, skip root/start
			    if (node.parent != -1)
			    {
				    Debug.DrawLine(start_vt, parent_end_vt, Color.magenta, 100f);
			    }
		    }
	    }
	    if (showNodeWeights)
	    {
		    foreach (var node in nodes)
		    {
			    Vector3 start_vt = new Vector3(node.x, 0, node.z);
			    Vector3 end_vt = new Vector3(node.x, node.cost, node.z);
			    Debug.DrawLine(start_vt, end_vt, Color.red, 100f);    
		    }
	    }
	    if (showBezierPath)
	    {
		    Vector3 old_vt =  new Vector3(path.VectorizedBezierPath[0].x, 0, path.VectorizedBezierPath[0].z);
		    foreach (var vt in path.VectorizedBezierPath)
		    {
			    for (int i = -1; i < 2; i++)
			    {
				    var offset = 0.05f * i;
				    Debug.DrawLine(old_vt + new Vector3(offset, 0f, offset), 
					    vt + new Vector3(offset, 0f, offset), Color.red, 100f);
					    
			    }
			    old_vt = vt;
			    Console.WriteLine(vt);
		    }
	    }
    }

    // Update is called once per frame
    void Update()
    {
        
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

    float[,] CreateVisibilityGraph()
    {
	    // Performs path plannning 
	    // Find x and z of start and goal vector3
	    float start_x = startPos.x;
	    float start_z = startPos.z;

	    // The goal position needs to be checked
	    float goal_x;
	    float goal_z;
	    
	    // It is possible for the goal to be withing a valid config space
	    // todo finish checking if goal is valid if not modify
	    int horizontal_offset = 0;
	    int vertical_offset = 0;
	    float spacing = 2.50f; // Ideall this would be as small as possible

	    var goal_indices = this.Convert_coords_to_indices(goalPos.x, goalPos.z);
	    if (this.config_grid[goal_indices[0], goal_indices[1]] == 0)
	    {
		    // Goal is not 'covered' by the config mask
		    goal_x = goalPos.x;
		    goal_z = goalPos.z;
	    }
	    else
	    {
		    // checks in both horizontal and vertical
		    // can offset in both is goal is near corner
		    // check horizontal
		    for (int i = -1; i < 2; i+=2)
		    {
			    var new_goal_inds = this.Convert_coords_to_indices(goalPos.x + i * spacing, goalPos.z);
			    if (this.config_grid[new_goal_inds[0], new_goal_inds[1]] == 0)
			    {
				    horizontal_offset += i;
			    }
		    }
		    // check vertical
		    for (int j = -1; j < 2; j+=2)
		    {
			    var new_goal_inds = this.Convert_coords_to_indices(goalPos.x, goalPos.z + j * spacing);
			    if (this.config_grid[new_goal_inds[0], new_goal_inds[1]] == 0)
			    {
				    vertical_offset += j;
			    }
		    }
		    // Apply offsets
		    goal_x = goalPos.x + (horizontal_offset * spacing);
		    goal_z = goalPos.z + (vertical_offset * spacing);
	    }

	    // Add the start and end points to the node list, graph
	    StartNode = new node(start_x, start_z, 0f, -1, (float)Math.PI/2);
	    GoalNode = new node(goal_x, goal_z);
	    this.nodes.Add(StartNode);
	    this.nodes.Add(GoalNode);
				    
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

	    return costs;
    }
    
    // Methods of constructing nodes from vertices
	public List<Vector3> find_grid_convex_verts(float offset)
    {
        // Using the grid, perferably the configuration grid, vertices are found. The continuous coordinates are returned in a list of vector3
        // The offset allows for the vertices to be further displaced from the obstacles indicated by the grid, offset >= 0
        
        // i: x
        // j: z <- z instead of y because of unity!
        int i_n = this.config_grid.GetLength(0);
        int j_n = this.config_grid.GetLength(1);

        List<Vector3> vertices = new List<Vector3>();
        
        // loop over non outer elements of grid
        for (int i = 1; i < i_n - 1; i++)
        {
	        for (int j = 1; j < j_n - 1; j++)
	        {
		        if (this.config_grid[i,j] == (float)1.0)
		        {
			        // There are 4 cases of vertices: UpperLeft, UpperRight, DownLeft, and DownRight
			        // Case 1: UpperLeft
			        if (   this.config_grid[i - 1,j] == 0 
			            && this.config_grid[i + 1,j] == 1
			            && this.config_grid[i,j - 1] == 1
			            && this.config_grid[i,j + 1] == 0 )
			        {
				        Vector3 vert = new Vector3(this.min_x + (i * this.x_res_config) - offset, 0, this.min_z + (j + 1) * this.z_res_config + offset);
				        vertices.Add(vert);
			        }
			        // Case 2: UpperRight
			        else if (   this.config_grid[i - 1,j] == 1 
			                 && this.config_grid[i + 1,j] == 0
			                 && this.config_grid[i,j - 1] == 1
			                 && this.config_grid[i,j + 1] == 0 )
			        {
				        Vector3 vert = new Vector3(this.min_x + (i + 1) * this.x_res_config + offset, 0, this.min_z + (j + 1) * this.z_res_config + offset);
				        vertices.Add(vert);
			        }
			        // Case 3: DownLeft
			        else if (   this.config_grid[i - 1,j] == 0 
			                    && this.config_grid[i + 1,j] == 1
			                    && this.config_grid[i,j - 1] == 0
			                    && this.config_grid[i,j + 1] == 1 )
			        {
				        Vector3 vert = new Vector3(this.min_x + (i * this.x_res_config) - offset, 0, this.min_z + (j * this.z_res_config) - offset);
				        vertices.Add(vert);
			        }
			        // Case 4: DownRight
			        else if (   this.config_grid[i - 1,j] == 1 
			                    && this.config_grid[i + 1,j] == 0
			                    && this.config_grid[i,j - 1] == 0
			                    && this.config_grid[i,j + 1] == 1 )
			        {
				        Vector3 vert = new Vector3(this.min_x + (i + 1) * this.x_res_config + offset, 0, this.min_z + (j * this.z_res_config) - offset);
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
	
	public List<int> Convert_coords_to_indices(float x, float z)
    {
	    // Converts the continuous coords to indices, used for checking collisions on the config grid
	    List<int> indices = new List<int>();
	    
	    // x -> i_index
	    int i_index = (int) Math.Floor((x - this.min_x) / this.x_res_config);
	    indices.Add(i_index);
	    
	    // z -> j_index
	    int j_index = (int) Math.Floor((z - this.min_z) / this.z_res_config);
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
        
	    int i_n = this.config_grid.GetLength(0);
	    int j_n = this.config_grid.GetLength(1);

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
		    if (this.config_grid[x, z] == 1)
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

	public class node
    {
	    public float x;
	    public float z;
	    public int parent;
	    public float cost;
	    public float orientation;
	    public float parent_orientation;
	    public float child_orientation;
	    public List<int> edge_ids;

	    public node(float x, float z, float cost = 0, int parent = -1, float orientation = 0f, float p_orientation = 0f, float c_orientation = 0f)
	    {
		    this.x = x;
		    this.z = z;
		    this.cost = cost;
		    this.parent = parent;
		    this.orientation = orientation;
		    this.parent_orientation = p_orientation;
		    this.child_orientation = c_orientation;
		    edge_ids = new List<int>();
	    }
    }
    
    public class Path
	{
		// Visibility graph
	    private VisibilityGraph visGraph;
	    
	    // Path related
	    public List<Vector3> path = new List<Vector3>();
	    public List<int> path_ids = new List<int>();

	    public bool path_status = false;
	    private int path_goal_id = -1;
	    
	    // controls for angular coefficient
	    [Range(0, 1000)] public float angCoefficient = 50f;

	    // Bezier curve
	    public List<Vector3> VectorizedBezierPath;
	    private List<List<float>> controlPointLengths;
	    
	    // Curvature for every node in vectorized bezier curve
	    public float[] Curvatures;
	    
		public Path(VisibilityGraph visGraph)
		{
			this.visGraph = visGraph;
			FindPath();
	        generate_path();
	        //controlPointLengths = Enumerable.Repeat(Enumerable.Repeat(.01f, 2).ToList(), path.Count()).ToList();
	        //controlPointLengths = Enumerable.Range(0, path.Count()).Select(l => new List<float>()).ToList();
	        controlPointLengths = Enumerable.Repeat(0.1f, path.Count()).ToList().Select(l => Enumerable.Repeat(0.01f, 2).ToList()).ToList();
	        VectorizedBezierPath = GenerateBezierPath();
	        var curvatures = CalculateCurvatures();
	        Curvatures = CalculateAverageCurvatures(curvatures);
		}

	    bool FindPath()
	    {
	        // Dijkstra's Algo
		    // Initialize the open and closed sets
		    List<int> open_list = new List<int>();
		    List<int> closed_list = new List<int>();
		    
		    int current_id = 0;
		    int start_id = find_node_id(visGraph.StartNode);
		    int goal_id = find_node_id(visGraph.GoalNode);
		    
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
				    this.path_status = false;
				    return false;
			    }
				
			    // Expand about open node with the lowest cost
			    current_id = lowest_cost_node_id(open_list);

			    open_list.Remove(current_id);
			    closed_list.Add(current_id);
			    
			    // loop over the nodes that are connected to the current node
			    foreach (var visit_id in visGraph.nodes[current_id].edge_ids)
			    {
				    // cost to visit this node
				    // new_cost = dist_cost + angular_coefficient * ang_cost + prev_cost
				    float dist_cost = visGraph.EdgeCosts[current_id, visit_id];
				    float prev_cost = visGraph.nodes[current_id].cost; //+ visGraph.EdgeCosts[current_id, visit_id];
				    
				    // Angular portion of the cost function
				    float new_angle = calc_node_orientation(visit_id, current_id);
				    float angle_between = Math.Abs(this.angle_between(visit_id, current_id));
				    // todo Think about if this makes sense, angle should be between 0-pi
				    float ang_cost = (float) Math.Pow(angle_between/Math.PI, 5);
					float new_cost;
					
					// Threshold method
				    /*if (angle_between > Math.PI*(5/8))
				    {
					    new_cost = dist_cost * this.angular_coefficient + prev_cost;
				    }
					else
					{
						new_cost = dist_cost + prev_cost;
					}*/
					
				    // Additive version
				    if (current_id == start_id && visGraph.ignoreStartOrientation)
				    {
					    new_cost = dist_cost + prev_cost;
				    }
				    else
				    {
					    new_cost = dist_cost + angCoefficient * ang_cost + prev_cost;
				    }

				    // visit existing valid neighbor node
				    if (open_list.Exists(element => element == visit_id))
				    {
					    if (new_cost < visGraph.nodes[visit_id].cost)
					    {
						    // Update nodes cost, parent, orientation of an already visited node
						    visGraph.nodes[visit_id].cost = new_cost;
						    visGraph.nodes[visit_id].parent = current_id;
						    visGraph.nodes[visit_id].orientation = new_angle;
						    visGraph.nodes[visit_id].parent_orientation = visGraph.nodes[current_id].orientation;

					    }
				    }
				    // visit valid new and non closed node
				    else if (!closed_list.Exists(element => element == visit_id))
				    {
					    // Update visGraph.nodes cost, parent, orientation of a newly visited node
					    visGraph.nodes[visit_id].cost = new_cost;
					    visGraph.nodes[visit_id].parent = current_id;
					    visGraph.nodes[visit_id].orientation = new_angle;
					    visGraph.nodes[visit_id].parent_orientation = visGraph.nodes[current_id].orientation;
					    
					    // Add this newly opened node to the open list
					    open_list.Add(visit_id);
				    }
			    }
		    }
	    }
	    
	    // Working with nodes
	    public int find_node_id(node node_a)
	    {
		    // find_node_id
		    // return the index of the input node as it is found in this.nodes
				    
		    float threshold = 0.5f;
		    for (int i = 0; i < visGraph.nodes.Count; i++)
		    {
			    if (node_dist(node_a, visGraph.nodes[i]) <= threshold)
			    {
				    return i;
			    }
		    }
		    // This is a problem
		    return -1;
	    }
	
	    public int node_id_from_coord(Vector3 input_vect)
	    {
		    float threshold = 0.5f;
		    for(int i = 0; i < visGraph.nodes.Count; i++)
		    {
			    var node = visGraph.nodes[i];
			    var node_vect = new Vector3(node.x, 0, node.z);
			    var dist = (input_vect - node_vect).magnitude;
			    if (dist < threshold)
			    {
				    return i;
			    }
		    }
		    return -1;
	    }
	    
	    public float node_dist(node node_a, node node_b)
	    {
		    // node_dist
		    // returns the distance between nodes
				    
		    // Calculate euclidean distance
		    float distance = (float) Math.Sqrt(Math.Pow(node_a.x - node_b.x, 2.0f) +
		                                       Math.Pow(node_a.z - node_b.z, 2.0f));

		    return distance;
	    }
	    
	    public int lowest_cost_node_id(List<int> id_list)
	    {
		    // lowest_cost_node_id
		    // The input is a list of indices of the nodes found in this.nodes
		    // The function returns the element of the input list that is the indes of the node with the lowest cost
				    
		    float min_val = -1;
		    int min_id = 0;

		    foreach (var node_id in id_list)
		    {
			    if (min_val == -1 || visGraph.nodes[node_id].cost < min_val)
			    {
				    min_val = visGraph.nodes[node_id].cost;
				    min_id = node_id;
			    }
		    }

		    return min_id;
	    }
	    
	    // Orientation Stuff
	    public float calc_node_orientation(int child_node_id, int parent_node_id)
	    {
		    //calc_node_orientation
		    // Calculate the orientation of line passing through parent to child
		    // Angle given in radian with respect - x axis ->
				
		    // Extract nodes from IDs
		    node child_node = visGraph.nodes[child_node_id];
		    node parent_node = visGraph.nodes[parent_node_id];
				
		    // Use node coords to find theta
		    // Atan2(delta_y, delta_x) in our case dalta_y = delta_z
		    float theta = (float)Math.Atan2((child_node.z - parent_node.z), (child_node.x - parent_node.x));

		    return theta;
	    }
	    
	    public float angle_between(int child_node_id, int parent_node_id)
	    {
		    // angle_between
		    // The angle of the traversal from parent -> child with respect to x-axis
		    // output: -pi <= theta < pi

		    // Extract nodes from IDs
		    node child_node = visGraph.nodes[child_node_id];
		    node parent_node = visGraph.nodes[parent_node_id];
				
		    // Use node coords to find theta
		    // Atan2(delta_y, delta_x) in our case dalta_y = delta_z
		    float theta = (float)Math.Atan2((child_node.z - parent_node.z), (child_node.x - parent_node.x));
				
		    // Angle between parent sorientation and new shild
		    // This angle curresponds to the amount of turn required
		    //float theta_delta = theta - parent_node.orientation;
				
		    // normalize the difference of orientations to -pi - pi
		    //theta_delta = (float)((theta_delta + Math.PI)%(2 * Math.PI) - Math.PI);
				
		    // USING UNITY
		    // finds the smallest angle between current and target
		    float theta_delta_2 =
			    Mathf.Deg2Rad * Mathf.DeltaAngle(Mathf.Rad2Deg * parent_node.orientation, 
				    Mathf.Rad2Deg * theta);
				
		    return theta_delta_2;
	    }

	    private void generate_path()
	    {
		    // Returns the path as a list of Vector3s, list is formed from the goal to the start and then reversed
				    
		    if (path_status)
		    {
			    // Start to populate path lists
			    path.Add(new Vector3(visGraph.nodes[this.path_goal_id].x, 0, visGraph.nodes[this.path_goal_id].z));
			    path_ids.Add(path_goal_id);
			    
			    int child = this.path_goal_id;
			    int parent = visGraph.nodes[this.path_goal_id].parent;

			    while (parent != -1)
			    {
				    // Update nodes to include their path child's orientation
				    visGraph.nodes[parent].child_orientation = visGraph.nodes[child].orientation;

				    // Add parent to list as path lists
				    path.Add(new Vector3(visGraph.nodes[parent].x, 0, visGraph.nodes[parent].z));
				    path_ids.Add(parent);
				    
				    // Update to find next parent
				    child = parent;
				    parent = visGraph.nodes[parent].parent;
			    }
					    
			    // reverse as path is form from the goal
			    path.Reverse();
			    path_ids.Reverse();
		    }
		    else
		    {
			    // this should be start and goal < this case might not be needed!!
			    path.Add(new Vector3(visGraph.StartNode.x, 0, visGraph.StartNode.z));
			    path.Add(new Vector3(visGraph.GoalNode.x, 0 ,visGraph.GoalNode.z));
		    }
				   
	    }

	    private List<Vector3> GenerateBezierPath()
	    {
		    BezierPath bezierPath = new BezierPath(path, false, PathSpace.xz);
		    bezierPath.ControlPointMode = BezierPath.ControlMode.Automatic;
		    bezierPath.ControlLengths = OptimizeControlPoints(bezierPath);
		    return new VertexPath(bezierPath, visGraph.PathCreator.transform, visGraph.vertexSpacing).localPoints.ToList();
	    }
	    
	    private List<List<float>> OptimizeControlPoints(BezierPath bezierPath)
	    {
		    const float start = 0f;
		    const float smallestStep = 0.01f;
		    const float stepUpdate = 0.5f;
		    var nSearch = 0;
		    
		    if (visGraph.individualControlPointOptimization)
		    {
			    for (var i = 0; i < path.Count; i++)
			    {
				    (controlPointLengths[i], nSearch) = OptimizeControlPointSpacing(bezierPath, i, start, smallestStep,
					    stepUpdate, visGraph.maxControlPointSpacing, nSearch);
				    Debug.LogFormat("Spacing anchor # {0}: {1}", i, controlPointLengths[i][0]);
			    }
		    }
		    else if (visGraph.asymmetricIndividualControlPointOptimization)
		    {
			    for (int i = 0; i < path.Count; i++)
			    {
				    for (int j = 0; j < 2; j++)
				    {
					    (controlPointLengths[i][j], nSearch) = OptimizeAsymmetricControlPointSpacing(bezierPath, i, start, smallestStep,
						    stepUpdate, visGraph.maxControlPointSpacing, nSearch, j);
				    }
				    Debug.LogFormat("Anchor {0}: left:{1}, right: {2}", i, controlPointLengths[i][0], controlPointLengths[i][1]);
			    }
		    }
		    else
		    {
			    (controlPointLengths, nSearch) = OptimizeAllControlPoints(bezierPath, start, smallestStep, stepUpdate, visGraph.maxControlPointSpacing,
				    nSearch);
		    }

		    Debug.LogFormat("number of spacings tried: {0}", nSearch);

		    return controlPointLengths;
	    }

	    private (List<List<float>>, int) OptimizeAllControlPoints(BezierPath bezierPath, float start, float smallestStep, float stepUpdate, float maxSpacing, int nSearch)
	    {
		    var step = 0.1f;
		    var spacing = start;
		    var lastValidSpacing = 0f;
		    while (step >= smallestStep)
		    {
			    Debug.LogFormat("Searching with step size = {0}", step);
			    for (var i = 0; i < Math.Floor((maxSpacing-lastValidSpacing)/step); i++)
			    {
				    nSearch++;
				    spacing = lastValidSpacing + step * i;
				    bezierPath.ControlLengths = Enumerable.Repeat(Enumerable.Repeat(spacing, 2).ToList(), path.Count).ToList();
				    VertexPath vertexPath = new VertexPath(bezierPath, visGraph.PathCreator.transform, visGraph.vertexSpacing);
				    var isValid = true;
				    for (var j = 0; j < vertexPath.localPoints.Length-1; j++)
				    {
					    var a = vertexPath.localPoints[j];
					    var b = vertexPath.localPoints[j+1];
					    var aIndices = visGraph.Convert_coords_to_indices(a.x, a.z);
					    var bIndices = visGraph.Convert_coords_to_indices(b.x, b.z);
					    isValid = visGraph.Check_grid_between_indices(aIndices[0], aIndices[1], bIndices[0], bIndices[1]);
					    if (!isValid)
					    {
						    Debug.LogFormat("collision at spacing = {0}", spacing);
						    break;
					    }
				    }

				    if (!isValid) break;
				    lastValidSpacing = spacing;
				    controlPointLengths = Enumerable.Repeat(Enumerable.Repeat(lastValidSpacing, 2).ToList(), path.Count).ToList();
			    }
			    step *= stepUpdate;
		    }
		    Debug.LogFormat("number of spacings tried: {0}", nSearch);

		    return (controlPointLengths, nSearch);
	    }

	    private (List<float>, int) OptimizeControlPointSpacing(BezierPath bezierPath, int anchorIndex, float start, float smallestStep, float stepUpdate, float maxSpacing, int nSearch)
	    {
		    var step = 0.1f;
		    var spacing = start;
		    var lastValidSpacing = 0f;
		    Debug.LogFormat("Searching for anchor {1} with step size = {0}", step, anchorIndex);
		    while (step >= smallestStep)
		    {
			    for (var i = 1; i < Math.Floor((maxSpacing-lastValidSpacing)/step); i++)
			    {
				    nSearch++;
				    spacing = lastValidSpacing + step * i;
				    // Update control length value for this anchor
					controlPointLengths[anchorIndex] = Enumerable.Repeat(spacing, 2).ToList();
					//var empty = Enumerable.Repeat(0.0f, path.Count).ToList();
				    //empty[1] = 0.2f;
				    bezierPath.ControlLengths = controlPointLengths; 
				    VertexPath vertexPath = new VertexPath(bezierPath, visGraph.PathCreator.transform, visGraph.vertexSpacing);

				    var isValid = false;
				    // Changing the control point spacing for this anchor can only affect the bezier path from 
				    // nodes anchor-1 to anchor+1
				    var inRelevantPath = false;
				    var endDist = 100f;
				    for (var j = 0; j < vertexPath.localPoints.Length-1; j++)
				    {
					    var a = vertexPath.localPoints[j];
					    var b = vertexPath.localPoints[j+1];
					    // Are we at the start of the relevant path segment?
					    if (Vector3.Distance(a, path[Math.Max(0, anchorIndex - 1)]) < visGraph.vertexSpacing * 2)
					    {
						    inRelevantPath = true;
					    }
					    if (!inRelevantPath) continue;
					    // At the end of the relevant path segment?
					    endDist = Vector3.Distance(b, path[Math.Min(path.Count - 1, anchorIndex + 1)]);
					    // Debug.Log(endDist);
					    if (endDist < visGraph.vertexSpacing * 2)
					    {
						    isValid = true;
						    break;
					    }
						var aIndices = visGraph.Convert_coords_to_indices(a.x, a.z);
					    var bIndices = visGraph.Convert_coords_to_indices(b.x, b.z);
					    isValid = visGraph.Check_grid_between_indices(aIndices[0], aIndices[1], bIndices[0], bIndices[1]);
					    if (!isValid)
					    {
						    Debug.DrawLine(a, b+Vector3.one*3, Color.magenta, 100f);
						    Debug.LogFormat("collision at spacing = {0} for vector {1}", spacing, j);
						    break;
					    }
				    }

				    if (!isValid) break;
			    }

			    lastValidSpacing = spacing-step;
			    step *= stepUpdate;
		    }

		    return (Enumerable.Repeat(lastValidSpacing, 2).ToList(), nSearch);
	    }
	    
	    private (float, int) OptimizeAsymmetricControlPointSpacing(BezierPath bezierPath, int anchorIndex, float start, float smallestStep, float stepUpdate, float maxSpacing, int nSearch, int controlPointIndex)
	    {
		    var step = 0.1f;
		    var spacing = start;
		    var lastValidSpacing = 0f;
		    Debug.LogFormat("Searching for anchor {1} cp {2} with step size = {0}", step, anchorIndex, controlPointIndex);
		    while (step >= smallestStep)
		    {
			    for (var i = 1; i < Math.Floor((maxSpacing-lastValidSpacing)/step); i++)
			    {
				    nSearch++;
				    spacing = lastValidSpacing + step * i;
				    // Update control length value for this anchor
					controlPointLengths[anchorIndex][controlPointIndex] = spacing;
					//var empty = Enumerable.Repeat(0.0f, path.Count).ToList();
				    //empty[1] = 0.2f;
				    bezierPath.ControlLengths = controlPointLengths; 
				    VertexPath vertexPath = new VertexPath(bezierPath, visGraph.PathCreator.transform, visGraph.vertexSpacing);
				    
				    var isValid = false;
				    // Changing the control point spacing for this anchor can only affect the bezier path from 
				    // nodes anchor-1 to anchor+1
				    var inRelevantPath = false;
				    var endDist = 100f;
				    var relevantPathStart = anchorIndex;
				    var relevantPathEnd = anchorIndex;
				    if (controlPointIndex == 0) relevantPathStart--;
				    else relevantPathEnd++;
				    for (var j = 0; j < vertexPath.localPoints.Length-1; j++)
				    {
					    var a = vertexPath.localPoints[j];
					    var b = vertexPath.localPoints[j+1];
					    // Are we at the start of the relevant path segment?
					    if (Vector3.Distance(a, path[Math.Max(0, relevantPathStart)]) < visGraph.vertexSpacing * 2)
					    {
						    inRelevantPath = true;
					    }
					    if (!inRelevantPath) continue;
					    // At the end of the relevant path segment?
					    endDist = Vector3.Distance(b, path[Math.Min(path.Count - 1, relevantPathEnd)]);
					    // Debug.Log(endDist);
					    if (endDist < visGraph.vertexSpacing * 2)
					    {
						    isValid = true;
						    break;
					    }
						var aIndices = visGraph.Convert_coords_to_indices(a.x, a.z);
					    var bIndices = visGraph.Convert_coords_to_indices(b.x, b.z);
					    isValid = visGraph.Check_grid_between_indices(aIndices[0], aIndices[1], bIndices[0], bIndices[1]);
					    if (!isValid)
					    {
						    Debug.DrawLine(a, b+Vector3.one*3, Color.magenta, 100f);
						    Debug.LogFormat("collision at spacing = {0} for vector {1}", spacing, j);
						    break;
					    }
				    }

				    if (!isValid) break;
			    }

			    lastValidSpacing = spacing-step;
			    step *= stepUpdate;
		    }

		    return (lastValidSpacing, nSearch);
	    }

	    public (Vector3[], Vector3, int[]) FindClosestLineSegment(Vector3 pos, bool useBezier)
	    {

		    var minDistance = float.PositiveInfinity;
		    var closestNodePair = new Vector3[2];
		    var closestNodePairIndices = new int[2];
		    // Todo remove
		    var smallestProjection = new Vector3();
		    List<Vector3> usedPath = path;
		    if (useBezier) usedPath = VectorizedBezierPath;
		    // Skip last node to avoid index error
		    for (var i = 0; i < usedPath.Count - 1; i++)
		    {
			    var a = usedPath[i];
			    var b = usedPath[i + 1];
			    var relativePos = pos - a;
			    var projectedPos = FindProjection(relativePos, a, b);
			    var distToSegment = (projectedPos - pos).magnitude;

			    if (distToSegment > minDistance) continue;
			    minDistance = distToSegment;
			    closestNodePair[0] = a;
			    closestNodePair[1] = b;
			    closestNodePairIndices[0] = i;
			    closestNodePairIndices[1] = i + 1;
			    // Todo remove
			    smallestProjection = projectedPos;
		    }
				    
		    // Todo remove
		    var diff = closestNodePair[1] - closestNodePair[0];
		    var mid = closestNodePair[0] + diff / 2;
		    // Debug.DrawLine(pos, smallestProjection, Color.cyan);


		    return (closestNodePair, smallestProjection, closestNodePairIndices);
	    }
	    
	    // Projects a position onto a path segment.
	    public static Vector3 FindProjection(Vector3 p, Vector3 a, Vector3 b)
	    {
		    var pathSegment = b - a;
		    var projectionScale = Vector3.Dot(p, pathSegment) / pathSegment.magnitude;
		    var projectedPos = a + pathSegment.normalized * projectionScale;
		    // If the normal point is not within the path segment, set the projected point to be the closest
		    // node on the segment.
		    if (projectedPos.x < Math.Min(a.x, b.x) ||
		        projectedPos.x > Math.Max(a.x, b.x) ||
		        projectedPos.z < Math.Min(a.z, b.z) ||
		        projectedPos.z > Math.Max(a.z, b.z))
		    {
			    var distA = p.magnitude;
			    var distB = (b - p).magnitude;
			    projectedPos = distA < distB ? a : b;
		    }

		    return projectedPos;
	    }

	    private float[] CalculateAverageCurvatures(float[] actualCurvatures)
	    {
		    float[] avgCurvature = new float[actualCurvatures.Length];
		    for (int i = 0; i < avgCurvature.Length; i++)
		    {
			    var start = Math.Max(0, i - visGraph.kCentralMovingAverage);
			    var end = Math.Min(actualCurvatures.Length-1, i + visGraph.kCentralMovingAverage);
			    // Must be non-zero
			    var n = end - start;
			    var segment = new ArraySegment<float>(actualCurvatures, start, n);
			    avgCurvature[i] = segment.Sum() / n;
		    }

		    return avgCurvature;
	    }

	    private float[] CalculateCurvatures()
	    {
		    float[] curvatures = new float[VectorizedBezierPath.Count];
		    var previousTangent = VectorizedBezierPath[1] - VectorizedBezierPath[0];
		    curvatures[0] = 0;
		    curvatures[1] = 0;
		    for (var i = 2; i < VectorizedBezierPath.Count; i++)
		    {
			    var tangent = VectorizedBezierPath[i] - VectorizedBezierPath[i - 1];
			    var approxSecondDerivative = tangent - previousTangent;
			    curvatures[i] = CalculateCurvature(tangent, approxSecondDerivative);
			    previousTangent = tangent;
		    }

		    return curvatures;
	    }
	    
	    
	    // For one point
	    private float CalculateCurvature(Vector3 firstDer, Vector3 secondDer)
	    {
		    var numerator = Mathf.Abs(firstDer.x * secondDer.z - firstDer.z * secondDer.x);
		    var denominator = Mathf.Pow(Mathf.Pow(firstDer.x, 2) + Mathf.Pow(firstDer.z, 2), 1.5f);
		    return numerator / denominator;
	    }

	    private float CalculateImprovisedCurvature(Vector3 secondDer)
	    {
		    return secondDer.magnitude;
	    }
	}
}
