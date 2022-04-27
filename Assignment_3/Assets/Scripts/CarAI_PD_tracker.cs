using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Linq;
using UnityEngine.Serialization;
using UnityEditor;
using PathCreation;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI_PD_tracker : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private VisibilityPath visPath;

        public GameObject[] friends;
        public GameObject[] enemies;

        public GameObject my_target;
        public Vector3 target_velocity;
        Vector3 old_target_pos;
        Vector3 desired_velocity;

        public float k_p = 2f;
        public float k_d = 0.5f;

        Rigidbody my_rigidbody;

        // Whether to enable AI tracknig
        public bool tracking;
        // The target velocity of the car
        [FormerlySerializedAs("targetVelocity")] public float desiredSpeed = 20f;
        // The radius of the path. When the target is within this distance of the path, we only steer forwards.
        public float pathRadius = 8f;
        // The coefficient for the vector to the next node when computing the desired velocity.
        public float nextNodeIncentive = 0.2f;
        // The coefficient of the velocity in computing the look ahead point.
        public float lookAheadVel = 0.5f;
        // The coefficient of the forward vector when computing the look ahead point.
        public float lookAheadForward = 1f;
        // Controls the momentum for the acceleration
        [Range(0, 1)] public float beta = 0.2f;
        
        // controls for angular coefficient
        [Range(0, 1000)] public float ang_coeff = 50f;
        // Visualization controls
        public bool show_complete_visability = true;
        public bool show_path = true;
        public bool show_node_weights = true;
        public bool show_node_orientations = true;
        
        private Vector3 oldDirection = Vector3.zero;
        
        private BezierPath bezierPath; 
        public PathCreator pathCreator;
        public float vertexSpacing = .1f; 
        public bool showSmoothPath = true;
        private VertexPath vertexPath;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            my_rigidbody = GetComponent<Rigidbody>();

            old_target_pos = my_target.transform.position;
            
            // Create visibility path
            visPath = new VisibilityPath(terrain_manager, ang_coeff);
            
            // Visualize the node weights
            if (show_node_weights)
            {
	            foreach (var node in visPath.nodes)
	            {
		            Vector3 start_vt = new Vector3(node.x, 0, node.z);
		            Vector3 end_vt = new Vector3(node.x, node.cost, node.z);
		            Debug.DrawLine(start_vt, end_vt, Color.red, 100f);    
	            }
            }

            // Visualize complete visibility graph
            if (show_complete_visability)
            {
	            foreach (var node in visPath.nodes)
	            {
		            Vector3 current_pos = new Vector3(node.x, 0, node.z);
		            // Visit the connecting nodes
		            foreach (var visit_id in node.edge_ids)
		            {
			            Vector3 visit_pos = new Vector3(visPath.nodes[visit_id].x, 0, visPath.nodes[visit_id].z);
			            Debug.DrawLine(current_pos, visit_pos, Color.yellow, 100f);
		            }
	            }
            }

            // Visualize the selected path
            if (show_path)
            {
	            if (visPath.path_status)
	            {
		            Vector3 old_vt =  new Vector3(visPath.nodes[0].x, 0, visPath.nodes[0].z);
		            foreach (var vt in visPath.vis_path)
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
            
            // Visualize the orientation of nodes
            if (show_node_orientations)
            {
	            int orient_scale = 10;
	            foreach (var node in visPath.nodes)
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
            
            bezierPath = new BezierPath(visPath.vis_path, false, PathSpace.xz);
            bezierPath.ControlPointMode = BezierPath.ControlMode.Automatic;
            bezierPath.AutoControlLength = OptimizeControlPointSpacing();
            vertexPath = new VertexPath(bezierPath, pathCreator.transform, vertexSpacing);
            if (showSmoothPath)
            {
	            Vector3 old_vt =  new Vector3(vertexPath.localPoints[0].x, 0, vertexPath.localPoints[0].z);
	            foreach (var vt in vertexPath.localPoints)
	            {
		            for (int i = 0; i < 3; i++)
		            {
			            var offset = 0.1f * i;
			            Debug.DrawLine(old_vt + new Vector3(offset, 0f, offset), 
				            vt + new Vector3(offset, 0f, offset), Color.red, 100f);
					    
		            }
		            old_vt = vt;
		            Console.WriteLine(vt);
	            }
            }
        }
        
        private void FixedUpdate()
        {
	        var currentPos = transform.position;
	        UsePdTracker(currentPos, vertexPath.localPoints.ToList());
	        //UsePurePursuit(currentPos);
        }

        private void UsePurePursuit(Vector3 currentPosition)
        {
	        var (direction, b) = SteerPath();
	        PurePursuit(currentPosition, b, direction);
        }

        private void UsePdTracker(Vector3 currentPos, List<Vector3> path)
        {
	        // Get the steering direction
	        var (closestPath, projectedCar) = FindClosestLineSegment(transform.position, path);
	        // var targetVector = currentPos + lookAheadForward * transform.forward + lookAheadVel * my_rigidbody.velocity;
	        // var (closestPathTarget, projectedTarget) = FindClosestLineSegment(targetVector, path);
	        PdTracker(currentPos, projectedCar, transform.forward);
        }

        private void PurePursuit(Vector3 currentPosition, Vector3 nextNodePosition, Vector3 steerDirection)
        {
	        // Momentum
	        var newDirection = beta * oldDirection + (1 - beta) * steerDirection;
	        oldDirection = newDirection;
	        // Compute the desired velocity by scaling the steering steerDirection by the target velocity
	        // Add vector from position to next node,  incentivizing traveling towards it.
	        Vector3 desiredVelocity = newDirection.normalized * desiredSpeed + nextNodeIncentive * (nextNodePosition - currentPosition);
	        Vector3 velocityError = desiredVelocity - my_rigidbody.velocity;
	        Vector3 desiredAcceleration = k_d * velocityError;
	        
	        float steering = Vector3.Dot(newDirection, transform.right);
	        float acceleration = Vector3.Dot(desiredAcceleration, transform.forward);
	        
	        if (tracking)
	        {
		        m_Car.Move(steering, acceleration, acceleration, 0f);
	        }

	        Debug.DrawLine(currentPosition, currentPosition + desiredAcceleration, Color.black);
	        Debug.DrawLine(currentPosition, currentPosition + steerDirection, Color.white);
	        Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
        }

        private void PdTracker(Vector3 currentPosition, Vector3 targetPosition, Vector3 forwardDirection)
        {
	        // keep track of target velocity
	        target_velocity = (targetPosition - old_target_pos) / Time.fixedDeltaTime;
	        old_target_pos = targetPosition;

	        // Steering
	        var positionError = targetPosition - currentPosition;
	        var velocityError = target_velocity - my_rigidbody.velocity;
	        var desiredDirection = k_p * positionError + k_d * velocityError;
	        var steering = Vector3.Dot(desiredDirection, transform.right);
	        //Debug.LogFormat("speed forward:{0}", forwardSpeed);
	        //Debug.LogFormat("speed error:{0}", velocityError);

	        // Acceleration
	        var forwardSpeed = desiredSpeed * forwardDirection;
	        var forwardSpeedError = forwardSpeed - my_rigidbody.velocity;
	        var acceleration = Vector3.Dot(forwardSpeedError, transform.forward);

	        Debug.DrawLine(targetPosition, targetPosition + target_velocity, Color.red);
	        Debug.DrawLine(currentPosition, currentPosition + my_rigidbody.velocity, Color.blue);
	        Debug.DrawLine(currentPosition, currentPosition + desiredDirection, Color.black);

	        // this is how you control the car
	        //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
	        if (tracking)
	        {
		        m_Car.Move(steering, acceleration, acceleration, 0f);
	        }
        }

        private void LegacyTracker(Vector3 target_position)
        {
	        // keep track of target position and velocity
	        // Vector3 target_position = my_target.transform.position;
	        target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
	        old_target_pos = target_position;

	        // a PD-controller to get desired velocity
	        Vector3 position_error = target_position - transform.position;
	        Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
	        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

	        float steering = Vector3.Dot(desired_acceleration, transform.right);
	        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

	        Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
	        Debug.DrawLine(transform.position, transform.position + my_rigidbody.velocity, Color.blue);
	        Debug.DrawLine(transform.position, transform.position + desired_acceleration, Color.black);

	        // this is how you control the car
	        Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
	        if (tracking)
	        {
		        m_Car.Move(steering, acceleration, acceleration, 0f);
	        }
        }

        private (Vector3, Vector3) SteerPath()
        {
	        var currentPos = transform.position;

	        var (closestPathCar, projectedCar) = FindClosestLineSegment(currentPos, visPath.vis_path);
	        var bCarDist = (closestPathCar[1] - currentPos).magnitude;
	        var carPathDist = (projectedCar - currentPos).magnitude;
	        var targetVector = currentPos + lookAheadForward * transform.forward + lookAheadVel * my_rigidbody.velocity;
	        if (carPathDist != 0)
	        {
		        var targetVectorScaling = 1.0f;
		        if (bCarDist > 20 && carPathDist < 2) ;
		        {
			        carPathDist = Mathf.Clamp(carPathDist, 0.0f, 1.0f);
			        targetVectorScaling = 3.0f; // / carPathDist; // 0 - 5
			        //targetVectorScaling *= bCarDist; // suppress when < 20
		        }

		        // targetVector += transform.forward * targetVectorScaling;
	        }
	        Debug.LogFormat("CAR PATH DIST: {0}", carPathDist);
	        Debug.LogFormat("CAR b DIST: {0}", bCarDist);
	        var (closestPathSegment, projectedTarget) = FindClosestLineSegment(targetVector, visPath.vis_path);
	        var a = closestPathSegment[0];
	        var b = closestPathSegment[1];
	        // Target position relative to the first node of the path segment
	        var relativeTarget = targetVector - a;
			// Compute the distance vector between the target and the projected point on the path.
			// Might be used later for path radius check
			var targetToPath = relativeTarget - projectedTarget;
			var diff = targetVector - projectedTarget;
			Debug.DrawLine(projectedTarget, projectedTarget + diff.normalized * pathRadius, Color.black);
			if (targetToPath.magnitude < pathRadius)
			{
				return (Vector3.forward, b);
			}
			var steeringDirection = projectedTarget - currentPos;
			return (steeringDirection, b);
        }

        public (Vector3[], Vector3) FindClosestLineSegment(Vector3 pos, List<Vector3> path)
        {

	        var minDistance = float.PositiveInfinity;
	        var closestNodePair = new Vector3[2];
	        // Todo remove
	        var smallestProjection = new Vector3();
				    
	        // Skip last node to avoid index error
	        for (var i = 0; i < path.Count - 1; i++)
	        {
		        var a = path[i];
		        var b = path[i + 1];
		        var relativePos = pos - a;
		        var projectedPos = FindProjection(relativePos, a, b);
		        var distToSegment = (projectedPos - pos).magnitude;

		        if (distToSegment > minDistance) continue;
		        minDistance = distToSegment;
		        closestNodePair[0] = a;
		        closestNodePair[1] = b;
		        // Todo remove
		        smallestProjection = projectedPos;
	        }
				    
	        // Todo remove
	        var diff = closestNodePair[1] - closestNodePair[0];
	        var mid = closestNodePair[0] + diff / 2;
	        Debug.DrawLine(pos, smallestProjection, Color.cyan);


	        return (closestNodePair, smallestProjection);
        }
        
        // Projects a position onto a path segment.
        public Vector3 FindProjection(Vector3 p, Vector3 a, Vector3 b)
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

        private float OptimizeControlPointSpacing()
        {
	        const float start = 0f;
	        const float smallestStep = 0.001f;
	        const float stepUpdate = 0.5f;
	        var step = 0.1f;
	        var spacing = start;
	        var lastValidSpacing = 0f;
	        var n_search = 0;
	        while (step >= smallestStep)
	        {
		        Debug.LogFormat("Searching with step size = {0}", step);
		        for (var i = 0; i < Math.Floor(1/step); i++)
		        {
			        n_search++;
			        spacing = lastValidSpacing + step * i;
			        bezierPath.AutoControlLength = spacing; 
			        vertexPath = new VertexPath(bezierPath, pathCreator.transform, vertexSpacing);
			        var isValid = true;
			        for (var j = 0; j < vertexPath.localPoints.Length-1; j++)
			        {
				        var a = vertexPath.localPoints[j];
				        var b = vertexPath.localPoints[j+1];
				        var aIndices = visPath.Convert_coords_to_indices(a.x, a.z);
				        var bIndices = visPath.Convert_coords_to_indices(b.x, b.z);
				        isValid = visPath.Check_grid_between_indices(aIndices[0], aIndices[1], bIndices[0], bIndices[1]);
				        if (!isValid)
				        {
					        Debug.LogFormat("collision at spacing = {0}", spacing);
					        break;
				        }
			        }

			        if (!isValid) break;
			        lastValidSpacing = spacing;
		        }
		        step *= stepUpdate;
	        }
	        Debug.LogFormat("number of spacings tried: {0}", n_search);

	        return spacing;
        }
        
        public class VisibilityPath
		{
			// Map related
			// X, this size is fixed for both occupancy and config
			public float min_x; // meters
			public float max_x; // meters
			
			public int x_n; // number of x grid elements, ORIGINAL OCCUPANCY
			public float x_res; // (max - min)/n
			public int x_n_config; // number of x grid elements, CONFIG
			public float x_res_config; // (max - min)/n_config
			
			// Z, this size is fixed for both occupancy and config	
			public float min_z; // meters
			public float max_z; // meters
			
			public int z_n; // number of z grid elements
			public float z_res; // (max - min)/n
			public int z_n_config; // number of x grid elements, CONFIG
			public float z_res_config; // (max - min)/n_config
			
			public float[,] occupancy_grid;
			public float[,] config_grid;
			
			// Path related
			public Vector3 start_pos;
			public Vector3 goal_pos;
			
			public float vertex_offset;

			public List<node> nodes;
			public List<List<int>> graph_edge_ids;
			public List<Vector3> vis_path;

			public bool path_status = false;
			private int path_goal_id = -1;
			
			// coefficient for adding orientation deviation to the cost
			public float angular_coefficient;

		    public VisibilityPath(TerrainManager terrain_manager, float angular_coefficient)
		    {
			    // Coefficients
			    this.angular_coefficient = angular_coefficient;
			    
			    // Initialize lists
			    this.nodes = new List<node>();
			    this.graph_edge_ids = new List<List<int>>();
			    this.vis_path = new List<Vector3>();
			    
			    // Map Info
			    this.occupancy_grid = terrain_manager.myInfo.traversability;
			    
			    // X
			    this.min_x = terrain_manager.myInfo.x_low; // todo: was x_min
			    this.max_x = terrain_manager.myInfo.x_high; // todo: was x_max
			    this.x_n = terrain_manager.myInfo.x_N;
			    this.x_res = (this.max_x - this.min_x)/x_n;
					
			    // Z (Y)
			    this.min_z = terrain_manager.myInfo.z_low; // todo: was z_min
			    this.max_z = terrain_manager.myInfo.z_high; // todo: was z_max
			    this.z_n = terrain_manager.myInfo.z_N;
			    this.z_res = (this.max_z - this.min_z)/z_n;
					
			    // Start and goal
			    this.start_pos = terrain_manager.myInfo.start_pos;
			    this.goal_pos = terrain_manager.myInfo.goal_pos;
					
			    // Upsample
			    // Select scaling factor
			    int desired_res = 1;
			    int x_scale = (int)(this.x_res / desired_res);
			    int z_scale = (int)(this.z_res / desired_res);
				
			    // Perform upsampling
			    float[,] new_grid = do_grid_upsample(this.occupancy_grid, x_scale, z_scale);
			    
			    // Update the size of the upsampled config grid
			    this.x_n_config = this.x_n * x_scale;
			    this.z_n_config = this.z_n * z_scale;
			    
			    // Update the resolution of the config grid
			    this.x_res_config = (this.max_x - this.min_x) / this.x_n_config;
			    this.z_res_config = (this.max_z - this.min_z) / this.z_n_config;

			    // Form the config grid <- Same size as new_grid above
			    int desired_border = 2;
			    this.config_grid = do_grid_config(new_grid, desired_border);
			    
			    // VisibilityPath_2nd_construct
			    //VisibilityPath vis_path_find = new VisibilityPath(config_grid, x_min, x_max, z_min, z_max, angular_coefficient);
			    
			    this.perform_planning(this.start_pos, this.goal_pos, (float)0.2);

			    vis_path = new List<Vector3>();
			    vis_path = this.generate_path();
		    }

		    public class node
			    {
				    public float x;
				    public float z;
				    public int parent;
				    public float cost;
				    public float orientation;
				    public float parent_orientation;
				    public List<int> edge_ids;

				    public node(float x, float z, float cost = 0, int parent = -1, float orientation = 0f, float parent_orientation = 0f)
				    {
					    this.x = x;
					    this.z = z;
					    this.cost = cost;
					    this.parent = parent;
					    this.orientation = orientation;
					    this.parent_orientation = parent_orientation;
					    this.edge_ids = new List<int>();
				    }
			    }

			// Constructor
			// Todo What is this constructor used for?
			/*public VisibilityPath_2nd_construct(float[,] occupancy_grid, float min_x, float max_x, float min_z, float max_z, float angular_coefficient)
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
				    
				    // Set coefficients
				    this.angular_coefficient = angular_coefficient;
			    }*/

			// Main methods

			public bool perform_planning(Vector3 start, Vector3 goal, float offset)
			    {
				    // Performs path plannning 
				    // Find x and z of start and goal vector3
				    float start_x = start.x;
				    float start_z = start.z;

				    float goal_x = goal.x;
				    float goal_z = goal.z;

				    // Add the start and end points to the node list, graph
				    node start_node = new node(start_x, start_z, 0f, -1, (float)Math.PI/2);
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
						    this.path_status = false;
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
						    // new_cost = dist_cost + angular_coefficient * ang_cost + prev_cost
						    float dist_cost = costs[current_id, visit_id];
						    float prev_cost = this.nodes[current_id].cost + costs[current_id, visit_id];
						    
						    // Angular portion of the cost function
						    float new_angle = this.calc_node_orientation(visit_id, current_id);
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
						    new_cost = dist_cost + this.angular_coefficient * ang_cost + prev_cost;

						    
						    // visit existing valid neighbor node
						    if (open_list.Exists(element => element == visit_id))
						    {
							    if (new_cost < this.nodes[visit_id].cost)
							    {
								    // Update nodes cost, parent, orientation of an already visited node
								    this.nodes[visit_id].cost = new_cost;
								    this.nodes[visit_id].parent = current_id;
								    this.nodes[visit_id].orientation = new_angle;
								    this.nodes[visit_id].parent_orientation = this.nodes[current_id].orientation;

							    }
						    }
						    // visit valid new and non closed node
						    else if (!closed_list.Exists(element => element == visit_id))
						    {
							    // Update nodes cost, parent, orientation of a newly visited node
							    this.nodes[visit_id].cost = new_cost;
							    this.nodes[visit_id].parent = current_id;
							    this.nodes[visit_id].orientation = new_angle;
							    this.nodes[visit_id].parent_orientation = this.nodes[current_id].orientation;
							    
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
				    
				    // reverse as path is form from the goal
				    path.Reverse();
			    }
			    else
			    {
				    // this should be start and goal < this case might not be needed!!
				    path.Add(this.goal_pos);
				    path.Add(this.start_pos);
			    }
			   
			    // Store path and return path
			    this.vis_path = path;
			    return path;
		    }
			    
			// Methods for pre preparing the grid
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
				    // find_node_id
				    // return the index of the input node as it is found in this.nodes
				    
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
					    if (min_val == -1 || this.nodes[node_id].cost < min_val)
					    {
						    min_val = this.nodes[node_id].cost;
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
				node child_node = this.nodes[child_node_id];
				node parent_node = this.nodes[parent_node_id];
				
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
				node child_node = this.nodes[child_node_id];
				node parent_node = this.nodes[parent_node_id];
				
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
		}
    }
}
