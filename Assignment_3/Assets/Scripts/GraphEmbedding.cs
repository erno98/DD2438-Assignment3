using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using Newtonsoft.Json;
//using Priority_Queue;
using System.Linq;

public class GraphEmbedding
// class representing a graph embedding used in the code
{
    // ----------------------------- attributes -----------------------------

    // string containing the path to the terrain
    private string _terrain_filename;

    // dictionary attribute containing the embedding
    public Dictionary<Tuple<int,int>, List<Tuple<int,int>>> embedding;

    // terrain storied in 2D float array
    public float[,] terrain_arr;

    // indices of the terrain cells stored in 2D array with coordinates 
    public int[,,] terrain_indices;

    // int representing the resolution of the embedding, !!!must be power of 2!!!
    // for example, 1 will give embedding where each terrain block represents a node
    // resolution=4 will mean that each building block is divided into 4 equal sized blocks
    // so in total there will be 4 times more nodes in the embedding
    private int resolution;

    // weights of the nodes in the embedding
    private Dictionary<Tuple<int, int>, double> weights;

    // padding to control how close the nodes can be to obstacles
    private float padding;


    // -------------------------- methods ---------------------------
    
    // constructor
    public GraphEmbedding(string terrain_filename, int resolution, float padding){
        this._terrain_filename = terrain_filename;
        this.padding = padding;
        this.resolution = resolution;
        this.terrain_arr = this.read_terrain();
        this.terrain_indices = this.get_terrain_indices();
        this.set_embedding_from_terrain();
        this.init_weights();
        
    }

    // embedding from coordinates, necessary for P1
    public GraphEmbedding(List<Vector3> coords, GraphEmbedding root_embedding, TerrainInfo terrain_info){
        this._terrain_filename = root_embedding._terrain_filename;
        this.resolution = root_embedding.resolution;
        this.padding = root_embedding.padding;
        var emb = new Dictionary<Tuple<int,int>, List<Tuple<int,int>>>();

        foreach(var coord in coords){
            var coord_i_idx = this.get_i_index(coord[0], terrain_info);
            var coord_j_idx = this.get_j_index(coord[2], terrain_info);
            var coord_tuple = new Tuple<int, int>(coord_i_idx, coord_j_idx);

            if(emb.Keys.ToList().Contains(coord_tuple)){
                continue;
            } else{
                emb.Add(coord_tuple, root_embedding.embedding[coord_tuple]);
            }
        }

        this.embedding = emb;
    }

    public List<Vector3> get_all_embedding_cords(TerrainInfo terrain_info){
        var cords = new List<Vector3>();
        foreach (var k in this.embedding.Keys){
            var adjacent = this.embedding[k];
            cords.Add(new Vector3(this.get_x_pos(k.Item1, terrain_info), 0, this.get_z_pos(k.Item2, terrain_info)));
            foreach (var adj in adjacent){
                cords.Add(new Vector3(this.get_x_pos(adj.Item1, terrain_info), 0, this.get_z_pos(adj.Item2, terrain_info)));
            }
        } 

        var unique_cords = cords.Distinct().ToList();

        return unique_cords;
    }

    public void draw_embedding(TerrainInfo terrain_info){
        var emb_debug_str = "Embedding\n";
        foreach (var k in this.embedding.Keys){
            var adjacent = this.embedding[k];
            emb_debug_str = emb_debug_str + "[" + k + "]: "; 
            foreach (var adj in adjacent){
            emb_debug_str = emb_debug_str + adj + ", ";
            Debug.DrawLine(
                new Vector3(this.get_x_pos(k.Item1, terrain_info), 0, this.get_z_pos(k.Item2, terrain_info)),
                new Vector3(this.get_x_pos(adj.Item1, terrain_info), 0, this.get_z_pos(adj.Item2, terrain_info)),
                Color.blue, 100f
            );
            }
            emb_debug_str += "\n";
        } 
        //Debug.Log(emb_debug_str);
    }

    private TerrainInfo get_terrain_data(){
        var jsonTextFile = Resources.Load<TextAsset>(this._terrain_filename);
        return TerrainInfo.CreateFromJSON(jsonTextFile.text); 
    }

    // private method that reads the contents of terrain filename (string)
    public float[,] read_terrain(){
        
        // load the traversability matrix
        var terrain_json = this.get_terrain_data().traversability; 

        var shape_x = terrain_json.GetLength(0);
        var shape_y = terrain_json.GetLength(1);

        var new_shape_x = shape_x * this.resolution;
        var new_shape_y = shape_y * this.resolution;

        // apply resolution (expand the terrain array)
        // possible TODO: make it dynamic given environment and obstacles
        // possible TODO: don't apply it on obstacles
        var new_terrain = new Single[new_shape_x, new_shape_y];

        for(int col_idx=0; col_idx < shape_x; col_idx++){
            for(int row_idx=0; row_idx < shape_y; row_idx++){
                for(int i_idx=0; i_idx < this.resolution; i_idx++){
                    for(int j_idx=0; j_idx < this.resolution; j_idx++){
                        new_terrain[col_idx*this.resolution+i_idx, row_idx*this.resolution+j_idx] = terrain_json[col_idx, row_idx];
                    }
                }
            }
        }

        return new_terrain;
    }



    public void set_weights(Tuple<int, int> node, List<Tuple<int, int>> visited_nodes, int distance){

        var adjacent_nodes = new List<Tuple<int, int>>(this.embedding[node]);
        adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

        if (adjacent_nodes.Count == 0){
            return;
        }

        foreach(var adjacent_node in adjacent_nodes){
            this.weights[adjacent_node] = distance;
            visited_nodes.Add(adjacent_node);
            this.set_weights(adjacent_node, visited_nodes, ++distance);
        }

    }

    private void init_weights(){

        // initialize weights to random between 0 and 1

        System.Random rng = new System.Random();
        Dictionary<Tuple<int, int>, double> weights = new Dictionary<Tuple<int, int>, double>();
        foreach(var node in this.embedding.Keys){
            weights.Add(node, rng.Next(0, 10000)/10000f);

        }
        this.weights = weights;
    }

    public void update_weights(List<Tuple<int, int>> nodes, Tuple<bool, List<double>> walks){

        // update the weights
        var multiplier = 1;

        if (walks.Item1){
            multiplier = 1;
        } else{
            // penalize the nodes that did not lead to the goal
            multiplier = 20;
        }

        if (walks.Item1){

            for(int i=0; i<nodes.Count; i++){
                // if goal is reached, set the weight as the node distance to the goal of the given node
                this.weights[nodes[i]] =  walks.Item2[i];
            }
            
        }else{
            for(int i=0; i<nodes.Count; i++){
                // goal was not reached, multiply the current weight
                this.weights[nodes[i]] = this.weights[nodes[i]] * (i+1);
            }
        }

    }

    // private void reset_except(List<Tuple<int, int>> nodes){
    //     for(int i=0; i<nodes.Count; i++){
    //         for(int j=0; j<this.weights.Count; j++){
    //             if (this.weights[i] != )
    //         }
    //     }

    // }

    // method that returns 2D array of indices of the nodes
    private int[,,] get_terrain_indices(){

        // generate the indices
        var indices = new int[this.terrain_arr.GetLength(0), this.terrain_arr.GetLength(1), 2];

        // each cell gets an individual index
        for(int col_idx=0; col_idx < this.terrain_arr.GetLength(0); col_idx++){
            for(int row_idx=0; row_idx < this.terrain_arr.GetLength(1); row_idx++){
                indices[col_idx, row_idx, 0] = col_idx;
                indices[col_idx, row_idx, 1] = row_idx;
            }
        }

        return indices;
    }

    

    public float get_x_pos(int i, TerrainInfo terrain_info){
        float step = (terrain_info.x_high - terrain_info.x_low) / (terrain_info.x_N*this.resolution);
        return terrain_info.x_low + step / 2 + step * i;

    }


    public float get_z_pos(int j, TerrainInfo terrain_info){
        float step = (terrain_info.z_high - terrain_info.z_low) / (terrain_info.z_N*this.resolution);
        return terrain_info.z_low + step / 2 + step * j;
    }


    public int get_i_index(float x, TerrainInfo terrain_info){
        int index = (int) Mathf.Floor(terrain_info.x_N*this.resolution * 
                    (x - terrain_info.x_low) / (terrain_info.x_high - terrain_info.x_low));
        if (index < 0)
        {
            index = 0;
        }else if (index > terrain_info.x_N*this.resolution - 1)
        {
            index = terrain_info.x_N*this.resolution - 1;
        }
        return index;
    }


    public int get_j_index(float z, TerrainInfo terrain_info){
        int index = (int) Mathf.Floor(terrain_info.z_N*this.resolution * 
                         (z - terrain_info.z_low) / (terrain_info.z_high - terrain_info.z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > terrain_info.z_N*this.resolution - 1)
        {
            index = terrain_info.z_N*this.resolution - 1;
        }
        return index;  
    }

    public float[,,] get_terrain_coordinates(){

        // return the coordinates of the terrain blocks

        var terrain = this.terrain_arr;
        var shape_x = terrain.GetLength(0);
        var shape_y = terrain.GetLength(1);
        float[,,] terrain_coords = new float[shape_x, shape_y, 2];

        var terrain_info = this.get_terrain_data(); 

        for(int x=0; x < shape_x; x++){
            for(int y=0; y < shape_y; y++){
                // terrain_coords[x,y,0] = terrain_info.get_x_pos(this.terrain_indices[x,y,0]);
                // terrain_coords[x,y,1] = terrain_info.get_z_pos(this.terrain_indices[x,y,1]);
                terrain_coords[x,y,0] = this.get_x_pos(this.terrain_indices[x,y,0], terrain_info);
                terrain_coords[x,y,1] = this.get_z_pos(this.terrain_indices[x,y,1], terrain_info);

            }
        }

        return terrain_coords;
    }


    public Vector3[,] get_terrain_coordinates_vector3(){

        var terrain = this.terrain_arr;
        var shape_x = terrain.GetLength(0);
        var shape_y = terrain.GetLength(1);
        Vector3[,] terrain_coords = new Vector3[shape_x, shape_y];

        var terrain_info = this.get_terrain_data(); 
        for(int x=0; x < shape_x; x++){
            for(int y=0; y < shape_y; y++){
                terrain_coords[x,y] = new Vector3(this.get_x_pos(this.terrain_indices[x,y,0], terrain_info),
                                                  0, 
                                                  this.get_z_pos(this.terrain_indices[x,y,1], terrain_info));

            }
        }

        return terrain_coords;
        
    }
    
    public double node_distance(Tuple<int,int> node1, Tuple<int,int> node2){

        // euclidean
        var distance =  Math.Sqrt(((node1.Item1 - node2.Item1) * (node1.Item1 - node2.Item1) + 
                    (node1.Item2 - node2.Item2) * (node1.Item2 - node2.Item2))); 

        return distance;
    }
    
    public double distance(double x1, double x2, double y1, double y2){
        return Math.Sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    public double h(Tuple<int,int> target_node, Tuple<int,int> goal_node){
        // return this.weights[target_node] + this.node_distance(target_node, goal_node)/2;
        return this.node_distance(target_node, goal_node);
    }



    private bool is_adjacent(int x1, int y1, int x2, int y2){
        // method that checks if two cells are adjacent
        // note: we treat one point to be adjacent to itself
        // below for p1
        //return Math.Sqrt(Math.Pow(x1-x2,2) + Math.Pow(y1-y2,2)) <= 1;
        // below for p2-p5
        return ( Math.Abs(x1 - x2) <= 1 && Math.Abs(y1 - y2) <= 1);
    } 

    private bool is_different(int x1, int y1, int x2, int y2){
        return !(x1 == x2 && y1 == y2);
    }

   /*  private bool is_traversable(int x1, int y1, int x2, int y2){
        // checks if we can traverse through two points, i.e. both are paths and not terrain blockades
        return (this.terrain_arr[x1, y1] == 0.0 && this.terrain_arr[x2, y2] == 0.0);
    } */

      private bool is_traversable(int x1, int y1, int x2, int y2){
        // checks if we can traverse through two points, i.e. both are paths and not terrain blockades

        var terrain_info = this.get_terrain_data(); 
        Vector2 start=new Vector2(this.get_x_pos(x1,terrain_info),this.get_z_pos(y1,terrain_info));
        Vector2 end=new Vector2(this.get_x_pos(x2,terrain_info),this.get_z_pos(y2,terrain_info));

        EmbeddedLine el=new EmbeddedLine(new Line(start,end),this.padding,this._terrain_filename);

        return (this.terrain_arr[x1, y1] == 0.0 && this.terrain_arr[x2, y2] == 0.0) && !(el.intersects_non_transversable());
    }


    // method that accepts the array representing the terrain (2D float array)
    // and sets it as the object's in correct format
    private void set_embedding_from_terrain(){

        var indices = this.terrain_indices;
        var terrain_arr = this.terrain_arr;

        Dictionary<Tuple<int,int>, List<Tuple<int,int>>> embedding = new Dictionary<Tuple<int,int>, List<Tuple<int,int>>>();

        var shape_x = terrain_arr.GetLength(0);
        var shape_y = terrain_arr.GetLength(1);

        // for each index
        // possible TODO: optimize to traverse only through adjacent cells
        for(int x1=0; x1 < shape_x; x1++){
            for(int y1=0; y1 < shape_y; y1++){

                var curr_x = indices[x1, y1, 0];
                var curr_y = indices[x1, y1, 1];
                Tuple<int,int> curr_idx = new Tuple<int,int>(curr_x, curr_y);

                // check other indices
                for(int x2=0; x2 < shape_x; x2++){
                    for(int y2=0; y2 < shape_y; y2++){
                        if ( this.is_adjacent(x1, y1, x2, y2) && this.is_traversable(x1, y1, x2, y2) && this.is_different(x1, y1, x2, y2)){

                            // possible TODO: optimize not to check whether key exists every time

                            // check if cell idx already in embedding
                            if (embedding.ContainsKey(curr_idx)){
                                // append to list
                                embedding[curr_idx].Add(new Tuple<int,int>(indices[x2, y2, 0],indices[x2, y2, 1]));
                            } else{
                                // add new index and value
                                embedding.Add(curr_idx, new List<Tuple<int,int>>{ new Tuple<int,int>(indices[x2, y2,0], indices[x2, y2,1]) } );

                            }
                        }
                    }
                }
                

            }
        }

        // set the embedding as attribute of the object
        this.embedding = embedding;

    }
      /*
    // Method to create minimum spanning tree of graph embedding
    public Dictionary<Tuple<int, int>, List<Tuple<int, int>>> minimum_spanning_tree(Tuple<int, int> start_indices){
        
        // Get info about the terrain
        var terrain_info = this.get_terrain_data();

        // Create a priority queue for vertices in the graph
        SimplePriorityQueue<Tuple<int, int>, double> queue = new SimplePriorityQueue<Tuple<int, int>, double>();

        // Initializing all priorities to inf. Initialize the priority of the start position to 0.
        foreach(var node in embedding.Keys){
            if (node.Item1 == start_indices.Item1 && node.Item2 == start_indices.Item2){
                queue.Enqueue(node, 0);
                continue;
            }
            queue.Enqueue(node, double.MaxValue);
        }

        // Create a set for the tree
        Dictionary<Tuple<int, int>, Tuple<int, int>> mst = new Dictionary<Tuple<int, int>, Tuple<int, int>>();

        // Create an intermediate set for the parents
        Dictionary<Tuple<int, int>, Tuple<int, int>> parents = new Dictionary<Tuple<int, int>, Tuple<int, int>>();

        // Add an artificial parent to the start indices
        parents.Add(start_indices, start_indices);

        // Adding indices to the spanning tree
        // While not all indices have been added to the spanning tree
        while(mst.Count < this.embedding.Count){

            // Find the indices with the least cost that doesn't exist in the spanning tree
            Tuple<int, int> current = queue.First;
            double priority = queue.GetPriority(current);
            current = queue.Dequeue();

            if(!mst.ContainsKey(current)){

                // Get the parent of the current node
                Tuple<int, int> parent = parents[current];

                // Add the indices to the spanning tree with pointer to its parent
                mst.Add(current, parent);

                // Update the costs of the adjacent indices which does not already exist in the spanning tree
                var adjacent = this.embedding[current];
                foreach (var adj in adjacent){

                    // If the node is in the queue with a priority higher than the current priority and is not in the spanning tree
                    if (queue.Contains(adj) && !mst.ContainsKey(adj) && priority < queue.GetPriority(adj)){
                        
                        // Update the priority
                        queue.UpdatePriority(adj, priority + 1);

                        // Update the parent
                        if(parents.ContainsKey(adj)){
                            parents.Remove(adj);
                        }

                        parents.Add(adj, current);

                    }

                }

            }


        }

        Dictionary<Tuple<int, int>, List<Tuple<int, int>>> mst_embedding = new Dictionary<Tuple<int, int>, List<Tuple<int, int>>>();

        // Checking every node of the minimum spanning tree
        foreach(var node in mst.Keys){

            // Add the node to the final embedding with its parent as a neighboor
            Tuple<int, int> parent = mst[node];
            mst_embedding.Add(node, new List<Tuple<int, int>>());
            mst_embedding[node].Add(parent);

            // Also include all the nodes which are children of the current node
            foreach(var child in mst.Keys){

                // If the child has the current node of above as parent
                if(mst[child].Item1 == node.Item1 && mst[child].Item2 == node.Item2){
                    
                    if(!mst_embedding[node].Contains(child)){
                        mst_embedding[node].Add(child);
                    }

                }

            }

        }

        return mst_embedding;

    }

    // Method to generate a path that circumnavigates a spanning tree
    public List<Vector3> circumnav_mst(Dictionary<Tuple<int, int>, List<Tuple<int, int>>> mst, Vector3 start_pos){

        // Get info about the terrain
        var terrain_info = this.get_terrain_data();

        // Find closest point in tree
        // Transform start position to grid indices (rounding)
        int start_i = get_i_index(start_pos.x, terrain_info);
        int start_j = get_j_index(start_pos.z, terrain_info);
        Tuple<int, int> current_ind = new Tuple<int, int>(start_i, start_j);

        // Transform grid incides back to their (x,y)-position
        Vector3 current_pos = new Vector3(get_x_pos(start_i, terrain_info), 0, get_z_pos(start_j, terrain_info));
        Vector3 previous_pos = start_pos;

        // Init
        Vector3 next_pos;
        Vector3 next_direction;
        List<Vector3> my_path = new List<Vector3>();
        bool circumnavigating = true;

        while(circumnavigating){

            // Identify the case
            // Compute the initial heading
            Vector3 current_direction = (current_pos - previous_pos) . normalized;
            string current_case;

            // Get the neighbours of the closest point (next point)
            List<Tuple<int, int>> current_neighbours = mst[current_ind];
            Dictionary<float, Tuple<int, int>> current_angles = new Dictionary<float, Tuple<int, int>>();

            foreach(Tuple<int, int> neighbour in current_neighbours){

                next_pos =  new Vector3(get_x_pos(neighbour.Item1, terrain_info), 0, get_z_pos(neighbour.Item2, terrain_info));
                next_direction = (next_pos - current_pos) . normalized;
                float next_angle = Vector3.SignedAngle(next_direction, current_direction, Vector3.down); // Measured clockwise?!?!!

                if(next_angle == -180f){

                    next_angle = 180f;

                }
                
                if(!current_angles.ContainsKey(next_angle) && !next_pos.Equals(current_pos)){

                    current_angles.Add(next_angle, neighbour);

                }

            }

            // Find the rightmost turn (counted counter-clockwise from the current direction)
            if(current_angles.ContainsKey(90f)) current_case = "right_turn";

            else if(current_angles.ContainsKey(0f)) current_case = "forward_drive";

            else if(current_angles.ContainsKey(-90f)) current_case = "left_turn";

            else current_case = "dead_end";

            // Init
            Tuple<int, int> next_ind;
            Vector3 perp_direction;
            Vector3 new_point;
            Vector3 new_point2;
            Vector3 new_point3;
            Vector3 new_point4;
            Vector3 new_point5;
            Vector3 new_point6;
            Vector3 new_point7;
            Vector3 new_point8;

            // Decide on next point to travel to and add points based on the current case
            switch (current_case){

                case "right_turn":

                Debug.Log("In right turn!");

                // Compute the next point to travel to
                next_ind =  current_angles[90f];
                next_pos = new Vector3(get_x_pos(next_ind.Item1, terrain_info), 0, get_z_pos(next_ind.Item2, terrain_info));
                next_direction = (next_pos - current_pos) . normalized;

                // Add points between the next point, the current point, and the previous point
                new_point = current_pos + 2*next_direction - 6*current_direction;
                new_point2 = current_pos + 6*next_direction - 2*current_direction;

                if(my_path.Contains(new_point)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point);

                if(my_path.Contains(new_point2)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point2);

                // Reorder the positions
                previous_pos = current_pos;
                current_pos = next_pos;
                current_ind = next_ind;

                break;

                case "forward_drive":

                Debug.Log("In forward drive!");

                // Compute the next point to travel to
                next_ind =  current_angles[0f];
                next_pos = new Vector3(get_x_pos(next_ind.Item1, terrain_info), 0, get_z_pos(next_ind.Item2, terrain_info));
                next_direction = (next_pos - current_pos) . normalized;
                perp_direction = Vector3.Cross(next_direction, Vector3.down); // To the right of...
                //Debug.Log("Perp direction:" + perp_direction);

                // Add points between the next point, the current point, and the previous point
                new_point = current_pos + 2*perp_direction;
                
                if(my_path.Contains(new_point)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point);

                // Reorder the positions
                previous_pos = current_pos;
                current_pos = next_pos;
                current_ind = next_ind;

                break;

                case "left_turn":

                Debug.Log("In left turn!");

                // Compute the next point to travel to
                next_ind =  current_angles[-90f];
                next_pos = new Vector3(get_x_pos(next_ind.Item1, terrain_info), 0, get_z_pos(next_ind.Item2, terrain_info));
                next_direction = (next_pos - current_pos) . normalized;

                // Add points on the outside of the next point, the current point, and the previous point
                new_point = current_pos - 2*next_direction - 2*current_direction;
                new_point2 = current_pos + 2*next_direction + 2*current_direction;
                
                if(my_path.Contains(new_point)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point);

                if(my_path.Contains(new_point2)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point2);

                // Reorder the positions
                previous_pos = current_pos;
                current_pos = next_pos;
                current_ind = next_ind;

                break;

                case "dead_end":

                Debug.Log("Dead end!");

                // Compute the next point to travel to
                next_ind =  current_angles[180f];
                next_pos = new Vector3(get_x_pos(next_ind.Item1, terrain_info), 0, get_z_pos(next_ind.Item2, terrain_info));
                next_direction = (next_pos - current_pos) . normalized;
                perp_direction = Vector3.Cross(next_direction, Vector3.down); // To the right of...

                // Add points on the outside of the next point, the current point, and the previous point
                new_point = current_pos - 2*perp_direction - 10*current_direction;
                new_point2 = current_pos - 6*perp_direction - 6*current_direction;
                new_point3 = current_pos - 6*perp_direction - 2*current_direction;
                new_point4 = current_pos - 2*perp_direction + 2*current_direction;
                new_point5 = current_pos + 2*perp_direction + 2*current_direction;
                new_point6 = current_pos + 6*perp_direction - 2*current_direction;
                new_point7 = current_pos + 6*perp_direction - 6*current_direction;
                new_point8 = current_pos + 2*perp_direction - 10*current_direction;
                
                if(my_path.Contains(new_point)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point);
                
                if(my_path.Contains(new_point2)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point2);

                if(my_path.Contains(new_point3)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point3);

                if(my_path.Contains(new_point4)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point4);

                if(my_path.Contains(new_point5)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point5);

                if(my_path.Contains(new_point6)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point6);

                if(my_path.Contains(new_point7)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point7);

                if(my_path.Contains(new_point8)){

                    Debug.Log("Duplicate found");
                    circumnavigating = false;

                }

                else my_path.Add(new_point8);

                // Reorder the positions
                previous_pos = current_pos;
                current_pos = next_pos;
                current_ind = next_ind;
                
                break;

            }

        }

        return my_path;

    }

    public HashSet<List<Vector3>> maximum_cover_set(){

        // Get info about the terrain
        var terrain_info = this.get_terrain_data();
        var graph_nodes = this.embedding.Keys;

        // Declare the full maximum cover set and a set for the current cover being expanded
        HashSet<List<Vector3>> mcs = new HashSet<List<Vector3>>();

        // Also create an occupancy set for explored nodes
        List<Vector3> explored_nodes = new List<Vector3>();

        // Declaration of first point in every cover
        Tuple<int, int> initial_indices = new Tuple<int, int>(0, 0);
        Vector3 pos;

        while(explored_nodes.Count < this.embedding.Keys.Count){

            List<Vector3> current_cover = new List<Vector3>();

            // Get an unexplored point from the graph
            foreach(Tuple<int, int> indices in graph_nodes){

                pos = new Vector3(get_x_pos(indices.Item1, terrain_info), 0, get_z_pos(indices.Item2, terrain_info));

                if(!explored_nodes.Contains(pos)){

                    initial_indices = indices;

                }

            }

            // Get the neighbours of this node
            List<Tuple<int, int>> neighbours = this.embedding[initial_indices];

            // Transform node to point
            Vector3 initial_pos = new Vector3(get_x_pos(initial_indices.Item1, terrain_info), 0, get_z_pos(initial_indices.Item2, terrain_info));
            current_cover.Add(initial_pos);
            explored_nodes.Add(initial_pos);

            // Expansion
            // Bools
            bool expansion = true;
            bool expand_right = true;
            bool expand_down = true;
            bool expand_left = true;
            bool expand_up = true;

            // Boundary of the rectangle
            List<Vector3> current_rightmost = new List<Vector3>();
            List<Vector3> current_lowest = new List<Vector3>();
            List<Vector3> current_leftmost = new List<Vector3>();
            List<Vector3> current_uppermost = new List<Vector3>();
            current_rightmost.Add(initial_pos);
            current_lowest.Add(initial_pos);
            current_leftmost.Add(initial_pos);
            current_uppermost.Add(initial_pos);

            while(expansion){

                List<Vector3> candidates = new List<Vector3>();

                if(expand_right){

                    foreach(Vector3 point in current_rightmost){

                        Tuple<int, int> point_indices = new Tuple<int, int>(get_i_index(point.x, terrain_info), get_j_index(point.z, terrain_info));
                        neighbours = this.embedding[point_indices];
                        Vector3 candidate = point + 20*Vector3.right;
                        Tuple<int, int> cand_indices = new Tuple<int, int>(get_i_index(candidate.x, terrain_info), get_j_index(candidate.z, terrain_info));
                        
                        if(neighbours.Contains(cand_indices)){

                            candidates.Add(candidate);

                        }

                        else {

                            Debug.Log("Can't expand to the right");
                            expand_right = false;
                            break; // The next if-statement will be false because atleast one candidate wasn't added
                            
                        }

                    }
                    
                    if(candidates.Count == current_rightmost.Count){

                        Debug.Log("Expanding to the right");
                        current_cover.AddRange(candidates);

                        // Avoid addding duplicates
                        foreach(Vector3 cand in candidates){

                            if(!explored_nodes.Contains(cand)){

                                explored_nodes.Add(cand);

                            }

                        }

                        current_rightmost = candidates;
                        current_uppermost.Add(candidates[0]);
                        current_lowest.Insert(0, candidates[candidates.Count - 1]);

                    }

                }

                candidates = new List<Vector3>();

                if(expand_down){

                    foreach(Vector3 point in current_lowest){

                        Tuple<int, int> point_indices = new Tuple<int, int>(get_i_index(point.x, terrain_info), get_j_index(point.z, terrain_info));
                        neighbours = this.embedding[point_indices];
                        Vector3 candidate = point + 20*Vector3.back;
                        Tuple<int, int> cand_indices = new Tuple<int, int>(get_i_index(candidate.x, terrain_info), get_j_index(candidate.z, terrain_info));
                        
                        if(neighbours.Contains(cand_indices)){

                            candidates.Add(candidate);

                        }

                        else {

                            Debug.Log("Can't expand downwards");
                            expand_down = false;
                            break;

                        }

                    }
                    
                    if(candidates.Count == current_lowest.Count){

                        Debug.Log("Expanding downwards");
                        current_cover.AddRange(candidates);
                        
                        // Avoid addding duplicates
                        foreach(Vector3 cand in candidates){

                            if(!explored_nodes.Contains(cand)){

                                explored_nodes.Add(cand);

                            }

                        }

                        current_lowest = candidates;
                        current_rightmost.Add(candidates[0]);
                        current_leftmost.Insert(0, candidates[candidates.Count - 1]);

                    }

                }

                candidates = new List<Vector3>();

                if(expand_left){

                    foreach(Vector3 point in current_leftmost){

                        Tuple<int, int> point_indices = new Tuple<int, int>(get_i_index(point.x, terrain_info), get_j_index(point.z, terrain_info));
                        neighbours = this.embedding[point_indices];
                        Vector3 candidate = point + 20*Vector3.left;
                        Tuple<int, int> cand_indices = new Tuple<int, int>(get_i_index(candidate.x, terrain_info), get_j_index(candidate.z, terrain_info));
                        
                        if(neighbours.Contains(cand_indices)){

                            candidates.Add(candidate);

                        }

                        else {

                            Debug.Log("Can't expand to the left");
                            expand_left = false;
                            break;

                        }

                    }
                    
                    if(candidates.Count == current_leftmost.Count){

                        Debug.Log("Expanding to the left");
                        current_cover.AddRange(candidates);
                        
                        // Avoid addding duplicates
                        foreach(Vector3 cand in candidates){

                            if(!explored_nodes.Contains(cand)){

                                explored_nodes.Add(cand);

                            }

                        }

                        current_leftmost = candidates;
                        current_lowest.Add(candidates[0]);
                        current_uppermost.Insert(0, candidates[candidates.Count - 1]);

                    }

                }

                candidates = new List<Vector3>();

                if(expand_up){

                    foreach(Vector3 point in current_uppermost){

                        Tuple<int, int> point_indices = new Tuple<int, int>(get_i_index(point.x, terrain_info), get_j_index(point.z, terrain_info));
                        neighbours = this.embedding[point_indices];
                        Vector3 candidate = point + 20*Vector3.forward;
                        Tuple<int, int> cand_indices = new Tuple<int, int>(get_i_index(candidate.x, terrain_info), get_j_index(candidate.z, terrain_info));
                        
                        if(neighbours.Contains(cand_indices)){

                            candidates.Add(candidate);

                        }

                        else {

                            Debug.Log("Can't expand up");
                            expand_up = false;
                            break;

                        }

                    }
                    
                    if(candidates.Count == current_uppermost.Count){

                        Debug.Log("Expanding up");
                        current_cover.AddRange(candidates);
                        
                        // Avoid addding duplicates
                        foreach(Vector3 cand in candidates){

                            if(!explored_nodes.Contains(cand)){

                                explored_nodes.Add(cand);

                            }

                        }

                        current_uppermost = candidates;
                        current_leftmost.Add(candidates[0]);
                        current_rightmost.Insert(0, candidates[candidates.Count - 1]);

                    }

                }

                if(!expand_right && !expand_down && !expand_left && !expand_up){

                    expansion = false;

                }

            }

            mcs.Add(current_cover);

        }

        Debug.Log("Total amount of nodes: " + this.embedding.Keys.Count);
        Debug.Log("Explored nodes: " + explored_nodes.Count);

        return mcs;

    }

    */
}


