using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class PathPlanner{


    public List<Vector3> plan_path(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info){

        // create empty path
        List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> node_path = new List<Tuple<int, int>>();

        var coords = embedding.get_terrain_coordinates_vector3();

        var path_results = a_star(start_pos, goal_pos, embedding, terrain_info);
        var reached_goal = path_results.Item2;
        var paths = path_results.Item1;

        path = paths.Item1;
        node_path = paths.Item2;        

        return path;
    }

    public int get_path_turns(List<Vector3> path){
        int turns = 0;
        if(path.Count < 3){
            return turns;
        }
        // get 3 points, if manhattan distance is equal to euclidean there is no turn
        // otherwise, there has been a turn
        for(int i=0; i<path.Count-2; i++){
            var euclidean_distance = Vector3.Distance(path[i], path[i+2]);
            var manhattan_distance = Mathf.Abs(path[i][0] - path[i+2][0]) + Mathf.Abs(path[i][2]- path[i+2][2]);
            if (euclidean_distance != manhattan_distance){
                turns++;
            }
        }
        return turns;
    }

    private Tuple<bool, List<double>> get_path_walks(GraphEmbedding emb, List<Vector3> path, bool reached_goal){

        var walks = new List<double>();

        for(int i=0; i < path.Count; i++){
            walks.Add(path.Count-i);
        }

        return new Tuple<bool, List<double>>(reached_goal, walks);
    }

    public double get_path_distance(GraphEmbedding emb, List<Vector3> path){

        double distance = 0.0;
        var start_node = path[0];
        foreach(var cord in path){
            distance = distance + emb.distance(start_node[0], cord[0], start_node[2], cord[2]);
            start_node = cord;
        }
        return distance;
    }


    
    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> a_star(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info){
        Tuple<int,int> start_node = new Tuple<int,int>(embedding.get_i_index(start_pos[0], terrain_info), 
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int,int> goal_node = new Tuple<int,int>(embedding.get_i_index(goal_pos[0], terrain_info), 
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        //List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();

        SortedList<Tuple<int,int>,float> openset=new SortedList<Tuple<int,int>,float>();

        List<Tuple<int, int>> to_visit = new List<Tuple<int, int>>();
        
        Tuple<int,int> current_node;
        Dictionary<Tuple<int,int>, Tuple<int,int>> parent = new Dictionary<Tuple<int,int>, Tuple<int,int>>();
        Dictionary<Tuple<int,int>, float> g = new Dictionary<Tuple<int,int>, float>();
        Dictionary<Tuple<int,int>, bool> in_queue = new Dictionary<Tuple<int,int>, bool>();

        List<Vector3> path=new List<Vector3>();
        List<Tuple<int,int>> node_path=new List<Tuple<int,int>>();

        //path.Add(start_pos);

        openset.Add(start_node,(float)embedding.h(start_node,goal_node));
        in_queue.Add(start_node,true);
        g.Add(start_node,0F);
        parent.Add(start_node,new Tuple<int,int>(-1,-1)); // root node of path

        float tentative_g;

        while(openset.Count>0){
            current_node=openset.Keys[0];
            openset.Remove(current_node);
            in_queue[current_node]=false;
            
            if(current_node==goal_node){
                break; // and reconstruct path
            }

            var adjacent_nodes = embedding.embedding[current_node];

            foreach(Tuple<int,int> node_ in adjacent_nodes){
                tentative_g=g[current_node]+(float)embedding.node_distance(current_node,node_);

                if(!(g.ContainsKey(node_))||(tentative_g<g[node_])){
                    
                    if(parent.ContainsKey(node_)){
                        parent[node_]=current_node;}
                    else{parent.Add(node_,current_node);}
                    
                    if(g.ContainsKey(node_)){
                        g[node_]=tentative_g;}
                    else{g.Add(node_,tentative_g);}

                    if(!(in_queue.ContainsKey(node_)) || in_queue[node_]==false){
                        if(in_queue.ContainsKey(node_)){
                        in_queue[node_]=true;
                        openset.Add(node_,g[node_]+(float)embedding.h(node_,goal_node));}
                    else{in_queue.Add(node_,true);
                        openset.Add(node_,g[node_]+(float)embedding.h(node_,goal_node));}
                    }
                }
            }

        }

        if(parent.ContainsKey(goal_node)){
            current_node=goal_node;
            node_path.Add(current_node);
            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
            while(parent[current_node].Item1!=-1 || parent[current_node].Item2!=-1){
                current_node=parent[current_node];
                node_path.Add(current_node);
                path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
                
            }

            path.Reverse();
            node_path.Reverse();
            var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);
        }
        else{
            Debug.Log("Path not found, no adjacent nodes.");
            var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
        }

    }


    private static Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool> bfs(Vector3 start_pos, Vector3 goal_pos, GraphEmbedding embedding, TerrainInfo terrain_info){
        Tuple<int,int> start_node = new Tuple<int,int>(embedding.get_i_index(start_pos[0], terrain_info), 
                                                       embedding.get_j_index(start_pos[2], terrain_info)
                                                       );
        Tuple<int,int> goal_node = new Tuple<int,int>(embedding.get_i_index(goal_pos[0], terrain_info), 
                                                      embedding.get_j_index(goal_pos[2], terrain_info)
                                                      );

        //List<Vector3> path = new List<Vector3>();
        List<Tuple<int, int>> visited_nodes = new List<Tuple<int, int>>();
        List<Tuple<int, int>> to_visit = new List<Tuple<int, int>>();
        List<Vector3> path=new List<Vector3>();
        List<Tuple<int,int>> node_path=new List<Tuple<int,int>>();
        Tuple<int,int> current_node;
        Dictionary<Tuple<int,int>, Tuple<int,int>> parent = new Dictionary<Tuple<int,int>, Tuple<int,int>>();

        //path.Add(start_pos);
        visited_nodes.Add(start_node);
        parent.Add(start_node,new Tuple<int,int>(-1,-1)); // root node of path

        foreach(Tuple<int,int> node in embedding.embedding[start_node]){
            to_visit.Add(node);
            parent.Add(node,start_node);
            visited_nodes.Add(node);
        }

        while(!(parent.ContainsKey(goal_node)) && to_visit.Count>0 ){
            current_node=to_visit[0];
            to_visit.RemoveAt(0);

            var adjacent_nodes = embedding.embedding[current_node];

            // remove visited
            adjacent_nodes.RemoveAll(l => visited_nodes.Contains(l));

            foreach(Tuple<int,int> node_ in adjacent_nodes){
                to_visit.Add(node_);
                parent.Add(node_,current_node);
                visited_nodes.Add(node_);
            }

        }

        if(parent.ContainsKey(goal_node)){
            current_node=goal_node;
            node_path.Add(current_node);
            path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
            while(parent[current_node].Item1!=-1 || parent[current_node].Item2!=-1){
                current_node=parent[current_node];
                node_path.Add(current_node);
                path.Add(new Vector3(embedding.get_x_pos(current_node.Item1, terrain_info), 0, embedding.get_z_pos(current_node.Item2, terrain_info)));
                
            }

            path.Reverse();
            node_path.Reverse();
            var paths_finished = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_finished, true);
        }
        else{
            Debug.Log("Path not found, no adjacent nodes.");
            var paths_no_adj = new Tuple<List<Vector3>, List<Tuple<int, int>>>(path, node_path);
            return new Tuple<Tuple<List<Vector3>, List<Tuple<int, int>>>, bool>(paths_no_adj, false);
        }

    }

}