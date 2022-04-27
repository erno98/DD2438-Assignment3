using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

// much of the code just copied from Ernest's GraphEmbedding script (or modified etc.)

public class Line // class representing lines with physical coordinates

{
    public Vector2 start;
    public Vector2 end;

    public Line(Vector2 start_,Vector2 end_)
    {
        this.start=start_;
        this.end=end_;
    }

    public Vector2 position(float t) // parameterize the line
    {
        return this.start*(1-t)+this.end*t;
    }

    public float distance_from(Vector2 p) // distance of point from the line determined by the two points
    {
        Vector2 p_=p-this.start; 

        Vector2 v=(this.end-this.start).normalized; // normalized directional vector of the line

        Vector2 p_parallel=v*(Vector2.Dot(v,p_));
        Vector2 p_perpendicular=p_-p_parallel;

        return p_perpendicular.magnitude;

    }

}

// class to manage line inside the terrain

public class EmbeddedLine
{
    public Line line;
    public float[,] terrain_matrix;
    public float x_low, x_high, z_low, z_high,padding;
    public int x_n, z_n;
    public string _terrain_filename;


    // initialize data members, load terrain info

    public EmbeddedLine(Line l, float padding, string terrain_filename)
    {
        this.line=l;
        this._terrain_filename=terrain_filename;

        // read the json file
        var jsonTextFile = Resources.Load<TextAsset>(this._terrain_filename);
        var json_handle= TerrainInfo.CreateFromJSON(jsonTextFile.text);

        // load the traversability matrix, and map size info
        // assume that respective low vlaues are indeed lower than the high values

        this.terrain_matrix = json_handle.traversability; 
        this.x_low = json_handle.x_low;
        this.x_high = json_handle.x_high;
        this.z_low = json_handle.z_low;
        this.z_high = json_handle.z_high;
        this.x_n = json_handle.x_N;
        this.z_n = json_handle.z_N;
        this.padding=padding;
    }

    // convert indices to top and bottom cordinates of cells

    public float x_bottom_from_idx(int idx)
    {
        return this.x_low-this.padding+(this.x_high-this.x_low)/(float)this.x_n*(float)idx;
    }

    public float x_top_from_idx(int idx)
    {
        return this.x_low+this.padding+(this.x_high-this.x_low)/(float)this.x_n*(float)(idx+1);
    }
   
    public float z_bottom_from_idx(int idx)
    {
        return this.z_low-this.padding+(this.z_high-this.z_low)/(float)this.z_n*(float)idx;
    }

    public float z_top_from_idx(int idx)
    {
        return this.z_low+this.padding+(this.z_high-this.z_low)/(float)this.z_n*(float)(idx+1);
    }

    // see if the stored line intersects the given cell

    public bool intersects_cell(int x, int z) 
    {
        // the idea is to check if there exist such a point on the line what is inside the cell

        // note that here - regarding line and global coordinates - we stick to x (line) - x (terrain), y(line) - z (terrain) convention
 
        float dx = this.line.end.x-this.line.start.x;
        float dz = this.line.end.y-this.line.start.y;

        bool intersection=false;
        float upper_limit=1F;
        float lower_limit=0F;
        float temp_lower, temp_upper;

        // manage x inequalities (for t)

        if(dx>0) // no need to rearrange
        {
            temp_lower=(this.x_bottom_from_idx(x)-this.line.start.x)/dx;
            if(temp_lower>lower_limit)
            {lower_limit=temp_lower;}

            temp_upper=(this.x_top_from_idx(x)-this.line.start.x)/dx;
            if(temp_upper<upper_limit)
            {upper_limit=temp_upper;}
        }
        else if (dx<0)
        {
            temp_upper=(this.x_bottom_from_idx(x)-this.line.start.x)/dx;
            if(temp_upper<upper_limit)
            {upper_limit=temp_upper;}

            temp_lower=(this.x_top_from_idx(x)-this.line.start.x)/dx;
            if(temp_lower>lower_limit)
            {lower_limit=temp_lower;}
        }
        else  // then dx==0
        {
            if (this.line.start.x<this.x_bottom_from_idx(x) | this.line.start.x>this.x_top_from_idx(x))
            {
                return false;
            }
        }

        // do the same for z 

         if(dz>0) // no need to rearrange
        {
            temp_lower=(this.z_bottom_from_idx(z)-this.line.start.y)/dz; // watch out for y in the line arguments
            if(temp_lower>lower_limit)
            {lower_limit=temp_lower;}

            temp_upper=(this.z_top_from_idx(z)-this.line.start.y)/dz;
            if(temp_upper<upper_limit)
            {upper_limit=temp_upper;}
        }
        else if (dz<0)
        {
            temp_upper=(this.z_bottom_from_idx(z)-this.line.start.y)/dz;
            if(temp_upper<upper_limit)
            {upper_limit=temp_upper;}

            temp_lower=(this.z_top_from_idx(z)-this.line.start.y)/dz;
            if(temp_lower>lower_limit)
            {lower_limit=temp_lower;}
        }
        else  // then dz==0
        {
            if (this.line.start.y<this.z_bottom_from_idx(z) | this.line.start.y>this.z_top_from_idx(z))
            {
                return false;
            }
        }

        if (lower_limit<upper_limit) // there is a solution, there is intersection
        {
            intersection=true;
        }
        return intersection;
    }
   
     public bool intersects_non_transversable()
        {
            // TODO use a less lazy method, eg check cells barely from bounding box

            for(int x=0; x<this.x_n; x++)
            {
                for(int z=0; z<this.z_n; z++)
                {
                    if(this.intersects_cell(x,z) & this.terrain_matrix[x,z]>0.5F) // hit this cell which is blocked
                    {return true;}
                }
            }
            return false;
        }
}

public class waypoint_list
{
    public List<Vector2> waypoints; // 2D vectors as usual

    public waypoint_list(List<Vector2> waypoints)
    {
        this.waypoints=waypoints;
    }

    public waypoint_list(List<Vector3> waypoints) 
    {
        this.waypoints=new List<Vector2>();
        for(int i=0;i<waypoints.Count; i++){
            this.waypoints.Add(new Vector2(waypoints[i].x,waypoints[i].z));
        }
    }

    public waypoint_list(waypoint_list other) // copy constructor
    {
        this.waypoints=new List<Vector2>();
        for(int i=0;i<other.waypoints.Count; i++){
            this.waypoints.Add(new Vector2(other.waypoints[i].x,other.waypoints[i].y));
        }
    }

    public float arc_length(int i, int j) // arc len between two waypoints, assume i<j
    {
        float _len=0;

        for(int k=i+1;k<=j;k++){
            _len+=(this.waypoints[k]-this.waypoints[k-1]).magnitude;
        }

        return _len;
    }

    public waypoint_list double_resolution() // meant to increatse the resolution of the path, JUST BEFORE driving, adding intermediate waypoints on line segments
    {
        List<Vector2> new_wps=new List<Vector2>();

        new_wps.Add(new Vector2(this.waypoints[0].x,this.waypoints[0].y));

        for(int i=1;i<this.waypoints.Count; i++){
            new_wps.Add(new Vector2((this.waypoints[i-1].x+this.waypoints[i].x)*0.5F,(this.waypoints[i-1].y+this.waypoints[i].y)*0.5F));
            new_wps.Add(new Vector2(this.waypoints[i].x,this.waypoints[i].y));
        }

        return new waypoint_list(new_wps);
    }

    public float max_segment_length()
    {
        float max_len=0F;
        float current_len=0F;

        for(int i=1;i<this.waypoints.Count; i++){
            current_len=(this.waypoints[i]-this.waypoints[i-1]).magnitude;
            if(max_len<current_len){
                max_len=current_len;
            }
        }
        return max_len;
    }

}

public class polygon_path // class represnting a polygonal path
{
    public waypoint_list wp; // 2D vectors as usual
    public List<float> speed_limit;
   
    public string _terrain_filename;
    public float padding;
    public int last_visited; // meant to log the waypoint that has just been used as lookahead to disregard earlier points (AHEAD)

    public polygon_path(waypoint_list waypoints, float padding, string _terrain_filename)
    {
        this.wp= new waypoint_list(waypoints);
        this.last_visited=0;
        this.padding=padding;
        this._terrain_filename=_terrain_filename;
        this.speed_limit=new List<float>();
    }

    public polygon_path(polygon_path other) // copy constructor, DOESN't copy speed limits
    {
        this.wp=new waypoint_list(other.wp);
        this.speed_limit=new List<float>();
        this._terrain_filename=other._terrain_filename;
        this.last_visited=0;
        this.padding=other.padding;
    }

    public void set_speed_limit(float friction_coeff) // assign maximal recommended speed to waypoints - make 'point mass' drivable
    {
        int n=this.wp.waypoints.Count;
        float angle,tangent,radius,velocity;

        List<float> limit=new List<float>();

        limit.Add(1000000F); // first point

        float tangent_lookahead;
        float leg1,leg2;


        for(int i=1;i<n-1;i++)
        {
            angle=Vector2.Angle(this.wp.waypoints[i]-this.wp.waypoints[i-1],this.wp.waypoints[i+1]-this.wp.waypoints[i]);
            tangent=Mathf.Tan(angle/2/180*Mathf.PI);

            leg1=(this.wp.waypoints[i]-this.wp.waypoints[i-1]).magnitude/2F;
            leg2=(this.wp.waypoints[i+1]-this.wp.waypoints[i]).magnitude/2F;

            tangent_lookahead=Mathf.Min(leg1,leg2);

            // all this elseif testing is to prevent overflow & stuff

            
            if(tangent>0.0000001F & tangent<1000000F){
                radius=tangent_lookahead/tangent;
            }
            else if(tangent>1000000F){
                radius=0.000001F;
            }
            else{
                radius=100000F;
            }

            velocity=Mathf.Sqrt(radius*9.82F*friction_coeff); // point mass model

            limit.Add(velocity);
        }

        limit.Add(1000000F); // last point

        this.speed_limit=limit;

    }

    public void set_speed_limit_drone() // assign maximal recommended speed to waypoints - make 'point mass' drivable
    {
        int n=this.wp.waypoints.Count;
        float angle,tangent,radius,velocity;

        List<float> limit=new List<float>();

        limit.Add(1000000F); // first point

        float tangent_lookahead;
        float leg1,leg2;


        for(int i=1;i<n-1;i++)
        {
            angle=Vector2.Angle(this.wp.waypoints[i]-this.wp.waypoints[i-1],this.wp.waypoints[i+1]-this.wp.waypoints[i]);
            tangent=Mathf.Tan(angle/2/180*Mathf.PI);

            leg1=(this.wp.waypoints[i]-this.wp.waypoints[i-1]).magnitude/2F;
            leg2=(this.wp.waypoints[i+1]-this.wp.waypoints[i]).magnitude/2F;

            tangent_lookahead=Mathf.Min(leg1,leg2);

            // all this elseif testing is to prevent overflow & stuff

            
            if(tangent>0.0000001F & tangent<1000000F){
                radius=tangent_lookahead/tangent;
            }
            else if(tangent>1000000F){
                radius=0.000001F;
            }
            else{
                radius=100000F;
            }

            velocity=Mathf.Sqrt(radius*15F); // point mass model

            limit.Add(velocity);
        }

        limit.Add(1000000F); // last point

        this.speed_limit=limit;

    }

    public void smooth_speed_limit(float friction_coeff) 
    {
        int n=this.wp.waypoints.Count;
        float a_max=9.82F*friction_coeff;

        for(int i=n-2;i>0;i--) // endpoint
        {
            for(int j=0;j<i;j++)
            {
                if(this.speed_limit[j]>Mathf.Sqrt(this.speed_limit[i]*this.speed_limit[i]+2F*this.wp.arc_length(j,i)*a_max)){ // what is physically possible
                    this.speed_limit[j]=Mathf.Sqrt(this.speed_limit[i]*this.speed_limit[i]+2F*this.wp.arc_length(j,i)*a_max); 
                }
            }
        }

    }

        public void smooth_speed_limit_drone() 
    {
        int n=this.wp.waypoints.Count;
        float a_max=15F;

        for(int i=n-2;i>0;i--) // endpoint
        {
            for(int j=0;j<i;j++)
            {
                if(this.speed_limit[j]>Mathf.Sqrt(this.speed_limit[i]*this.speed_limit[i]+2F*this.wp.arc_length(j,i)*a_max)){ // what is physically possible
                    this.speed_limit[j]=Mathf.Sqrt(this.speed_limit[i]*this.speed_limit[i]+2F*this.wp.arc_length(j,i)*a_max); 
                }
            }
        }

    }



    public polygon_path linear_smoothing(float max_deviation) // returns polygon path with waypoints being 'deep copied'
    {
        List<Vector2> smoothed = new List<Vector2>();

        int n=this.wp.waypoints.Count;

        int start=0;

        Debug.Log("********************************************************************************************************");

        smoothed.Add(new Vector2(this.wp.waypoints[start].x,this.wp.waypoints[start].y));

        for(int end=1; end<n; end++){
            EmbeddedLine el = new EmbeddedLine(new Line(this.wp.waypoints[start],this.wp.waypoints[end]),this.padding,this._terrain_filename);


            float max_distance=0F;
            float temp_d=0F;

            for(int interim_wp=start;interim_wp<=end;interim_wp++)
            {
                temp_d=el.line.distance_from(this.wp.waypoints[interim_wp]);
                if(temp_d>max_distance){
                    max_distance=temp_d;
                }

            }

            if(el.intersects_non_transversable() | max_distance>max_deviation){
                Debug.Log("max distance: +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
                Debug.Log(max_distance);
                //Debug.Log("start: "+this.wp.waypoints[start]);
                //Debug.Log("end: "+this.wp.waypoints[end]);
                start=end-1;
                smoothed.Add(new Vector2(this.wp.waypoints[start].x,this.wp.waypoints[start].y));
                
            }
        }

        smoothed.Add(new Vector2(this.wp.waypoints[n-1].x,this.wp.waypoints[n-1].y));

        return new polygon_path(new waypoint_list(smoothed),this.padding,this._terrain_filename);
    }

    public float find_speed_limit2d(Vector2 pos, float lookahead_distance){
        
        float min_deviation,temp_dev;
        int min_index;

        min_deviation=Math.Abs((this.wp.waypoints[this.last_visited]-pos).magnitude-lookahead_distance);
        min_index=this.last_visited;

        for(int i=this.last_visited;i<this.wp.waypoints.Count;i++){
            temp_dev=Math.Abs((this.wp.waypoints[i]-pos).magnitude-lookahead_distance);
            if(temp_dev<min_deviation){
                min_deviation=temp_dev;
                min_index=i;
            }
        }

        return this.speed_limit[min_index];
        }

      public bool is_point_blocked(Vector2 point, Vector2 position,string terrain_filename){
        Line l=new Line(position,point);
        EmbeddedLine el=new EmbeddedLine(l,this.padding,terrain_filename); // MAYBE NOT NEEDED????

        return el.intersects_non_transversable();
      }

    

      public Vector2 find_lookahead_point(Vector2 pos, float lookahead_distance) // find the waypoint whose distance from pos is closest to 
    {
        float min_deviation,temp_dev;
        int min_index;

        min_deviation=Math.Abs((this.wp.waypoints[this.last_visited]-pos).magnitude-lookahead_distance);
        min_index=this.last_visited;

        for(int i=this.last_visited;i<this.wp.waypoints.Count;i++){
            temp_dev=Math.Abs((this.wp.waypoints[i]-pos).magnitude-lookahead_distance);
            if(temp_dev<min_deviation && !(this.is_point_blocked(this.wp.waypoints[i],pos,this._terrain_filename))){
                min_deviation=temp_dev;
                min_index=i;
            }
        }

        this.last_visited=min_index; // for later searches
        return this.wp.waypoints[min_index];
    }

    public Vector2 find_closest_point(Vector2 pos){
        
        float min_deviation,temp_dev;
        int min_index;

        min_deviation=(this.wp.waypoints[0]-pos).magnitude;
        min_index=0;

        for(int i=0;i<this.wp.waypoints.Count;i++){
            temp_dev=(this.wp.waypoints[i]-pos).magnitude;
            if(temp_dev<min_deviation){
                min_deviation=temp_dev;
                min_index=i;
            }
        }

        return this.wp.waypoints[min_index];
    }

     public Vector2 find_closest_point_stuck(Vector2 pos){
        
        float min_deviation,temp_dev;
        int min_index;

        min_deviation=(this.wp.waypoints[0]-pos).magnitude;
        min_index=0;

        for(int i=0;i<this.wp.waypoints.Count;i++){
            temp_dev=(this.wp.waypoints[i]-pos).magnitude;
            if(temp_dev<min_deviation){
                min_deviation=temp_dev;
                min_index=i;
            }
        }

        

        return this.wp.waypoints[Math.Max(min_index-3,0)];
    }


    public int find_closest_idx(Vector2 pos){
        
        float min_deviation,temp_dev;
        int min_index;

        min_deviation=(this.wp.waypoints[0]-pos).magnitude;
        min_index=0;

        for(int i=0;i<this.wp.waypoints.Count;i++){
            temp_dev=(this.wp.waypoints[i]-pos).magnitude;
            if(temp_dev<min_deviation){
                min_deviation=temp_dev;
                min_index=i;
            }
        }

        return min_index;
    }

    public Vector2 desired_driving_direction(Vector2 pos)
    {
        int c_idx=find_closest_idx(pos);

        if(c_idx==this.wp.waypoints.Count-1){
            return new Vector2(0,0);
        }
        
        return (this.wp.waypoints[c_idx+1]-this.wp.waypoints[c_idx])/(this.wp.waypoints[c_idx+1]-this.wp.waypoints[c_idx]).magnitude;
    }

    public Vector3 desired_driving_direction_3d(Vector3 pos)
    {
        Vector2 _pos=new Vector2(pos.x,pos.z);
        Vector2 _dir=this.desired_driving_direction(_pos);

        return new Vector3(_dir.x,0,_dir.y);
    }

    public Vector3 find_closest_point3d(Vector3 pos){
        
        Vector2 _pos=new Vector2(pos.x,pos.z);
        Vector2 _closest=this.find_closest_point(_pos);

        return new Vector3(_closest.x,0,_closest.y);
    }

     public Vector3 find_closest_point3d_stuck(Vector3 pos){
        
        Vector2 _pos=new Vector2(pos.x,pos.z);
        Vector2 _closest=this.find_closest_point_stuck(_pos);

        return new Vector3(_closest.x,0,_closest.y);
    }


    public int get_last_visited(){
        return this.last_visited;
    }

    public void double_resolution(){
        this.wp=this.wp.double_resolution();

        List<float> new_speed_limit=new List<float>();

        for(int i=0;i<this.speed_limit.Count-1;i++){
            new_speed_limit.Add(this.speed_limit[i]);
            new_speed_limit.Add(100000F);
        }

        new_speed_limit.Add(this.speed_limit[this.speed_limit.Count-1]);

        this.speed_limit=new_speed_limit;
    }

    public Vector3 lookahead_3d(Vector3 pos, float lookahead_distance){
        Vector2 _pos=new Vector2(pos.x,pos.z);
        Vector2 _lookahead=this.find_lookahead_point(_pos,lookahead_distance);

        return new Vector3(_lookahead.x,0,_lookahead.y);
    }

     public float find_speed_limit3d(Vector3 pos, float lookahead_distance){
        Vector2 _pos=new Vector2(pos.x,pos.z);

        return this.find_speed_limit2d(_pos,lookahead_distance);
    }

}

