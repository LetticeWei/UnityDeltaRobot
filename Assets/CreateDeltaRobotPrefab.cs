using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class DeltaRobotModel{
    public float rb, l, rou, re,temp_y,temp_theta, tilt_ang;
    public float bar_thickness;
    public Vector3 s0,s1,s2;public Vector3[] s_l;
    public DeltaRobotModel(float base_radius, float upper_arm_length, float lower_arm_length, float end_radius){
        this.temp_theta=Mathf.PI*0.25f;
        this.rb=base_radius;
        this.l=upper_arm_length;
        this.rou=lower_arm_length;
        this.re=end_radius;
        this.bar_thickness = 0.3f;

        this.s0=new Vector3(1f,0,0);
        this.s1=new Vector3(-1f/2f,0,-1f/2f*Mathf.Sqrt(3f));
        this.s2=new Vector3(-1f/2f,0,1f/2f*Mathf.Sqrt(3f));
        this.s_l=new Vector3[]{s0,s1,s2};
        this.Calculate_temp_vars();
        // this.temp_y=-10.63186143f;
        this.CreateBase();
        this.CreatePlatform();
        this.CreateElbowBar();
        
    }       

    public void Calculate_temp_vars(){
        float end_offset = this.l * Mathf.Cos(this.temp_theta) + this.rb-this.re;
        float height_offset= Mathf.Sqrt( this.rou*this.rou - end_offset*end_offset);
        this.temp_y= -1f*(height_offset+ this.l * Mathf.Sin(this.temp_theta));
        this.tilt_ang=90f-Mathf.Acos(end_offset/this.rou)/Mathf.PI *180f;    //converted to degree!
    }
    public void CreateBase(){
        float t= this.bar_thickness;
        GameObject[] base_bars= new GameObject[3];
        for (int i = 0; i < 3; i++){
            base_bars[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            base_bars[i].transform.Rotate(0f, 120f*(float)i, 0f, Space.Self);
            base_bars[i].transform.Rotate(90f, 0f, 0f, Space.Self);
            base_bars[i].transform.localScale=new Vector3(t,this.rb*Mathf.Sqrt(3f),t);
            base_bars[i].transform.position= this.rb*s_l[i];
        }
    }
    public void CreatePlatform(){
        // float temp_y= -10.63186143f;
        float temp_y= this.temp_y;
        float t= this.bar_thickness;
        GameObject[] platform_bars= new GameObject[3];
        GameObject[] platform_balls= new GameObject[3];
        for (int i = 0; i < 3; i++){
            platform_bars[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            platform_bars[i].transform.Rotate(0f, 60f + 120f*(float)i, 0f, Space.Self);
            platform_bars[i].transform.position= this.re*(s_l[i % 3 ]+s_l[(i+1) % 3]) + new Vector3(0,temp_y,0);
            platform_bars[i].transform.Rotate(0, 0f, 90f, Space.Self);
            platform_bars[i].transform.localScale=new Vector3(t,this.re,t) ;

            platform_balls[i]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            platform_balls[i].transform.localScale*=t*2f;
            platform_balls[i].transform.position= 2*this.re*(s_l[i % 3 ]+s_l[(i+1) % 3]) + new Vector3(0,temp_y,0) ;

        }
    }
    public void CreateElbowBar(){
        float temp_crank_angle=this.temp_theta;
        float t= this.bar_thickness;
        GameObject[] elbow_balls= new GameObject[6];
        GameObject[] elbow_bars= new GameObject[3];
        for (int i = 0; i < 3; i++){
            elbow_bars[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            elbow_bars[i].transform.Rotate(0f, 120f*(float)i, 0f, Space.Self);
            elbow_bars[i].transform.Rotate(90f, 0f, 0f, Space.Self);
            elbow_bars[i].transform.localScale=new Vector3(t,this.re*Mathf.Sqrt(3f),t);
            Debug.Log(this.rb+this.l*Mathf.Cos(temp_crank_angle));
            Debug.Log(this.l*Mathf.Sin(temp_crank_angle));
            Vector3 elbow_position=(this.rb+this.l*Mathf.Cos(temp_crank_angle))*this.s_l[i]+this.l*Mathf.Sin(temp_crank_angle)*new Vector3(0,-1f,0);
            elbow_bars[i].transform.position= elbow_position;

            elbow_balls[2*i]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            elbow_balls[2*i+1]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            elbow_balls[2*i].transform.localScale*=t*2f;
            elbow_balls[2*i+1].transform.localScale*=t*2f;
            Vector3 ball_offset1 = this.re*this.s_l[(i+1)%3];
            Vector3 ball_offset2 = this.re*this.s_l[(i+2)%3];
            elbow_balls[2*i].transform.position= elbow_position+ (ball_offset1 - ball_offset2);
            elbow_balls[2*i+1].transform.position= elbow_position+ (-ball_offset1 + ball_offset2);
        }
    }
    public void CreateUpperArm(GameObject upper_arm_prefab){
        float temp_crank_angle=this.temp_theta;
        GameObject[] upper_arms= new GameObject[3];
        for (int i = 0; i < 3; i++){
            upper_arms[i]=GameObject.Instantiate(upper_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;;
            upper_arms[i].transform.Rotate(0f,90f+120f*i, 0f, Space.Self);
            upper_arms[i].transform.Rotate( 45f, 0f, 0f, Space.Self);
            upper_arms[i].transform.Rotate( 0f, 0f, 90f, Space.Self);
            Vector3 elbow_position=(this.rb+this.l*Mathf.Cos(temp_crank_angle))*this.s_l[i]+this.l*Mathf.Sin(temp_crank_angle)*new Vector3(0,-1f,0);
            Vector3 motor_position=this.rb*s_l[i];
            upper_arms[i].transform.position=0.5f*(elbow_position+motor_position);
        }
    }

    public void CreateLowerArm(GameObject lower_arm_prefab){
        // float temp_y= -10.63186143f;
        float temp_y= this.temp_y;
        float temp_crank_angle=this.temp_theta;
        GameObject[] lower_arms= new GameObject[6];
        for (int i = 0; i < 3; i++){
            lower_arms[2*i]=GameObject.Instantiate(lower_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;;
            lower_arms[2*i+1]=GameObject.Instantiate(lower_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;;
            lower_arms[2*i].transform.Rotate( 90f, 0f, 0f, Space.Self);
            lower_arms[2*i+1].transform.Rotate( 90f, 0f, 0f, Space.Self);

            if (i==1){
                lower_arms[2*i].transform.Rotate( 0, 0f,-30f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, 0f, -30f, Space.Self);
            }
            else if (i==2){
                lower_arms[2*i].transform.Rotate( 0, 0f,-150f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, 0f, -150f, Space.Self);
            }
            Debug.Log("this.tilt_ang");
            Debug.Log(this.tilt_ang);
            if (i!=0){
            lower_arms[2*i].transform.Rotate( - this.tilt_ang, 0f, 0f, Space.Self);
            lower_arms[2*i+1].transform.Rotate( -this.tilt_ang, 0f, 0f, Space.Self);}
            else{
                lower_arms[2*i].transform.Rotate( 0, -90f, -90f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, -90f, -90f, Space.Self);
                lower_arms[2*i].transform.Rotate( 90f+this.tilt_ang, 0,0, Space.Self);
                lower_arms[2*i+1].transform.Rotate(90f+this.tilt_ang, 0,0, Space.Self);
            }
            

            // lower_arms[i].transform.Rotate( 0f, 0f, 90f, Space.Self);

            Vector3 elbow_position=(this.rb+this.l*Mathf.Cos(temp_crank_angle))*this.s_l[i]+this.l*Mathf.Sin(temp_crank_angle)*new Vector3(0,-1f,0);

            Vector3 platform_mid_point_position= this.re*s_l[i]- new Vector3(0,-1f*temp_y,0);

            Vector3 lower_arm_position_simplified= 0.5f*(elbow_position+platform_mid_point_position);
            
            Vector3 offset1 = this.re*this.s_l[(i+1)%3];
            Vector3 offset2 = this.re*this.s_l[(i+2)%3];

            lower_arms[2*i].transform.position=lower_arm_position_simplified + (offset1 - offset2);
            lower_arms[2*i+1].transform.position=lower_arm_position_simplified+ (-offset1 + offset2);
        }
    }
}


public class CreateDeltaRobotPrefab : MonoBehaviour
{
    // Start is called before the first frame update
    
    public DeltaRobotModel RobotModel1;
    public float scale_fact;

    public GameObject upper_arm_5_unit_prefab;
    public GameObject lower_arm_9_unit_prefab;

    void Start()
    {   scale_fact=0.5f;
        RobotModel1=new DeltaRobotModel(3f,5f,9f,1f);
        //Instantiate(upper_arm_5_unit_prefab, new Vector3(1f, 2f, 3f), Quaternion.identity);
        RobotModel1.CreateUpperArm(upper_arm_5_unit_prefab);
        RobotModel1.CreateLowerArm(lower_arm_9_unit_prefab);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
