using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class DeltaRobotModel{
    public float rb, l, rou, re,temp_y,temp_theta, tilt_ang;
    public Vector3 overall_offset;
    public static int count = 0;  
    public float bar_thickness;
    public GameObject delta_robot_model;
    public GameObject base_model, upper_arm_set_model,elbow_set_model,lower_arm_set_model,platform_model;
    public GameObject[] elbow_model_list,lower_arm_model_list ;
    public Vector3 s0,s1,s2;public Vector3[] s_l;
    public bool use_gravity=false;
    public float upper_arm_mass, elbow_mass, lower_arm_mass, platform_mass, drag,angular_drag;
    public DeltaRobotModel(float base_radius, float upper_arm_length, float lower_arm_length, float end_radius, Vector3 assigned_offset,GameObject upper_arm_prefab, GameObject lower_arm_prefab){
        upper_arm_mass=1f;elbow_mass=0.01f;lower_arm_mass=0.01f;platform_mass=0.01f; drag=0f;angular_drag=0f;
        this.temp_theta=Mathf.PI*0.25f;
        this.rb=base_radius;
        this.l=upper_arm_length;
        this.rou=lower_arm_length;
        this.re=end_radius;
        this.bar_thickness = 0.3f;
        this.overall_offset=assigned_offset;
        this.s0=new Vector3(1f,0,0);
        this.s1=new Vector3(-1f/2f,0,-1f/2f*Mathf.Sqrt(3f));
        this.s2=new Vector3(-1f/2f,0,1f/2f*Mathf.Sqrt(3f));
        this.s_l=new Vector3[]{s0,s1,s2};
        this.Calculate_temp_vars();
        // this.temp_y=-10.63186143f;
        this.delta_robot_model=new GameObject("Delta robot Model " + count.ToString());
        this.CreateBase();
        this.CreatePlatform();
        this.CreateElbowBar();
        this.CreateUpperArm(upper_arm_prefab);
        this.CreateLowerArm(lower_arm_prefab);
        // this.delta_robot_model.AddComponent("RobotCongifurator2.cs");
        count = count + 1; 
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
        GameObject[] base_bar_links= new GameObject[3];
        this.base_model=new GameObject("Base Model");
        this.base_model.transform.SetParent(this.delta_robot_model.transform, true);
        for (int i = 0; i < 3; i++){
            base_bars[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            base_bar_links[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            base_bars[i].transform.Rotate(0f, 120f*(float)i, 0f, Space.Self);
            base_bars[i].transform.Rotate(90f, 0f, 0f, Space.Self);
            base_bar_links[i].transform.Rotate(0f, 60f+120f*(float)i, 0f, Space.Self);
            base_bar_links[i].transform.Rotate(90f, 0f, 0f, Space.Self);
            base_bars[i].transform.localScale=new Vector3(t,0.35f*this.rb*Mathf.Sqrt(3f),t);
            base_bars[i].transform.position= this.rb*s_l[i]+this.overall_offset;
            base_bar_links[i].transform.localScale=new Vector3(t,0.35f*this.rb*Mathf.Sqrt(3f),t);
            base_bar_links[i].transform.position= this.rb*(s_l[i]+s_l[(i+1)%3])+this.overall_offset;

            base_bars[i].transform.SetParent(this.base_model.transform, true);
            base_bar_links[i].transform.SetParent(this.base_model.transform, true);
        }

        Rigidbody base_rigid_body = this.base_model.AddComponent<Rigidbody>();
        this.base_model.AddComponent<FixedJoint> ();  
    }

    public void CreatePlatform(){
        // float temp_y= -10.63186143f;
        float temp_y= this.temp_y;
        float t= this.bar_thickness;
        GameObject[] platform_bars= new GameObject[3];
        GameObject[] platform_balls= new GameObject[3];
        this.platform_model=new GameObject("Platform Model");
        this.platform_model.transform.SetParent(this.delta_robot_model.transform, true);
        for (int i = 0; i < 3; i++){
            platform_bars[i]=GameObject.CreatePrimitive(PrimitiveType.Capsule);
            platform_bars[i].transform.Rotate(0f, 60f + 120f*(float)i, 0f, Space.Self);
            platform_bars[i].transform.position= this.re*(s_l[i % 3 ]+s_l[(i+1) % 3]) + new Vector3(0,temp_y,0)+this.overall_offset;
            platform_bars[i].transform.Rotate(0, 0f, 90f, Space.Self);
            platform_bars[i].transform.localScale=new Vector3(t,0.5f*t+this.re,t) ;

            platform_balls[i]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            platform_balls[i].transform.localScale*=t*2f;
            platform_balls[i].transform.position= 2*this.re*(s_l[i % 3 ]+s_l[(i+1) % 3]) + new Vector3(0,temp_y,0) +this.overall_offset;
            platform_bars[i].transform.SetParent(this.platform_model.transform, true);
            platform_balls[i].transform.SetParent(this.platform_model.transform, true);
        }
        Rigidbody platform_rigid_body = this.platform_model.AddComponent<Rigidbody>();
        updateRigidBodyParam(platform_rigid_body,platform_mass,drag,angular_drag,use_gravity);
    }

    public void updateRigidBodyParam(Rigidbody RigBod,float desired_mass, float desired_drag, float desired_angularDrag,bool use_gravity_bool){
        RigBod.mass=desired_mass;RigBod.drag=desired_drag; RigBod.angularDrag =desired_angularDrag; RigBod.useGravity=use_gravity_bool;
    }

    public void CreateUpperArm(GameObject upper_arm_prefab){
        float temp_crank_angle=this.temp_theta;
        this.upper_arm_set_model=new GameObject("Upper Arm Set Model");
        this.upper_arm_set_model.transform.SetParent(this.delta_robot_model.transform, true);
        GameObject[] upper_arms= new GameObject[3];
        for (int i = 0; i < 3; i++){
            upper_arms[i]=GameObject.Instantiate(upper_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;;


            var temp_Renderer = upper_arms[i].GetComponent<Renderer>();

            //Call SetColor using the shader property name "_Color" and setting the color to red
            temp_Renderer.material.SetColor("_Color", Color.red);

            upper_arms[i].transform.Rotate(0f,90f+120f*i, 0f, Space.Self);
            upper_arms[i].transform.Rotate( 45f, 0f, 0f, Space.Self);
            upper_arms[i].transform.Rotate( 0f, 0f, 90f, Space.Self);
            Vector3 elbow_position=(this.rb+this.l*Mathf.Cos(temp_crank_angle))*this.s_l[i]+this.l*Mathf.Sin(temp_crank_angle)*new Vector3(0,-1f,0);
            Vector3 motor_position=this.rb*s_l[i];
            upper_arms[i].transform.position=0.5f*(elbow_position+motor_position)+this.overall_offset;
            upper_arms[i].transform.SetParent(this.upper_arm_set_model.transform, true);
            
            Rigidbody upper_arm_rigid_body = upper_arms[i].AddComponent<Rigidbody>();

            updateRigidBodyParam(upper_arm_rigid_body,upper_arm_mass,drag,angular_drag,use_gravity);
            upper_arms[i].AddComponent<HingeJoint> ();  
            HingeJoint joint = upper_arms[i].GetComponent<HingeJoint>();
            joint.connectedBody = base_model.GetComponent<Rigidbody>();
            var z_scale=upper_arms[i].transform.localScale.z;
            joint.anchor= new Vector3(0,0,-1f/z_scale*this.l*0.5f);
            joint.axis=new Vector3(0,1f,0);

            HingeJoint joint2=upper_arms[i].AddComponent<HingeJoint>();  
            var temp= this.elbow_set_model.transform.GetChild(i).gameObject;
            var el_bar= temp.transform.GetChild(0).gameObject;
            joint2.connectedBody = el_bar.GetComponent<Rigidbody>();
            joint2.anchor= new Vector3(0,0,1f/z_scale*this.l*0.5f);
            joint2.axis=new Vector3(0,1f,0);
        }
    }

    public void CreateLowerArm(GameObject lower_arm_prefab){
        // float temp_y= -10.63186143f;
        float temp_y= this.temp_y;
        float temp_crank_angle=this.temp_theta;
        this.lower_arm_set_model=new GameObject("Lower Arm Set Model");
        this.lower_arm_set_model.transform.SetParent(this.delta_robot_model.transform, true);
        lower_arm_model_list=new GameObject[3];
        GameObject[] lower_arms= new GameObject[6];
        for (int i = 0; i < 3; i++){
            lower_arms[2*i]=GameObject.Instantiate(lower_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;
            lower_arms[2*i+1]=GameObject.Instantiate(lower_arm_prefab, new Vector3(0f, 0f, 0f), Quaternion.identity) as GameObject;
            
            var temp_Renderer = lower_arms[2*i].GetComponent<Renderer>();
            temp_Renderer.material.SetColor("_Color", Color.cyan);

            temp_Renderer = lower_arms[2*i+1].GetComponent<Renderer>();
            temp_Renderer.material.SetColor("_Color", Color.yellow);


            lower_arms[2*i].transform.Rotate( 90f, 0f, 0f, Space.Self);
            lower_arms[2*i+1].transform.Rotate( 90f, 0f, 0f, Space.Self);

            if (i==1){
                lower_arms[2*i].transform.Rotate( 0, 0f,-30f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, 0f, -30f, Space.Self);
                lower_arms[2*i].transform.Rotate( 180f, 0f, 0, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 180f, 0f, 0, Space.Self);
            }
            else if (i==2){
                lower_arms[2*i].transform.Rotate( 0, 0f,-150f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, 0f, -150f, Space.Self);
                lower_arms[2*i].transform.Rotate( 180f, 0f, 0, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 180f, 0f, 0, Space.Self);
            }
            if (i!=0){
            lower_arms[2*i].transform.Rotate( - this.tilt_ang, 0f, 0f, Space.Self);
            lower_arms[2*i+1].transform.Rotate( -this.tilt_ang, 0f, 0f, Space.Self);}
            else{
                lower_arms[2*i].transform.Rotate( 0, -90f, -90f, Space.Self);
                lower_arms[2*i+1].transform.Rotate( 0, -90f, -90f, Space.Self);
                lower_arms[2*i].transform.Rotate( 90f+this.tilt_ang, 0,0, Space.Self);
                lower_arms[2*i+1].transform.Rotate(90f+this.tilt_ang, 0,0, Space.Self);
            }
            Vector3 elbow_position=(this.rb+this.l*Mathf.Cos(temp_crank_angle))*this.s_l[i]+this.l*Mathf.Sin(temp_crank_angle)*new Vector3(0,-1f,0);

            Vector3 platform_mid_point_position= this.re*s_l[i]- new Vector3(0,-1f*temp_y,0);

            Vector3 lower_arm_position_simplified= 0.5f*(elbow_position+platform_mid_point_position);
            
            Vector3 offset1 = this.re*this.s_l[(i+1)%3];
            Vector3 offset2 = this.re*this.s_l[(i+2)%3];

            lower_arms[2*i].transform.position=lower_arm_position_simplified + (offset1 - offset2)+this.overall_offset;
            lower_arms[2*i+1].transform.position=lower_arm_position_simplified+ (-offset1 + offset2)+this.overall_offset;

            //set parent in heirarchy

            this.lower_arm_model_list[i]=new GameObject("Lower Arm Model"+i.ToString());
            this.lower_arm_model_list[i].transform.SetParent(this.lower_arm_set_model.transform, true);
            lower_arms[2*i].transform.SetParent(this.lower_arm_model_list[i].transform, true);
            lower_arms[2*i+1].transform.SetParent(this.lower_arm_model_list[i].transform, true);
            //create rigid body
            Rigidbody lower_arm_rigid_body1 = lower_arms[2*i].AddComponent<Rigidbody>();
            Rigidbody lower_arm_rigid_body2 = lower_arms[2*i+1].AddComponent<Rigidbody>();
            updateRigidBodyParam(lower_arm_rigid_body1,lower_arm_mass,drag,angular_drag,use_gravity);
            updateRigidBodyParam(lower_arm_rigid_body2,lower_arm_mass,drag,angular_drag,use_gravity);
            //create character joint
            CharacterJoint joint1=lower_arms[2*i].AddComponent<CharacterJoint>();  
            CharacterJoint joint2=lower_arms[2*i+1].AddComponent<CharacterJoint>();  
            //attach to the capsule of the elbow
            var temp= this.elbow_set_model.transform.GetChild(i).gameObject;
            var el_bar= temp.transform.GetChild(0).gameObject;

            var z_scale=lower_arms[2*i].transform.localScale.z;
            joint1.connectedBody = el_bar.GetComponent<Rigidbody>();
            joint1.anchor= new Vector3(0,0,1f/z_scale*this.rou*0.5f);
            joint2.connectedBody = el_bar.GetComponent<Rigidbody>();
            joint2.anchor= new Vector3(0,0,1f/z_scale*this.rou*0.5f);

            CharacterJoint joint1_p=lower_arms[2*i].AddComponent<CharacterJoint>();  
            CharacterJoint joint2_p=lower_arms[2*i+1].AddComponent<CharacterJoint>();  
            joint1_p.connectedBody = this.platform_model.GetComponent<Rigidbody>();
            joint1_p.anchor= new Vector3(0,0,-1f/z_scale*this.rou*0.5f);
            joint2_p.connectedBody = this.platform_model.GetComponent<Rigidbody>();
            joint2_p.anchor= new Vector3(0,0,-1f/z_scale*this.rou*0.5f);
        }
    }

    public void CreateElbowBar(){
        float temp_crank_angle=this.temp_theta;
        float t= this.bar_thickness;
        this.elbow_set_model= new GameObject("Elbow Set Model");
        this.elbow_set_model.transform.SetParent(this.delta_robot_model.transform, true);
        elbow_model_list = new GameObject[3];
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
            elbow_bars[i].transform.position= elbow_position+this.overall_offset;

            elbow_balls[2*i]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            elbow_balls[2*i+1]=GameObject.CreatePrimitive(PrimitiveType.Sphere);
            elbow_balls[2*i].transform.localScale*=t*2f;
            elbow_balls[2*i+1].transform.localScale*=t*2f;
            Vector3 ball_offset1 = this.re*this.s_l[(i+1)%3];
            Vector3 ball_offset2 = this.re*this.s_l[(i+2)%3];
            elbow_balls[2*i].transform.position= elbow_position+ (ball_offset1 - ball_offset2)+this.overall_offset;
            elbow_balls[2*i+1].transform.position= elbow_position+ (-ball_offset1 + ball_offset2)+this.overall_offset;

            elbow_bars[i].AddComponent<Rigidbody>();
            updateRigidBodyParam(elbow_bars[i].GetComponent<Rigidbody>(),0.4f*elbow_mass,drag,angular_drag,use_gravity);
            elbow_balls[2*i].AddComponent<Rigidbody>();
            updateRigidBodyParam(elbow_balls[2*i].GetComponent<Rigidbody>(),0.3f*elbow_mass,drag,angular_drag,use_gravity);
            elbow_balls[2*i+1].AddComponent<Rigidbody>();
            updateRigidBodyParam(elbow_balls[2*i+1].GetComponent<Rigidbody>(),0.3f*elbow_mass,drag,angular_drag,use_gravity);
            elbow_balls[2*i].AddComponent<FixedJoint> ();  
            elbow_balls[2*i+1].AddComponent<FixedJoint>();
            FixedJoint ball1_joint = elbow_balls[2*i].GetComponent<FixedJoint>();
            FixedJoint ball2_joint = elbow_balls[2*i+1].GetComponent<FixedJoint>();
            ball1_joint.connectedBody = elbow_bars[i].GetComponent<Rigidbody>();
            ball2_joint.connectedBody = elbow_bars[i].GetComponent<Rigidbody>();
            //Sort out the heirarchy
            elbow_model_list[i]=new GameObject("Elbow Model"+i.ToString());
            elbow_model_list[i].transform.SetParent(elbow_set_model.transform, true);
            elbow_bars[i].transform.SetParent(elbow_model_list[i].transform, true);
            elbow_balls[2*i].transform.SetParent(elbow_model_list[i].transform, true);
            elbow_balls[2*i+1].transform.SetParent(elbow_model_list[i].transform, true);
        }
    }    
}


public class CreateDeltaRobotPrefab : MonoBehaviour
{
    // Start is called before the first frame update
    
    public DeltaRobotModel RobotModel1;
    public float scale_fact;
    public Vector3 assigned_offset;

    public GameObject upper_arm_5_unit_prefab;
    public GameObject lower_arm_9_unit_prefab;

    void Start()
    {   scale_fact=0.5f;
        assigned_offset=new Vector3(8f,0,8f);
        RobotModel1=new DeltaRobotModel(2f,5f,9f,1f,assigned_offset, upper_arm_5_unit_prefab, lower_arm_9_unit_prefab);
        // DeltaRobotModel RobotModel2=new DeltaRobotModel(1f,5f,9f,3f,assigned_offset+new Vector3(8f,0,8f), upper_arm_5_unit_prefab, lower_arm_9_unit_prefab);
        // DeltaRobotModel RobotModel3=new DeltaRobotModel(2f,7f,14f,1f,assigned_offset+2f*new Vector3(8f,0,8f), upper_arm_5_unit_prefab, lower_arm_9_unit_prefab);
        //Instantiate(upper_arm_5_unit_prefab, new Vector3(1f, 2f, 3f), Quaternion.identity);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
