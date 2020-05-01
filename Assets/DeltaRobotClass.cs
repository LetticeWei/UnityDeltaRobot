using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
    // Inverse Kinematics - Position
    // Given end point positions
    // Deduce elbow joints positions (or three thetas)


public class DeltaRobot{
    public float rb, l, rou, re;
    public Vector3 s0,s1,s2;
    public Vector3 end_point, dy_dt;
    public float[] theta_l,dtheta_dt_l;
    public float[,] inv_jaco;
    public Vector3[] frwd_jaco;
    public Point[] Base_l, Elbow_l, Platform_l;
    public Vector3[] s_l;
    public DeltaRobot(float base_radius, float upper_arm_length, float lower_arm_length, float end_radius){
        
        this.rb=base_radius;
        this.l=upper_arm_length;
        this.rou=lower_arm_length;
        this.re=end_radius;

        this.s0=new Vector3(1,0,0);
        this.s1=new Vector3(-1f/2f,0,-1f/2f*Mathf.Sqrt(3f));
        this.s2=new Vector3(-1f/2f,0,1f/2f*Mathf.Sqrt(3f));
        this.s_l=new Vector3[]{s0,s1,s2};
        this.Base_l=new Point[3];this.Elbow_l=new Point[3];this.Platform_l=new Point[3];

        for (int i =0; i<3;i++){
            this.Base_l[i]=new Point();
            this.Base_l[i].Point3D=this.s_l[i]*this.rb;
            this.Base_l[i].SetupGameObject(0.1f,0.7f,0.9f); //set the colour

            this.Elbow_l[i]=new Point();
            this.Elbow_l[i].Point3D=(this.rb+this.l*Mathf.Cos(Mathf.PI/4))*s_l[i]+ pnt_to_vector(l*Mathf.Sin(Mathf.PI/4)*(-1f*e2)); //set a default theta = pi/4
            this.Elbow_l[i].SetupGameObject(0.7f,0.9f,0.1f); //set the colour

            this.Platform_l[i]=new Point();
            this.Platform_l[i].Point3D=new Vector3(0,-1f*this.l,0); //set a default endpoint position
            this.Platform_l[i].SetupGameObject(0.9f,0.1f,0.7f); //set the colour
        }
    }
    
    public void update_framework(){
        //given this.endpoint and this.theta_l are updated 
        for(int i =0; i<3;i++){
            // Debug.Log(Elbow_l[i]);
            this.Platform_l[i].Point3D=this.end_point+this.s_l[i]*this.re;
            this.Platform_l[i].UpdateGameObject();
            this.Elbow_l[i].Point3D=(this.rb+this.l*Mathf.Cos(this.theta_l[i]))*this.s_l[i]+ pnt_to_vector(l*Mathf.Sin(this.theta_l[i])*(-1f*e2));
            this.Elbow_l[i].UpdateGameObject();

            }
    }

    public CGA.CGA find_Ci(Vector3 si){
        var si_cga=vector_to_pnt(si);
        var res=up_v(rb*si);
        CGA.CGA temp_Sphere_dual= res -0.5f*l*l*ei; 
        CGA.CGA Ci = temp_Sphere_dual^(I3*((-1f*e2)^si_cga));
        return Ci;
    }
    public CGA.CGA diff_up(CGA.CGA dxdalpha, CGA.CGA x){
        CGA.CGA dXdalpha=(dxdalpha|x)[0]* ei + dxdalpha;
        return dXdalpha;
    }
    public CGA.CGA find_Ti(CGA.CGA Sigmai5D,CGA.CGA Ci){
        return !((!Sigmai5D)^Ci);
    }
    public CGA.CGA find_dTi_dalpha(CGA.CGA dSi_dalpha,CGA.CGA Ci){
        return !(dSi_dalpha^Ci);
    }
    public (CGA.CGA, CGA.CGA) find_P(CGA.CGA T){ //T is a point pair
        var beta=(float) Mathf.Sqrt((T*T)[0]);
        var F=1.0f/beta*T;
        var P=0.5f*(one +F);
        var P_d=0.5f*(one -F);
        return (P,P_d);
    }
    public CGA.CGA find_dP_dthetai(CGA.CGA T,CGA.CGA dT_dthetai){
        float beta = Mathf.Sqrt((T|T)[0]);
        CGA.CGA dbeta_dthetai = (dT_dthetai|T)*(1f/beta);
        CGA.CGA dP_dthetai = (dT_dthetai|T)*(1f/beta);
        dP_dthetai = 0.5f*(beta*dT_dthetai - T*dbeta_dthetai)*(1f/(beta*beta));
        return dP_dthetai;
    }
    public CGA.CGA find_zi(CGA.CGA a, Vector3 si){
        var si_cga=vector_to_pnt(si);
        var z=a-rb*si_cga;
        return z;
    }
    public CGA.CGA find_dA_dalpha(CGA.CGA T,CGA.CGA P,CGA.CGA P_d,CGA.CGA dP_dthetai,CGA.CGA dT_dthetai, bool is_dY_dtheta=false){
        var dP_d_dthetai=~dP_dthetai;
        CGA.CGA part1= -1f*dP_d_dthetai*(T|ei)*P;
        CGA.CGA part2=-1f*P_d*(dT_dthetai|ei)*P;
        CGA.CGA part3=-1f*P_d*(T|ei)*dP_dthetai;
        return get_grade_1(part1+part2+part3);
    }
    public CGA.CGA find_da_dalpha(CGA.CGA dAi_dt, CGA.CGA Ai, bool isY=false){
        float denom = (Ai|ei)[0]*(Ai|ei)[0];
        CGA.CGA left=one-one;  //zero vector
        CGA.CGA right=one-one;  //zero vector
        CGA.CGA[] basis_array= {e1, e2, e3};
        foreach (CGA.CGA b in basis_array){
          left +=-1f*(dAi_dt|b)*b*(Ai|ei); 
          right+= (Ai|b)*b*(dAi_dt|ei);}
        return (left+right)*(1f/denom);
    }

    public float find_dtheta_dalpha(CGA.CGA zi, CGA.CGA dz_dt,Vector3 si){
        var si_cga=vector_to_pnt(si);
        var denom=(zi|zi)[0];
        var nume=(zi|si_cga)[0]*(dz_dt|(-1f*e2))[0]-(zi|(-1f*e2))[0]*(dz_dt|si_cga)[0];
        return nume/denom;
    }
    public CGA.CGA find_dSigmai_dthetai(float thetai,Vector3 si){
        var si_cga=vector_to_pnt(si);
        var dai_dthetai=-1f*l*Mathf.Sin(thetai)*si_cga + l*Mathf.Cos(thetai)*(-1f*e2);
        var dai2_dthetai = -2f*(rb-re)*l*Mathf.Sin(thetai);
        var dAi_dthetai = 0.5f*dai2_dthetai*ei + dai_dthetai;
        var dSigmai_dthetai = dAi_dthetai;
        return dSigmai_dthetai;
    }
    public CGA.CGA find_dT_dtheta(CGA.CGA dSigma_dtheta,CGA.CGA C){
        var dT_dtheta= !(dSigma_dtheta^C); 
        return dT_dtheta;
    }
    public CGA.CGA find_dP_dtheta(CGA.CGA T,CGA.CGA dT_dthetai){
        float beta = Mathf.Sqrt((T|T)[0]);
        CGA.CGA dbeta_dthetai = (dT_dthetai|T)*(1f/beta);
        CGA.CGA dP_dthetai = (dT_dthetai|T)*(1f/beta);
        dP_dthetai = 0.5f*(beta*dT_dthetai - T*dbeta_dthetai)*(1f/(beta*beta));
        return dP_dthetai;
    }
    
    public void assign_end_point_pisition(Vector3 end_point){this.end_point=end_point;}
    public void assign_end_point_velocity(Vector3 dy_dt){this.dy_dt=dy_dt;}
    public void assign_joint_angle_position(float[] theta_l){this.theta_l=theta_l;}
    public void assign_joint_angle_velocity(float[] dtheta_dt_l){this.dtheta_dt_l=dtheta_dt_l;}

    public Vector3 forward_kinematics(){
        
        Vector3[] si_l={this.s0, this.s1,this.s2};
        CGA.CGA[] si_l_cga={vector_to_pnt(this.s0),
            vector_to_pnt(this.s1),
            vector_to_pnt(this.s2)};
        float rb=this.rb; float re=this.re; float rou=this.rou; float l = this.l;
        float[] joint_angle_l=this.theta_l;
        
        CGA.CGA[] elbow_l=new CGA.CGA[3];CGA.CGA[] S_l=new CGA.CGA[3];
        CGA.CGA[] a_l=new CGA.CGA[3];CGA.CGA[] A_l=new CGA.CGA[3];

        for(int i=0;i<3;i++){
            elbow_l[i]=(rb+l*Mathf.Cos(theta_l[i]))*si_l_cga[i]+l*Mathf.Sin(joint_angle_l[i])*(-1f*e2);
            a_l[i]=elbow_l[i]-re*si_l_cga[i];A_l[i]=up_v(pnt_to_vector(a_l[i])); 
            elbow_l[i]=up_v(pnt_to_vector(elbow_l[i]));
            CGA.CGA Centre5D=up_v(pnt_to_vector(a_l[i]));
            S_l[i]= !(Centre5D+(-0.5f)*rou*rou*ei);
        }
        CGA.CGA T=get_grade_2(!(!S_l[0]^!S_l[1]^!S_l[2]));
        var result = find_P(T);
        var P=result.Item1;var P_d=result.Item2;
        CGA.CGA Y=ExtractPntfromPntPairs(T,true,true); Vector3 y=pnt_to_vector(down(Y));
        this.end_point=y;
        return y;
    }

    public Vector3[] differential_forward_kinematics( bool to_display=false){
        float[] joint_angle_l=this.theta_l;
        Vector3[] si_l={this.s0, this.s1,this.s2}; s0=si_l[0]; s1=si_l[1]; s2=si_l[2]; 
        CGA.CGA[] ebasis= {e1,-1f*e2,e3};
        float rb=this.rb; float re=this.re; float rou=this.rou; float l = this.l;
        float theta0 = joint_angle_l[0];float theta1 = joint_angle_l[1];float theta2 = joint_angle_l[2];
        float[] theta_l=joint_angle_l;
        CGA.CGA[] si_l_cga={vector_to_pnt(this.s0),
                    vector_to_pnt(this.s1),
                    vector_to_pnt(this.s2)};

        CGA.CGA[] elbow_l=new CGA.CGA[3];

        CGA.CGA[] a_l=new CGA.CGA[3];CGA.CGA[] A_l=new CGA.CGA[3];CGA.CGA[] S_l=new CGA.CGA[3];
        CGA.CGA[] C_l=new CGA.CGA[3];
        for(int i=0;i<3;i++){
            elbow_l[i]=(rb+l*Mathf.Cos(theta_l[i]))*si_l_cga[i]+l*Mathf.Sin(joint_angle_l[i])*(-1f*e2);
            a_l[i]=elbow_l[i]-re*si_l_cga[i];A_l[i]=up_v(pnt_to_vector(a_l[i])); 
            elbow_l[i]=up_v(pnt_to_vector(elbow_l[i]));
            CGA.CGA Centre5D=up_v(pnt_to_vector(a_l[i]));
            S_l[i]= !(Centre5D+(-0.5f)*rou*rou*ei);


        }
        CGA.CGA T=get_grade_2(!(!S_l[0]^!S_l[1]^!S_l[2]));
        var result = find_P(T);
        var P=result.Item1;var P_d=result.Item2;
        CGA.CGA Y=ExtractPntfromPntPairs(T,true,true); Vector3 y=pnt_to_vector(down(Y));

        CGA.CGA unnormedY = ExtractPntfromPntPairs(T, false,true);

        Vector3[] jacobian=new Vector3[3];
        int[] ind_l={0,1,2};

        for (int i=0;i<3;i++){ // Which theta to deal with
            CGA.CGA dSigma_dtheta=find_dSigmai_dthetai(theta_l[i],si_l[i]);
            int [] other_indices;

            if (i==0){other_indices= new int[] {1,2};}
            else if (i==1){other_indices= new int[] {0,2};}
            else{other_indices= new int[] {0,1};}

            C_l[i] =(!S_l[other_indices[0]])^(!S_l[other_indices[1]]);
            CGA.CGA dT_dtheta= find_dT_dtheta(dSigma_dtheta,C_l[i]);
            CGA.CGA dP_dtheta= find_dP_dtheta(T,dT_dtheta);
            CGA.CGA dY_dtheta=find_dA_dalpha(~T,~P,~P_d,~dP_dtheta,-1f*dT_dtheta,true);
            Vector3 dy_dtheta=-1f*pnt_to_vector(find_da_dalpha(dY_dtheta, unnormedY,true));
            if (i==1){dy_dtheta=-1f*dy_dtheta;}
            jacobian[i]=dy_dtheta;
            if (to_display){Debug.Log(jacobian[i]);}
        }
        this.frwd_jaco=jacobian;
        return jacobian;
    }

    public float[] inverse_kinematics(){
        Vector3[] si_l={this.s0, this.s1,this.s2};
        float rb=this.rb; float re=this.re; float rou=this.rou; float l = this.l;
        Vector3 y=this.end_point;
        
        CGA.CGA[] C_l=new CGA.CGA[3];
        CGA.CGA[] A_l=new CGA.CGA[3];
        CGA.CGA[] S_l=new CGA.CGA[3];CGA.CGA[] T_l=new CGA.CGA[3];
        CGA.CGA[] z_l=new CGA.CGA[3];
        float[] joint_angle_l=new float[3];
        
        CGA.CGA Y=up_v(y);
        for (int i=0; i<3; i++){
            C_l[i]=find_Ci(si_l[i]);
            S_l[i]= !(up_v(y+re*si_l[i])+(-0.5f)*rou*rou*ei);
            T_l[i]=find_Ti(S_l[i],C_l[i]);
            }
        CGA.CGA C0=C_l[0]; CGA.CGA C1=C_l[1]; CGA.CGA C2=C_l[2]; 
        bool reachable= ((T_l[0]*T_l[0])[0] > 0f) && ((T_l[1]*T_l[1])[0] > 0f) && ((T_l[2]*T_l[2])[0] > 0f);
        if(reachable){
            for (int i=0; i<3; i++){
            A_l[i]= ExtractPntfromPntPairs(T_l[i],true,true);
            z_l[i]=find_zi(down(A_l[i]),si_l[i]);
            joint_angle_l[i]=Mathf.Atan2((z_l[i]|(-1f*e2))[0], (z_l[i]|vector_to_pnt(si_l[i]))[0]);
            }
        }
        this.theta_l=joint_angle_l;
        return joint_angle_l;
    }
    public float[,] differential_inverse_kinematics(bool to_display=false){
        // Inverse Jacobian: 
        // Given current end points location 
        // Deduce how the rotational speed of three joints change with three Cartesian directions. 
        Vector3 end_point= this.end_point;
        CGA.CGA y= vector_to_pnt(end_point);
        CGA.CGA[] ebasis= {e1,-1f*e2,e3};

        Vector3[] si_l={this.s0, this.s1,this.s2};

        CGA.CGA[] si_l_cga={vector_to_pnt(this.s0),
                            vector_to_pnt(this.s1),
                            vector_to_pnt(this.s2)};
        float rb=this.rb; float re=this.re; float rou=this.rou; float l = this.l;
        CGA.CGA[] C_l=new CGA.CGA[3];
        for (int i = 0; i < 3; i++){
            C_l[i]=find_Ci(si_l[i]);
        }
        CGA.CGA C0=C_l[0]; CGA.CGA C1=C_l[1]; CGA.CGA C2=C_l[2]; 
        float[,] jacobian=new float[3,3];
        for (int k=0;k<3;k++){
            for (int i=0; i<3; i++){
                CGA.CGA dydalpha = ebasis[k];
                // Diff through the up 
                CGA.CGA x=y+re*si_l_cga[i];
                CGA.CGA dxdalpha=dydalpha;
                CGA.CGA dXdalpha=  diff_up(dxdalpha,x);
                // Diff the T
                CGA.CGA Centre5D=up_v(pnt_to_vector(x));
                CGA.CGA Sigmai= !(Centre5D+(-0.5f)*rou*rou*ei);

                CGA.CGA T=find_Ti(Sigmai,C_l[i]);
                CGA.CGA dT_dalpha=find_dTi_dalpha(dXdalpha,C_l[i]);
                // Diff the endpoint
                var result = find_P(T);
                var P=result.Item1;var P_d=result.Item2;
                CGA.CGA dP_dalpha=find_dP_dthetai(T,dT_dalpha);
                CGA.CGA dA_dalpha=find_dA_dalpha(~T,~P,~P_d,~dP_dalpha,-1f*dT_dalpha);
                // Diff through the down
                CGA.CGA unnormedA= ExtractPntfromPntPairs(T,false,true);
                CGA.CGA da_dalpha=-1f*find_da_dalpha(dA_dalpha, unnormedA);
                if (k==1){da_dalpha= -1f*da_dalpha;}
                // Get the theta derivative
                CGA.CGA A= ExtractPntfromPntPairs(T,true,true);
                Vector3 a=pnt_to_vector(down(A));
                CGA.CGA z=find_zi(vector_to_pnt(a),si_l[i]);
                float dtheta_dalpha=find_dtheta_dalpha(z, da_dalpha,si_l[i]);
                jacobian[i,k] = dtheta_dalpha;
            }
        }
        if(to_display){
            for (int i = 0; i < 3; i++){
                for (int j = 0; j < 3; j++){
                Debug.Log(jacobian[i, j]);}
            }
        }
        this.inv_jaco=jacobian;
        return jacobian;
    }
    public float[] inverse_kinematics_velocity(bool to_display=false){
        // Inverse Kinematics - Velocity
        // Given the current platform position and velocity, work out inverse Jacobian
        // Deduce the corresponding rotational speed for the three hinge joints
        Vector3 end_point=this.end_point;
        Vector3 dy_dt=this.dy_dt;
        float[,] inv_jaco=differential_inverse_kinematics();
        this.inv_jaco=inv_jaco;
        float dtheta0_dt=inv_jaco[0,0]*dy_dt.x+inv_jaco[0,1]*dy_dt.y+inv_jaco[0,2]*dy_dt.z;
        float dtheta1_dt=inv_jaco[1,0]*dy_dt.x+inv_jaco[1,1]*dy_dt.y+inv_jaco[1,2]*dy_dt.z;
        float dtheta2_dt=inv_jaco[2,0]*dy_dt.x+inv_jaco[2,1]*dy_dt.y+inv_jaco[2,2]*dy_dt.z;
        float[] dtheta_dt_l=new float[]{dtheta0_dt,dtheta1_dt,dtheta2_dt};
        // Debug.Log(dtheta0_dt);
        if(to_display){for (int i = 0; i < 3; i++){Debug.Log(dtheta_dt_l[i]);}}
        //update parameter for the DeltaRobot Object
        this.dtheta_dt_l=dtheta_dt_l;
        for(int i=0; i<3;i++){
            this.theta_l[i]+=this.dtheta_dt_l[i];}
        return dtheta_dt_l;
    }
}
public class DeltaRobotClass : MonoBehaviour
{
    // public float[,] inv_jaco;
    public DeltaRobot Robot1;
    public float time;
    public float[] dtheta_dt_l,theta_l;
    public float scale_fact;
    // public float dtheta_dt0,dtheta_dt1;

    // Start is called before the first frame update

    void Start()
    {   
        scale_fact=0.5f;
        Robot1=new DeltaRobot(scale_fact*4.773502f,scale_fact*10f,scale_fact*21.96040974f,scale_fact*3.58013f);
        Robot1.assign_end_point_pisition(scale_fact*new Vector3(0,-24f,0));
        time=0f;
        //inv_jaco= Robot1.differential_inverse_kinematics(end_point);
        Robot1.dtheta_dt_l=new float[]{0,0,0};

        // float[] joint_angle_l= {(Mathf.PI)/3f,(Mathf.PI)/4f,(Mathf.PI)/5f} ;
        // Vector3[] frwd_jaco=Robot1.differential_forward_kinematics(joint_angle_l,true);
        
    }
    // Update is called once per frame
    void Update()
    {
        // Vector3 dy_dt_to_assign=new Vector3((float) Mathf.Cos(time)*0.3f,(float) Mathf.Cos(time*0.6f)*(-0.25f),(float) Mathf.Cos(time*1.2f)*0.35f);
        // Vector3 dy_dt_to_assign=new Vector3(0,0,(float) Mathf.Cos(time*0.6f)*(-0.45f));
        Vector3 dy_dt_to_assign=scale_fact*new Vector3(0,(float) -1f*Mathf.Cos(time*0.6f)*(-0.25f),(float) Mathf.Sin(time*0.6f)*(-0.25f));

        Robot1.assign_end_point_velocity(dy_dt_to_assign);
        // Debug.Log(Robot1.end_point);
        // Debug.Log(Robot1.dy_dt);
        Robot1.assign_end_point_pisition(Robot1.end_point+dy_dt_to_assign);
        
        theta_l=Robot1.inverse_kinematics();
        dtheta_dt_l=Robot1.inverse_kinematics_velocity();
        // dtheta_dt0=dtheta_dt_l[0];dtheta_dt1=dtheta_dt_l[1];
        //  Debug.Log(dtheta_dt_l);
        // for (int i = 0; i < 3; i++){
        //     Debug.Log(dtheta_dt_l[i]);
        //     }
        Robot1.update_framework();
        time+=Mathf.PI*0.05f;
    }

}