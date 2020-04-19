using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
    // Inverse Kinematics - Position
    // Given end point positions
    // Deduce elbow joints positions (or three thetas)

    // Inverse Kinematics - Velocity
    // Given the inverse Jacobian and current platform velocity
    // Deduce the corresponding rotational speed for the three hinge joints



public class DeltaRobot{
    public float rb, l, rou, re;
    public Vector3 s0,s1,s2;
    public DeltaRobot(float base_radius, float upper_arm_length, float lower_arm_length, float end_radius){
        
        this.rb=base_radius;
        this.l=upper_arm_length;
        this.rou=lower_arm_length;
        this.re=end_radius;

        this.s0=new Vector3(1,0,0);
        this.s1=new Vector3(-1f/2f,0,-1f/2f*Mathf.Sqrt(3f));
        this.s2=new Vector3(-1f/2f,0,1f/2f*Mathf.Sqrt(3f));
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

    public Vector3[] differential_forward_kinematics(float[] joint_angle_l, bool to_display=false){
        Vector3[] si_l={this.s0, this.s1,this.s2}; s0=si_l[0]; s1=si_l[1]; s2=si_l[2]; 
        CGA.CGA[] ebasis= {e1,-1f*e2,e3};
        float rb=this.rb; float re=this.re; float rou=this.rou; float l = this.l;
        float theta0 = joint_angle_l[0];float theta1 = joint_angle_l[1];float theta2 = joint_angle_l[2];
        float[] theta_l=joint_angle_l;
        CGA.CGA[] si_l_cga={vector_to_pnt(this.s0),
                    vector_to_pnt(this.s1),
                    vector_to_pnt(this.s2)};

        CGA.CGA[] elbow_l=new CGA.CGA[3];

        CGA.CGA[] a_l=new CGA.CGA[3];CGA.CGA[] A_l=new CGA.CGA[3];Sphere[] S_l=new Sphere[3];
        CGA.CGA[] C_l=new CGA.CGA[3];
        for(int i=0;i<3;i++){
            elbow_l[i]=(rb+l*Mathf.Cos(theta_l[i]))*si_l_cga[i]+l*Mathf.Sin(joint_angle_l[i])*(-1f*e2);
            a_l[i]=elbow_l[i]-re*si_l_cga[i];A_l[i]=up_v(pnt_to_vector(a_l[i])); 
            elbow_l[i]=up_v(pnt_to_vector(elbow_l[i]));

            S_l[i]=new Sphere();S_l[i].SphereObj.active = false;
            S_l[i].Centre=pnt_to_vector(a_l[i]);S_l[i].Radius= rou;
            S_l[i].FindSphere5DbyCandRou();
        }
        CGA.CGA T=get_grade_2(!(!S_l[0].Sphere5D^!S_l[1].Sphere5D^!S_l[2].Sphere5D));
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

            C_l[i] =(!S_l[other_indices[0]].Sphere5D)^(!S_l[other_indices[1]].Sphere5D);
            CGA.CGA dT_dtheta= find_dT_dtheta(dSigma_dtheta,C_l[i]);
            CGA.CGA dP_dtheta= find_dP_dtheta(T,dT_dtheta);
            CGA.CGA dY_dtheta=find_dA_dalpha(~T,~P,~P_d,~dP_dtheta,-1f*dT_dtheta,true);
            Vector3 dy_dtheta=-1f*pnt_to_vector(find_da_dalpha(dY_dtheta, unnormedY,true));
            if (i==1){dy_dtheta=-1f*dy_dtheta;}
            jacobian[i]=dy_dtheta;
            if (to_display){Debug.Log(jacobian[i]);}
        }
        return jacobian;
    }

    public float[,] differential_inverse_kinematics(Vector3 end_point, bool to_display=false){
        // Inverse Jacobian: 
        // Given current end points location 
        // Deduce how the rotational speed of three joints change with three Cartesian directions. 
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
                Sphere Sigmai=new Sphere();Sigmai.SphereObj.active = false;Sigmai.Centre=pnt_to_vector(x);
                Sigmai.Radius= rou;Sigmai.FindSphere5DbyCandRou();
                CGA.CGA T=find_Ti(Sigmai.Sphere5D,C_l[i]);
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
        return jacobian;
    }
}



public class DeltaRobotClass : MonoBehaviour
{
    // Start is called before the first frame update
    public float[,] inv_jaco;
    void Start()
    {
        DeltaRobot Robot1=new DeltaRobot(4f,5f,9f,1f);
        Vector3 end_point=new Vector3(0,-12f,0);
        float[] joint_angle_l= {(Mathf.PI)/3f,(Mathf.PI)/4f,(Mathf.PI)/5f} ;
        //inv_jaco= Robot1.differential_inverse_kinematics(end_point);
        Vector3[] frwd_jaco=Robot1.differential_forward_kinematics(joint_angle_l,true);
        

    // Update is called once per frame
    void Update()
    {
        
    }
}
}