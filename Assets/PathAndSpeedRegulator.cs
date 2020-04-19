using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
using System;
using static ShapeClasses;

public class PathAndSpeedRegulator : MonoBehaviour
{
    public float rb,l,rou,re;
    public float theta0,theta1,theta2; //in degree
    public Vector3 s0,s1,s2;
    public Vector3 a0,a1,a2,elb0,elb1,elb2;
    public Point elbow0,elbow1,elbow2,pivot0,pivot1,pivot2,point_y,point_y_another;
    public Vector3 y;
    public CGA.CGA T,P,P_d,Y;
    public CGA.CGA C0,C1,C2;
    public CGA.CGA dSigma0_dtheta0,dSigma1_dtheta1,dSigma2_dtheta2;
    public CGA.CGA dT_dtheta0,dT_dtheta1,dT_dtheta2,dP_dtheta0,dP_dtheta1,dP_dtheta2,dY_dtheta0,dY_dtheta1,dY_dtheta2;
    public Vector3 dy_dtheta0,dy_dtheta1,dy_dtheta2;
    double[] dthetai_dt_array;
    public float dtheta0_dt,dtheta1_dt,dtheta2_dt;

    public Vector3 dy_dt; // will be changed later
    public Sphere Sigma0,Sigma1,Sigma2;
    public float timer;
    public double[][] mat;
    public Vector3 findEndPoint_ai(float thetai,Vector3 si){
        var si_cga=vector_to_pnt(si);
        CGA.CGA ai=(rb-re+l*Mathf.Cos(thetai))*si_cga+l*Mathf.Sin(thetai)*(-1f*e2);
        return pnt_to_vector(ai);
    }
    public Vector3 findEndPoint_elbi(float thetai,Vector3 si){
        var si_cga=vector_to_pnt(si);
        CGA.CGA elbi=(rb+l*Mathf.Cos(thetai))*si_cga+l*Mathf.Sin(thetai)*(-1f*e2);
        return pnt_to_vector(elbi);
    }
    public CGA.CGA find_dSigmai_dthetai(Vector3 ai,float thetai,Vector3 si){

        var si_cga=vector_to_pnt(si);

        var dai_dthetai=-1f*l*Mathf.Sin(thetai)*si_cga + l*Mathf.Cos(thetai)*(-1f*e2);

        // var dAi_dthetai=dai_dthetai + (vector_to_pnt(ai)|dai_dthetai)[0]*ei-eo;

        //Hugo's Code
        var dai2_dthetai = -2f*(rb-re)*l*Mathf.Sin(thetai);
        var dAi_dthetai = 0.5f*dai2_dthetai*ei + dai_dthetai;

        var dSigmai_dthetai = dAi_dthetai;
        
        return dSigmai_dthetai;
    }
    public CGA.CGA find_dT_dthetai(CGA.CGA dSigmai_dthetai,CGA.CGA Ci,CGA.CGA Sigmai5D){
        var dT_dthetai= !(dSigmai_dthetai^Ci); //////// The denominator may also change!!! be carefulll!!!!!!!!!!!!!!! may need modification later!
        return dT_dthetai;
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
    public CGA.CGA find_dY_dthetai(CGA.CGA T,CGA.CGA P,CGA.CGA P_d,CGA.CGA dP_dthetai,CGA.CGA dT_dthetai){
        var dP_d_dthetai=~dP_dthetai;
        CGA.CGA part1= -1f*dP_d_dthetai*(T|ei)*P;
        CGA.CGA part2=-1f*P_d*(dT_dthetai|ei)*P;
        CGA.CGA part3=-1f*P_d*(T|ei)*dP_dthetai;
        return get_grade_1(part1+part2+part3);
    }
    public CGA.CGA find_dy_dthetai(CGA.CGA dY_dthetai, CGA.CGA Y){
        float denom = (Y|ei)[0]*(Y|ei)[0];
        CGA.CGA left=one-one;  //zero vector
        CGA.CGA right=one-one;  //zero vector
        CGA.CGA[] basis_array= {e1, e2, e3};
        foreach (CGA.CGA b in basis_array){
          left +=-1f*(dY_dthetai|b)*b*(Y|ei); 
          right+= (Y|b)*b*(dY_dthetai|ei);}
        return (left+right)*(1f/denom);
    }
    public double[] vector_to_array(Vector3 vec){
        double[] double_array = new double[] { (double)vec.x,(double)vec.y,(double)vec.z };
        return double_array;}
    public double[,] ImperativeConvert(double[][] source){
        double[,] result = new double[source.Length, source[0].Length];
        for (int i = 0; i < source.Length; i++){
            for (int k = 0; k < source[0].Length; k++){
                result[i, k] = source[i][k];}
        }
        return result;}
    public double[] ComputeCoefficents(double[,] X, double[] Z){
      int I, J, K, K1, N;
      double[] Y = (double[]) Z.Clone();
      N = Y.Length;
      for (K = 0; K < N; K++){
        K1 = K + 1;
        for (I = K; I < N; I++){
          if (X[I, K] != 0){
            for (J = K1; J < N; J++){
              X[I, J] /= X[I, K];
            }
            Y[I] /= X[I, K];
          }
        }
        for (I = K1; I < N; I++){
          if (X[I, K] != 0){
            for (J = K1; J < N; J++){
              X[I, J] -= X[K, J];
            }
            Y[I] -= Y[K];
          }
        }
      }
      for (I = N - 2; I >= 0; I--){
        for (J = N - 1; J >= I + 1; J--){
          Y[I] -= X[I, J] * Y[J];
        }
      }
      return Y;
    }

    // Start is called before the first frame update
    void Start()
    {   dy_dt=new Vector3(0,0,0);
        double[] desired_velocity = vector_to_array(dy_dt);
        //set up the framework
        rb=4f;l=5f;rou=9f;re=1f;

        s0=new Vector3(rb,0,0)/rb;
        s1=new Vector3(-1*rb/2f,0,-1*rb/2f*Mathf.Sqrt(3f))/rb;
        s2=new Vector3(-1*rb/2f,0,rb/2f*Mathf.Sqrt(3f))/rb;

        pivot0=new Point();pivot0.Point3D=s0;pivot0.SetupGameObject(0.1f,0.3f,0.7f);
        pivot1=new Point();pivot1.Point3D=s1;pivot1.SetupGameObject(0.1f,0.3f,0.7f);
        pivot2=new Point();pivot2.Point3D=s2;pivot2.SetupGameObject(0.1f,0.3f,0.7f);


        theta0= (Mathf.PI)/3f;
        theta1= (Mathf.PI)/4f;
        theta2= (Mathf.PI)/5f;

        //ai are centres of the spheres
        a0=findEndPoint_ai(theta0,s0);
        a1=findEndPoint_ai(theta1,s1);
        a2=findEndPoint_ai(theta2,s2);

        elb0=findEndPoint_elbi(theta0,s0);
        elb1=findEndPoint_elbi(theta1,s1);
        elb2=findEndPoint_elbi(theta2,s2);

        // For visualisation purpose
        elbow0=new Point();elbow0.Point3D=elb0;elbow0.SetupGameObject(1f,0.2f,0.1f);
        elbow1=new Point();elbow1.Point3D=elb1;elbow1.SetupGameObject(1f,0.2f,0.1f);
        elbow2=new Point();elbow2.Point3D=elb2;elbow2.SetupGameObject(1f,0.2f,0.1f);

        Sigma0=new Sphere();Sigma0.SphereObj.active = false;Sigma0.Centre=a0;Sigma0.Radius= rou;Sigma0.FindSphere5DbyCandRou();
        Sigma1=new Sphere();Sigma1.SphereObj.active = false;Sigma1.Centre=a1;Sigma1.Radius= rou;Sigma1.FindSphere5DbyCandRou();
        Sigma2=new Sphere();Sigma2.SphereObj.active = false;Sigma2.Centre=a2;Sigma2.Radius= rou;Sigma2.FindSphere5DbyCandRou();

        C0=(!Sigma1.Sphere5D)^(!Sigma2.Sphere5D);
        C1=(!Sigma0.Sphere5D)^(!Sigma2.Sphere5D);
        C2=(!Sigma1.Sphere5D)^(!Sigma0.Sphere5D);

        T=get_grade_2(!(!Sigma0.Sphere5D^!Sigma1.Sphere5D^!Sigma2.Sphere5D));
        var result = find_P(T);
        P=result.Item1;P_d=result.Item2;
        Y=ExtractPntBfromPntPairs(T); //extract one point from the point pair
        y=pnt_to_vector(down(Y));
        point_y=new Point();point_y.Point3D=y;point_y.SetupGameObject(0f,0f,0f);
        point_y_another=new Point();point_y_another.Point3D=y;point_y_another.SetupGameObject(0.7f,0.1f,0.4f);
        //compute the derivatives
        dSigma0_dtheta0=find_dSigmai_dthetai(a0,theta0,s0);
        dSigma1_dtheta1=find_dSigmai_dthetai(a1,theta1,s1);
        dSigma2_dtheta2=find_dSigmai_dthetai(a2,theta2,s2);
        
        dT_dtheta0= find_dT_dthetai(dSigma0_dtheta0,C0,!Sigma0.Sphere5D);
        dT_dtheta1= find_dT_dthetai(dSigma1_dtheta1,C1,!Sigma1.Sphere5D);
        dT_dtheta2= find_dT_dthetai(dSigma2_dtheta2,C2,!Sigma2.Sphere5D);

        dP_dtheta0=find_dP_dthetai(T,dT_dtheta0);
        dP_dtheta1=find_dP_dthetai(T,dT_dtheta1);
        dP_dtheta2=find_dP_dthetai(T,dT_dtheta2);

        dY_dtheta0=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta0,-1f*dT_dtheta0);
        dY_dtheta1=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta1,-1f*dT_dtheta1);
        dY_dtheta2=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta2,-1f*dT_dtheta2);

        // check ExtractPntAfromPntPairs!!!!
        CGA.CGA unnormedY = ExtractPntBfromPntPairs(T, false);
        dy_dtheta0=-1f*pnt_to_vector(find_dy_dthetai(dY_dtheta0, unnormedY));
        dy_dtheta1=pnt_to_vector(find_dy_dthetai(dY_dtheta1, unnormedY));
        dy_dtheta2=pnt_to_vector(find_dy_dthetai(dY_dtheta2, unnormedY));

        double[] dy_dtheta0_arr = vector_to_array(dy_dtheta0);
        double[] dy_dtheta1_arr = vector_to_array(dy_dtheta1);
        double[] dy_dtheta2_arr = vector_to_array(dy_dtheta2);

        mat = new double[3][]; 
        mat[0]=dy_dtheta0_arr;mat[1]=dy_dtheta1_arr;mat[2]=dy_dtheta2_arr;


    }
   
    // Update is called once per frame
    public void update_elbowi(Point elbowjointi0,float dthetai_dt){
    }
    public void update_point_y(Point point_y, Vector3 dy_dt){
        point_y.GameObj.transform.position+=dy_dt;
    }
    void FixedUpdate()
    {    

        dtheta0_dt=(float) Mathf.Cos(timer)*0.05f;
        dtheta1_dt=(float) Mathf.Cos(timer*0.6f)*(-0.05f);
        dtheta2_dt=(float) Mathf.Cos(timer*1.2f)*0.05f;

        var dthetai_dt_vec=new Vector3(dtheta0_dt,dtheta1_dt,dtheta2_dt);
        dthetai_dt_array=vector_to_array(dthetai_dt_vec);
        //foreach(var dthetai_dt in dthetai_dt_array){Debug.Log(dthetai_dt.ToString());}

        //update the framework according to dthetai_dt
        theta0+=dtheta0_dt;
        theta1+=dtheta1_dt;
        theta2+=dtheta2_dt;
        

        //ai are centres of the spheres
        a0=findEndPoint_ai(theta0,s0);
        a1=findEndPoint_ai(theta1,s1);
        a2=findEndPoint_ai(theta2,s2);

        elb0=findEndPoint_elbi(theta0,s0);
        elb1=findEndPoint_elbi(theta1,s1);
        elb2=findEndPoint_elbi(theta2,s2);

        // For visualisation purpose
        elbow0.Point3D=elb0;elbow0.UpdateGameObject();
        elbow1.Point3D=elb1;elbow1.UpdateGameObject();
        elbow2.Point3D=elb2;elbow2.UpdateGameObject();

        Sigma0.Centre=a0;Sigma0.FindSphere5DbyCandRou();
        Sigma1.Centre=a1;Sigma1.FindSphere5DbyCandRou();
        Sigma2.Centre=a2;Sigma2.FindSphere5DbyCandRou();

        C0=(!Sigma1.Sphere5D)^(!Sigma2.Sphere5D);
        C1=(!Sigma0.Sphere5D)^(!Sigma2.Sphere5D);
        C2=(!Sigma1.Sphere5D)^(!Sigma0.Sphere5D);

        T=get_grade_2(!(!Sigma0.Sphere5D^!Sigma1.Sphere5D^!Sigma2.Sphere5D));
        var result = find_P(T);
        P=result.Item1;P_d=result.Item2;
        Y=ExtractPntBfromPntPairs(T); //extract one point from the point pair
        var y_old=y;
        y=pnt_to_vector(down(Y));
        var dy_dt_real=y-y_old;
        point_y_another.Point3D=y;



        //compute the derivatives
        dSigma0_dtheta0=find_dSigmai_dthetai(a0,theta0,s0);
        dSigma1_dtheta1=find_dSigmai_dthetai(a1,theta1,s1);
        dSigma2_dtheta2=find_dSigmai_dthetai(a2,theta2,s2);
        
        dT_dtheta0= find_dT_dthetai(dSigma0_dtheta0,C0,!Sigma0.Sphere5D);
        dT_dtheta1= find_dT_dthetai(dSigma1_dtheta1,C1,!Sigma1.Sphere5D);
        dT_dtheta2= find_dT_dthetai(dSigma2_dtheta2,C2,!Sigma2.Sphere5D);

        dP_dtheta0=find_dP_dthetai(T,dT_dtheta0);
        dP_dtheta1=find_dP_dthetai(T,dT_dtheta1);
        dP_dtheta2=find_dP_dthetai(T,dT_dtheta2);

        dY_dtheta0=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta0,-1f*dT_dtheta0);
        dY_dtheta1=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta1,-1f*dT_dtheta1);
        dY_dtheta2=find_dY_dthetai(~T,~P,~P_d,~dP_dtheta2,-1f*dT_dtheta2);

        // check ExtractPntAfromPntPairs!!!!
        CGA.CGA unnormedY = ExtractPntBfromPntPairs(T, false);
        dy_dtheta0=-1f*pnt_to_vector(find_dy_dthetai(dY_dtheta0, unnormedY));
        dy_dtheta1=pnt_to_vector(find_dy_dthetai(dY_dtheta1, unnormedY));
        dy_dtheta2=pnt_to_vector(find_dy_dthetai(dY_dtheta2, unnormedY));


        // double[] dy_dtheta0_arr = vector_to_array(dy_dtheta0);
        // double[] dy_dtheta1_arr = vector_to_array(dy_dtheta1);
        // double[] dy_dtheta2_arr = vector_to_array(dy_dtheta2);

        // mat[0]=dy_dtheta0_arr;mat[1]=dy_dtheta1_arr;mat[2]=dy_dtheta2_arr;
        
        var dy_dt_1=dy_dtheta0[0]*dthetai_dt_vec[0]+dy_dtheta1[0]*dthetai_dt_vec[1]+dy_dtheta2[0]*dthetai_dt_vec[2];
        var dy_dt_2=dy_dtheta0[1]*dthetai_dt_vec[0]+dy_dtheta1[1]*dthetai_dt_vec[1]+dy_dtheta2[1]*dthetai_dt_vec[2];
        var dy_dt_3=dy_dtheta0[2]*dthetai_dt_vec[0]+dy_dtheta1[2]*dthetai_dt_vec[1]+dy_dtheta2[2]*dthetai_dt_vec[2];
        var dy_dt_cal=new Vector3(dy_dt_1,dy_dt_2,dy_dt_3);
        point_y.Point3D+=dy_dt_cal;

        // Debug.Log("dy_dt_error");
        // Debug.Log(dy_dt_cal-dy_dt_real);
        
        point_y.UpdateGameObject();
        point_y_another.UpdateGameObject();

        timer+=Mathf.PI*0.05f;
    }
}
