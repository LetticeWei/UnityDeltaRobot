using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
using System;
using static ShapeClasses;
public class PathRegulator2 : MonoBehaviour{
   public Sphere Sigma0,Sigma1,Sigma2;
    public Point PointX0,PointX1,PointX2, Elbow0,Elbow1,Elbow2, Base0,Base1,Base2,Elbow0_,Elbow1_,Elbow2_;
    public float rb,l,rou,re, theta0,theta1,theta2,theta0_,theta1_,theta2_ ;
    public Vector3 x0,x1,x2,y,a0,a1,a2,s0,s1,s2;
    public Vector3 dx0_dt,dx1_dt,dx2_dt,dy_dt;
    public CGA.CGA dSigma0_dalphax,dSigma1_dalphax,dSigma2_dalphax,dSigma0_dalphay,dSigma1_dalphay,dSigma2_dalphay,dSigma0_dalphaz,dSigma1_dalphaz,dSigma2_dalphaz,C0,C1,C2,T0,T1,T2,A0,A1,A2;

    public CGA.CGA dA0_dt,dA1_dt,dA2_dt,da0_dt,da1_dt,da2_dt,z0,z1,z2,dy_dalphax,dy_dalphay,dy_dalphaz;

    public CGA.CGA dA0_dalphax,dA0_dalphay,dA0_dalphaz,dA1_dalphax,dA1_dalphay,dA1_dalphaz,dA2_dalphax,dA2_dalphay,dA2_dalphaz; 
    public CGA.CGA dx0_dalphax,dx0_dalphay,dx0_dalphaz,dx1_dalphax,dx1_dalphay,dx1_dalphaz,dx2_dalphax,dx2_dalphay,dx2_dalphaz;
    public CGA.CGA da0_dalphax, da0_dalphay,da0_dalphaz,da1_dalphax, da1_dalphay,da1_dalphaz,da2_dalphax, da2_dalphay,da2_dalphaz;
    public CGA.CGA dT0_dalphax,dT0_dalphay,dT0_dalphaz, dT1_dalphax,dT1_dalphay,dT1_dalphaz, dT2_dalphax,dT2_dalphay,dT2_dalphaz;

    public float dtheta0_dalphax,dtheta0_dalphay,dtheta0_dalphaz,dtheta1_dalphax,dtheta1_dalphay,dtheta1_dalphaz,dtheta2_dalphax,dtheta2_dalphay,dtheta2_dalphaz;
    public float dtheta0_dt,dtheta1_dt,dtheta2_dt;
    public float timer;
    // Start is called before the first frame update
    public CGA.CGA find_dSigmai_dalpha(CGA.CGA dxi_dt, Vector3 xi){
        CGA.CGA find_dSigmai_dalpha=(dxi_dt|vector_to_pnt(xi))[0]* ei + dxi_dt;
        return find_dSigmai_dalpha;
    }

    public CGA.CGA find_Ti(CGA.CGA Sigmai5D,CGA.CGA Ci){
        return !((!Sigmai5D)^Ci);
    }
    public CGA.CGA find_dTi_dalpha(CGA.CGA dSigmai_dt,CGA.CGA Ci){
        return !(dSigmai_dt^Ci);
    }
    public CGA.CGA find_dAi_dalpha(CGA.CGA T,CGA.CGA P,CGA.CGA P_d,CGA.CGA dP_dthetai,CGA.CGA dT_dthetai){
        var dP_d_dthetai=~dP_dthetai;
        CGA.CGA part1= -1f*dP_d_dthetai*(T|ei)*P;
        CGA.CGA part2=-1f*P_d*(dT_dthetai|ei)*P;
        CGA.CGA part3=-1f*P_d*(T|ei)*dP_dthetai;
        return get_grade_1(part1+part2+part3);
    }


    public CGA.CGA find_dai_dalpha(CGA.CGA dAi_dt, CGA.CGA Ai){
        float denom = (Ai|ei)[0]*(Ai|ei)[0];
        CGA.CGA left=one-one;  //zero vector
        CGA.CGA right=one-one;  //zero vector
        CGA.CGA[] basis_array= {e1, e2, e3};
        foreach (CGA.CGA b in basis_array){
          left +=-1f*(dAi_dt|b)*b*(Ai|ei); 
          right+= (Ai|b)*b*(dAi_dt|ei);}
        return (left+right)*(1f/denom);
    }
    public CGA.CGA find_zi(CGA.CGA ai, Vector3 si){
        var si_cga=vector_to_pnt(si);
        var zi=ai-rb*si_cga;
        return zi;
    }
    public float find_thetai(CGA.CGA zi,Vector3 si){
        var si_cga=vector_to_pnt(si);
        return Mathf.Atan((zi|(-1f*e2))[0]/(zi|si_cga)[0]);
    }
    public float find_dthetai_dalpha(CGA.CGA zi, CGA.CGA dzi_dt,Vector3 si){
        var si_cga=vector_to_pnt(si);
        var denom=(zi|zi)[0];
        var nume=(zi|si_cga)[0]*(dzi_dt|(-1f*e2))[0]-(zi|(-1f*e2))[0]*(dzi_dt|si_cga)[0];
        return nume/denom;
    }

    public CGA.CGA find_dP_dthetai(CGA.CGA T,CGA.CGA dT_dthetai){
        float beta = Mathf.Sqrt((T|T)[0]);
        CGA.CGA dbeta_dthetai = (dT_dthetai|T)*(1f/beta);
        CGA.CGA dP_dthetai = (dT_dthetai|T)*(1f/beta);
        dP_dthetai = 0.5f*(beta*dT_dthetai - T*dbeta_dthetai)*(1f/(beta*beta));
        return dP_dthetai;
    }

    public Vector3 findEndPoint_ai(float thetai,Vector3 si){
        var si_cga=vector_to_pnt(si);
        CGA.CGA ai=(rb-re+l*Mathf.Cos(thetai))*si_cga+l*Mathf.Sin(thetai)*(-1f*e2);
        return pnt_to_vector(ai);
    }
    public (CGA.CGA, CGA.CGA) find_P(CGA.CGA T){ //T is a point pair
        var beta=(float) Mathf.Sqrt((T*T)[0]);
        var F=1.0f/beta*T;
        var P=0.5f*(one +F);
        var P_d=0.5f*(one -F);
        return (P,P_d);
    }
    public CGA.CGA find_Ai(CGA.CGA Ti, CGA.CGA P, CGA.CGA P_d){ // may not be used... 
        return -1*P_d*(Ti|ei)[0]*P;
    }
    public CGA.CGA find_Ci(Vector3 si){
        var si_cga=vector_to_pnt(si);
        var res=up(rb*si.x, rb*si.y,rb*si.z);
        CGA.CGA temp_Sphere_dual= res -0.5f*l*l*ei; 
        CGA.CGA Ci = temp_Sphere_dual^(I3*((-1f*e2)^si_cga));
        return Ci;
    }
    void Start()
    {   //set this platform velocity
        dy_dt=new Vector3(1f,0,0);

        rb=4f;l=5f;rou=9f;re=1f;
        y=new Vector3(0,-12f,0);
        s0=new Vector3(1,0,0);
        s1=new Vector3(-1f/2f,0,-1f/2f*Mathf.Sqrt(3f));
        s2=new Vector3(-1f/2f,0,1f/2f*Mathf.Sqrt(3f));
        x0=y+re*s0;
        x1=y+re*s1;
        x2=y+re*s2;

        C0=find_Ci(s0);C1=find_Ci(s1);C2=find_Ci(s2);

        Base0=new Point();Base0.Point3D=re*s0;Base0.SetupGameObject(0.7f,0.3f,0.0f);
        Base1=new Point();Base1.Point3D=re*s1;Base1.SetupGameObject(0.7f,0.3f,0.0f);
        Base2=new Point();Base2.Point3D=re*s2;Base2.SetupGameObject(0.7f,0.3f,0.0f);


        PointX0=new Point();PointX0.Point3D=x0;PointX0.SetupGameObject(0.1f,0.3f,0.7f);
        PointX1=new Point();PointX1.Point3D=x1;PointX1.SetupGameObject(0.1f,0.3f,0.7f);
        PointX2=new Point();PointX2.Point3D=x2;PointX2.SetupGameObject(0.1f,0.3f,0.7f);

        Sigma0=new Sphere();Sigma0.SphereObj.active = false;Sigma0.Centre=x0;Sigma0.Radius= rou;Sigma0.FindSphere5DbyCandRou();
        Sigma1=new Sphere();Sigma1.SphereObj.active = false;Sigma1.Centre=x1;Sigma1.Radius= rou;Sigma1.FindSphere5DbyCandRou();
        Sigma2=new Sphere();Sigma2.SphereObj.active = false;Sigma2.Centre=x2;Sigma2.Radius= rou;Sigma2.FindSphere5DbyCandRou();

        T0=find_Ti(Sigma0.Sphere5D,C0);T1=find_Ti(Sigma1.Sphere5D,C1); T2=find_Ti(Sigma2.Sphere5D,C2);

        A0= ExtractPntBfromPntPairs(T0);A1= ExtractPntBfromPntPairs(T1);A2= ExtractPntBfromPntPairs(T2);
        a0=pnt_to_vector(down(A0));a1=pnt_to_vector(down(A1));a2=pnt_to_vector(down(A2));
        z0=find_zi(vector_to_pnt(a0), s0);z1=find_zi(vector_to_pnt(a1), s1);z2=find_zi(vector_to_pnt(a2), s2);
        theta0=theta0_=find_thetai(z0,s0);theta1=theta1_=find_thetai(z1,s1);theta2=theta2_=find_thetai(z2,s2);

        Elbow0=new Point();Elbow0.Point3D=a0+re*s0;Elbow0.SetupGameObject(0.0f,0.4f,0.2f);
        Elbow1=new Point();Elbow1.Point3D=a1+re*s1;Elbow1.SetupGameObject(0.0f,0.4f,0.2f);
        Elbow2=new Point();Elbow2.Point3D=a2+re*s2;Elbow2.SetupGameObject(0.0f,0.4f,0.2f);

        Elbow0_=new Point();Elbow0_.Point3D=Elbow0.Point3D;Elbow0_.SetupGameObject(0.3f,0.3f,0.3f);
        Elbow1_=new Point();Elbow1_.Point3D=Elbow1.Point3D;Elbow1_.SetupGameObject(0.3f,0.3f,0.3f);
        Elbow2_=new Point();Elbow2_.Point3D=Elbow2.Point3D;Elbow2_.SetupGameObject(0.3f,0.3f,0.3f);


        dy_dalphax=dx0_dalphax=dx1_dalphax=dx2_dalphax=e1;
        dy_dalphay=dx0_dalphay=dx1_dalphay=dx2_dalphay=-1f*e2;
        dy_dalphaz=dx0_dalphaz=dx1_dalphaz=dx2_dalphaz=e3;

        dx0_dt=dx1_dt=dx2_dt=dy_dt; // can be removed

        dSigma0_dalphax=find_dSigmai_dalpha(dx0_dalphax,x0);
        dSigma1_dalphax=find_dSigmai_dalpha(dx1_dalphax,x1);
        dSigma2_dalphax=find_dSigmai_dalpha(dx2_dalphax,x2);

        dSigma0_dalphay=find_dSigmai_dalpha(dx0_dalphay,x0);
        dSigma1_dalphay=find_dSigmai_dalpha(dx1_dalphay,x1);
        dSigma2_dalphay=find_dSigmai_dalpha(dx2_dalphay,x2);

        dSigma0_dalphaz=find_dSigmai_dalpha(dx0_dalphaz,x0);
        dSigma1_dalphaz=find_dSigmai_dalpha(dx1_dalphaz,x1);
        dSigma2_dalphaz=find_dSigmai_dalpha(dx2_dalphaz,x2);


        dT0_dalphax=find_dTi_dalpha(dSigma0_dalphax,C0);
        dT0_dalphay=find_dTi_dalpha(dSigma0_dalphay,C0);
        dT0_dalphaz=find_dTi_dalpha(dSigma0_dalphaz,C0);

        dT1_dalphax=find_dTi_dalpha(dSigma1_dalphax,C1);
        dT1_dalphay=find_dTi_dalpha(dSigma1_dalphay,C1);
        dT1_dalphaz=find_dTi_dalpha(dSigma1_dalphaz,C1);
        
        dT2_dalphax=find_dTi_dalpha(dSigma2_dalphax,C2);
        dT2_dalphay=find_dTi_dalpha(dSigma2_dalphay,C2);
        dT2_dalphaz=find_dTi_dalpha(dSigma2_dalphaz,C2);

        var result = find_P(T0);
        var P=result.Item1;var P_d=result.Item2;
        var dP0_dalphax=find_dP_dthetai(T0,dT0_dalphax);
        var dP0_dalphay=find_dP_dthetai(T0,dT0_dalphay);
        var dP0_dalphaz=find_dP_dthetai(T0,dT0_dalphaz);

        dA0_dalphax=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphax,-1f*dT0_dalphax);
        dA0_dalphay=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphay,-1f*dT0_dalphay);
        dA0_dalphaz=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphaz,-1f*dT0_dalphaz);

        result = find_P(T1);
        P=result.Item1;P_d=result.Item2;
        var dP1_dalphax=find_dP_dthetai(T1,dT1_dalphax);
        var dP1_dalphay=find_dP_dthetai(T1,dT1_dalphay);
        var dP1_dalphaz=find_dP_dthetai(T1,dT1_dalphaz);

        dA1_dalphax=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphax,-1f*dT1_dalphax);
        dA1_dalphay=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphay,-1f*dT1_dalphay);
        dA1_dalphaz=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphaz,-1f*dT1_dalphaz);

                
        result = find_P(T2);
        P=result.Item1;P_d=result.Item2;
        var dP2_dalphax=find_dP_dthetai(T2,dT2_dalphax);
        var dP2_dalphay=find_dP_dthetai(T2,dT2_dalphay);
        var dP2_dalphaz=find_dP_dthetai(T2,dT2_dalphaz);

        dA2_dalphax=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphax,-1f*dT2_dalphax);  
        dA2_dalphay=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphay,-1f*dT2_dalphay);
        dA2_dalphaz=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphaz,-1f*dT2_dalphaz);

        CGA.CGA unnormedA0= ExtractPntBfromPntPairs(T0,false);
        CGA.CGA unnormedA1= ExtractPntBfromPntPairs(T1, false);
        CGA.CGA unnormedA2= ExtractPntBfromPntPairs(T2,false);

        da0_dalphax=-1f*find_dai_dalpha(dA0_dalphax, unnormedA0);
        da0_dalphay=find_dai_dalpha(dA0_dalphay, unnormedA0);
        da0_dalphaz=-1f*find_dai_dalpha(dA0_dalphaz, unnormedA0); 
        da1_dalphax=-1f*find_dai_dalpha(dA1_dalphax, unnormedA1);
        da1_dalphay=find_dai_dalpha(dA1_dalphay, unnormedA1);
        da1_dalphaz=-1f*find_dai_dalpha(dA1_dalphaz, unnormedA1); 
        da2_dalphax=-1f*find_dai_dalpha(dA2_dalphax, unnormedA2);
        da2_dalphay=find_dai_dalpha(dA2_dalphay, unnormedA2);
        da2_dalphaz=-1f*find_dai_dalpha(dA2_dalphaz, unnormedA2); 


        dtheta0_dalphax=find_dthetai_dalpha(z0, da0_dalphax,s0);
        dtheta0_dalphay=find_dthetai_dalpha(z0, da0_dalphay,s0);
        dtheta0_dalphaz=find_dthetai_dalpha(z0, da0_dalphaz,s0);

        dtheta1_dalphax=find_dthetai_dalpha(z1, da1_dalphax,s1);
        dtheta1_dalphay=find_dthetai_dalpha(z1, da1_dalphay,s1);
        dtheta1_dalphaz=find_dthetai_dalpha(z1, da1_dalphaz,s1);

        dtheta2_dalphax=find_dthetai_dalpha(z2, da2_dalphax,s2);
        dtheta2_dalphay=find_dthetai_dalpha(z2, da2_dalphay,s2);
        dtheta2_dalphaz=find_dthetai_dalpha(z2, da2_dalphaz,s2);

        dtheta0_dt=dtheta0_dalphax*dy_dt.x+dtheta0_dalphay*dy_dt.y+dtheta0_dalphaz*dy_dt.z;
        dtheta1_dt=dtheta1_dalphax*dy_dt.x+dtheta1_dalphay*dy_dt.y+dtheta1_dalphaz*dy_dt.z;
        dtheta2_dt=dtheta2_dalphax*dy_dt.x+dtheta2_dalphay*dy_dt.y+dtheta2_dalphaz*dy_dt.z;

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //update the framework according to dy_dt
        dy_dt=new Vector3((float) Mathf.Cos(timer)*0.05f,(float) Mathf.Cos(timer*0.6f)*(-0.1f),(float) Mathf.Cos(timer*1.2f)*0.05f);
        // dy_dt=new Vector3(0,0,(float) Mathf.Cos(timer)*0.05f);
        y+=dy_dt;

        x0=y+re*s0;
        x1=y+re*s1;
        x2=y+re*s2;

        PointX0.Point3D=x0;PointX0.UpdateGameObject();
        PointX1.Point3D=x1;PointX1.UpdateGameObject();
        PointX2.Point3D=x2;PointX2.UpdateGameObject();

        Sigma0.Centre=x0;Sigma0.FindSphere5DbyCandRou();
        Sigma1.Centre=x1;Sigma1.FindSphere5DbyCandRou();
        Sigma2.Centre=x2;Sigma2.FindSphere5DbyCandRou();

        T0=find_Ti(Sigma0.Sphere5D,C0);T1=find_Ti(Sigma1.Sphere5D,C1); T2=find_Ti(Sigma2.Sphere5D,C2);

        A0= ExtractPntBfromPntPairs(T0);A1= ExtractPntBfromPntPairs(T1);A2= ExtractPntBfromPntPairs(T2);
        a0=pnt_to_vector(down(A0));a1=pnt_to_vector(down(A1));a2=pnt_to_vector(down(A2));
        z0=find_zi(vector_to_pnt(a0), s0);z1=find_zi(vector_to_pnt(a1), s1);z2=find_zi(vector_to_pnt(a2), s2);
        theta0=find_thetai(z0,s0);theta1=find_thetai(z1,s1);theta2=find_thetai(z2,s2);
        
        dSigma0_dalphax=find_dSigmai_dalpha(dx0_dalphax,x0);
        dSigma1_dalphax=find_dSigmai_dalpha(dx1_dalphax,x1);
        dSigma2_dalphax=find_dSigmai_dalpha(dx2_dalphax,x2);

        dSigma0_dalphay=find_dSigmai_dalpha(dx0_dalphay,x0);
        dSigma1_dalphay=find_dSigmai_dalpha(dx1_dalphay,x1);
        dSigma2_dalphay=find_dSigmai_dalpha(dx2_dalphay,x2);

        dSigma0_dalphaz=find_dSigmai_dalpha(dx0_dalphaz,x0);
        dSigma1_dalphaz=find_dSigmai_dalpha(dx1_dalphaz,x1);
        dSigma2_dalphaz=find_dSigmai_dalpha(dx2_dalphaz,x2);


        dT0_dalphax=find_dTi_dalpha(dSigma0_dalphax,C0);
        dT0_dalphay=find_dTi_dalpha(dSigma0_dalphay,C0);
        dT0_dalphaz=find_dTi_dalpha(dSigma0_dalphaz,C0);

        dT1_dalphax=find_dTi_dalpha(dSigma1_dalphax,C1);
        dT1_dalphay=find_dTi_dalpha(dSigma1_dalphay,C1);
        dT1_dalphaz=find_dTi_dalpha(dSigma1_dalphaz,C1);
        
        dT2_dalphax=find_dTi_dalpha(dSigma2_dalphax,C2);
        dT2_dalphay=find_dTi_dalpha(dSigma2_dalphay,C2);
        dT2_dalphaz=find_dTi_dalpha(dSigma2_dalphaz,C2);

        var result = find_P(T0);
        var P=result.Item1;var P_d=result.Item2;
        var dP0_dalphax=find_dP_dthetai(T0,dT0_dalphax);
        var dP0_dalphay=find_dP_dthetai(T0,dT0_dalphay);
        var dP0_dalphaz=find_dP_dthetai(T0,dT0_dalphaz);

        dA0_dalphax=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphax,-1f*dT0_dalphax);
        dA0_dalphay=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphay,-1f*dT0_dalphay);
        dA0_dalphaz=find_dAi_dalpha(~T0,~P,~P_d,~dP0_dalphaz,-1f*dT0_dalphaz);

        result = find_P(T1);
        P=result.Item1;P_d=result.Item2;
        var dP1_dalphax=find_dP_dthetai(T1,dT1_dalphax);
        var dP1_dalphay=find_dP_dthetai(T1,dT1_dalphay);
        var dP1_dalphaz=find_dP_dthetai(T1,dT1_dalphaz);

        dA1_dalphax=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphax,-1f*dT1_dalphax);
        dA1_dalphay=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphay,-1f*dT1_dalphay);
        dA1_dalphaz=find_dAi_dalpha(~T1,~P,~P_d,~dP1_dalphaz,-1f*dT1_dalphaz);

                
        result = find_P(T2);
        P=result.Item1;P_d=result.Item2;
        var dP2_dalphax=find_dP_dthetai(T2,dT2_dalphax);
        var dP2_dalphay=find_dP_dthetai(T2,dT2_dalphay);
        var dP2_dalphaz=find_dP_dthetai(T2,dT2_dalphaz);

        dA2_dalphax=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphax,-1f*dT2_dalphax);  
        dA2_dalphay=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphay,-1f*dT2_dalphay);
        dA2_dalphaz=find_dAi_dalpha(~T2,~P,~P_d,~dP2_dalphaz,-1f*dT2_dalphaz);


        CGA.CGA unnormedA0= ExtractPntBfromPntPairs(T0,false);
        CGA.CGA unnormedA1= ExtractPntBfromPntPairs(T1, false);
        CGA.CGA unnormedA2= ExtractPntBfromPntPairs(T2, false);

        da0_dalphax=-1f*find_dai_dalpha(dA0_dalphax, unnormedA0);
        da0_dalphay=find_dai_dalpha(dA0_dalphay, unnormedA0);
        da0_dalphaz=-1f*find_dai_dalpha(dA0_dalphaz, unnormedA0); 
        da1_dalphax=-1f*find_dai_dalpha(dA1_dalphax, unnormedA1);
        da1_dalphay=find_dai_dalpha(dA1_dalphay, unnormedA1);
        da1_dalphaz=-1f*find_dai_dalpha(dA1_dalphaz, unnormedA1); 
        da2_dalphax=-1f*find_dai_dalpha(dA2_dalphax, unnormedA2);
        da2_dalphay=find_dai_dalpha(dA2_dalphay, unnormedA2);
        da2_dalphaz=-1f*find_dai_dalpha(dA2_dalphaz, unnormedA2); 


        dtheta0_dalphax=find_dthetai_dalpha(z0, da0_dalphax,s0);
        dtheta0_dalphay=find_dthetai_dalpha(z0, da0_dalphay,s0);
        dtheta0_dalphaz=find_dthetai_dalpha(z0, da0_dalphaz,s0);

        dtheta1_dalphax=find_dthetai_dalpha(z1, da1_dalphax,s1);
        dtheta1_dalphay=find_dthetai_dalpha(z1, da1_dalphay,s1);
        dtheta1_dalphaz=find_dthetai_dalpha(z1, da1_dalphaz,s1);

        dtheta2_dalphax=find_dthetai_dalpha(z2, da2_dalphax,s2);
        dtheta2_dalphay=find_dthetai_dalpha(z2, da2_dalphay,s2);
        dtheta2_dalphaz=find_dthetai_dalpha(z2, da2_dalphaz,s2);

        dtheta0_dt=dtheta0_dalphax*dy_dt.x+dtheta0_dalphay*dy_dt.y+dtheta0_dalphaz*dy_dt.z;
        dtheta1_dt=dtheta1_dalphax*dy_dt.x+dtheta1_dalphay*dy_dt.y+dtheta1_dalphaz*dy_dt.z;
        dtheta2_dt=dtheta2_dalphax*dy_dt.x+dtheta2_dalphay*dy_dt.y+dtheta2_dalphaz*dy_dt.z;

        theta0_+=dtheta0_dt;
        theta1_+=dtheta1_dt;
        theta2_+=dtheta2_dt;

        Elbow0.Point3D=a0+re*s0;Elbow0.UpdateGameObject();
        Elbow1.Point3D=a1+re*s1;Elbow1.UpdateGameObject();
        Elbow2.Point3D=a2+re*s2;Elbow2.UpdateGameObject();

        Elbow0_.Point3D=find_elbow_3d(theta0_,s0);Elbow0_.UpdateGameObject();
        Elbow1_.Point3D=find_elbow_3d(theta1_,s1);Elbow1_.UpdateGameObject();
        Elbow2_.Point3D=find_elbow_3d(theta2_,s2);Elbow2_.UpdateGameObject();

        timer+=Mathf.PI*0.05f;
    }

    public Vector3 find_elbow_3d(float thetai_,Vector3 si){
        var si_cga=vector_to_pnt(si);
        var result_pnt= (rb+re+l*Mathf.Cos(thetai_))*si_cga+l*Mathf.Sin(thetai_)*(-1f*e2); //i feel there shouldnt be an re here
        return pnt_to_vector(result_pnt);
    }
}
