﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
using System;
using static ShapeClasses;
public class ArmDemo2 : MonoBehaviour
{
    // Start is called before the first frame update
    public float scale_factor;

    public float Angle_A;
    public float Angle_B;
    public float Angle_C;

    public static Point Wrist;

    public static Point Pivot;public static Circle PivotReachA1;public static Circle PivotReachA2;
    public static Point ElbowJointA;public static Point ElbowJointA1;public static Point ElbowJointA2;
    public static Sphere LowerArmReachD;
    

    public static Point PivotB; public static Circle PivotReachB1;public static Circle PivotReachB2;
    public static Point ElbowJointB;public static Point ElbowJointB1;public static Point ElbowJointB2;
    public static Sphere LowerArmReachE;

    public static Point PivotC;public static Circle PivotReachC1;public static Circle PivotReachC2;
    public static Point ElbowJointC;public static Point ElbowJointC1;public static Point ElbowJointC2;
    public static Sphere LowerArmReachF;

    public static Point PlatformVertexD;
    public static Point PlatformVertexE;
    public static Point PlatformVertexF;
    
    public static Point IntersectPointDA1_1;public static Point IntersectPointDA1_2;public static CGA.CGA CSIntersectionDA1;
    public static Point IntersectPointDB2_1;public static Point IntersectPointDB2_2;public static CGA.CGA CSIntersectionDB2;

    public static Point IntersectPointEB1_1;public static Point IntersectPointEB1_2;public static CGA.CGA CSIntersectionEB1;
    public static Point IntersectPointEC2_1;public static Point IntersectPointEC2_2;public static CGA.CGA CSIntersectionEC2;

    public static Point IntersectPointFC1_1;public static Point IntersectPointFC1_2;public static CGA.CGA CSIntersectionFC1;
    public static Point IntersectPointFA2_1;public static Point IntersectPointFA2_2;public static CGA.CGA CSIntersectionFA2;



    void Start()
    {
    scale_factor=0.3f;
    //update the scale of the robot model (seems unsuccessful)
    // GameObject rob = GameObject.Find ("DeltaRobot1");
    // rob.transform.localScale=new Vector3(1f,1f,1f)*scale_factor;

    Wrist=new Point();
    Wrist.Point3D=new Vector3(5f/3f*(float) Math.Sqrt(3f),-24,5f)*scale_factor;
    Wrist.SetupGameObject(0.1f,0.7f,0.3f); //set the colour

    //Three fixed pivots at ground
    Pivot=new Point();
    Pivot.Point3D=new Vector3(0,0,0f);
    Pivot.SetupGameObject(0.1f,0.3f,0.7f); //set the colour

    PivotB=new Point();
    PivotB.Point3D=new Vector3(0,0,10f)*scale_factor;
    PivotB.SetupGameObject(0.1f,0.3f,0.7f); 

    PivotC=new Point();
    PivotC.Point3D=new Vector3(5f* (float) Math.Sqrt(3f) ,0,5f)*scale_factor;
    PivotC.SetupGameObject(0.1f,0.3f,0.7f); 

    ElbowJointA=new Point();
    ElbowJointA.SetupGameObject(1f,0.2f,0.1f); //set the colour
    ElbowJointB=new Point();
    ElbowJointB.SetupGameObject(1f,0.2f,0.1f); //set the colour
    ElbowJointC=new Point();
    ElbowJointC.SetupGameObject(1f,0.2f,0.1f); //set the colour


    ElbowJointA1=new Point();
    ElbowJointA1.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointB1=new Point();
    ElbowJointB1.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointC1=new Point();
    ElbowJointC1.SetupGameObject(0.7f,0.5f,0.1f); //set the colour

    ElbowJointA2=new Point();
    ElbowJointA2.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointB2=new Point();
    ElbowJointB2.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointC2=new Point();
    ElbowJointC2.SetupGameObject(0.7f,0.5f,0.1f); //set the colour


    //create a circle class object with pivit position as centre, and arm length as radius
    PivotReachA1= new Circle(); //will be fixed all the time, should be supressed
    PivotReachA1.PlaneVisible=false;
    Vector3 AtoA1=new Vector3(-1f*(10*(float) Math.Sqrt(3f)-3)/8f,0f,(10-(float) Math.Sqrt(3f))/8f)*scale_factor;
    PivotReachA1.PointA=Pivot.Point3D+ new Vector3(0,10f,0)*scale_factor+AtoA1;
    PivotReachA1.PointB=Pivot.Point3D+ new Vector3(0,-10f,0)*scale_factor+AtoA1;
    PivotReachA1.PointC=Pivot.Point3D+ new Vector3(5f,0,5f* (float) Math.Sqrt(3f))*scale_factor+AtoA1;
    PivotReachA1.initialiseCircle();

    PivotReachA2= new Circle(); //will be fixed all the time, should be supressed
    PivotReachA2.PlaneVisible=false;
    //Vector3 AtoA2=new Vector3(-(10*(float) Math.Sqrt(3f)-3)/8f,0f,(10-(float) Math.Sqrt(3f))/8f);
    PivotReachA2.PointA=Pivot.Point3D+ new Vector3(0,10f,0)*scale_factor-AtoA1;
    PivotReachA2.PointB=Pivot.Point3D+ new Vector3(0,-10f,0)*scale_factor-AtoA1;
    PivotReachA2.PointC=Pivot.Point3D+ new Vector3(5f,0,5f* (float) Math.Sqrt(3f))*scale_factor-AtoA1;
    PivotReachA2.initialiseCircle();

    PivotReachB1= new Circle(); //will be fixed all the time, should be 
    PivotReachB1.PlaneVisible=false;
    Vector3 BtoB1=new Vector3((10*(float) Math.Sqrt(3f)-3)/8f,0f,(10-(float) Math.Sqrt(3f))/8f)*scale_factor;
    PivotReachB1.PointA=PivotB.Point3D+ new Vector3(0,10f,0)*scale_factor+BtoB1;
    PivotReachB1.PointB=PivotB.Point3D+ new Vector3(0,-10f,0)*scale_factor+BtoB1;
    PivotReachB1.PointC=PivotB.Point3D+ new Vector3(5f,0,-5f* (float) Math.Sqrt(3f))*scale_factor+BtoB1;
    PivotReachB1.initialiseCircle();

    PivotReachB2= new Circle(); //will be fixed all the time, should be 
    PivotReachB2.PlaneVisible=false;
    Vector3 BtoB2=new Vector3((10*(float) Math.Sqrt(3f)-3)/8f,0f,(10-(float) Math.Sqrt(3f))/8f)*scale_factor;
    PivotReachB2.PointA=PivotB.Point3D+ new Vector3(0,10f,0)*scale_factor-BtoB1;
    PivotReachB2.PointB=PivotB.Point3D+ new Vector3(0,-10f,0)*scale_factor-BtoB1;
    PivotReachB2.PointC=PivotB.Point3D+ new Vector3(5f,0,-5f* (float) Math.Sqrt(3f))*scale_factor-BtoB1;
    PivotReachB2.initialiseCircle();

    PivotReachC1= new Circle(); //will be fixed all the time, should be 
    PivotReachC1.PlaneVisible=false;
    Vector3 CtoC2=new Vector3(0,0,-1f*(10-(float) Math.Sqrt(3f))/4f)*scale_factor;
    PivotReachC1.PointA=PivotC.Point3D+ new Vector3(0,10f,0)*scale_factor+CtoC2;
    PivotReachC1.PointB=PivotC.Point3D+ new Vector3(0,-10f,0)*scale_factor+CtoC2;
    PivotReachC1.PointC=PivotC.Point3D+ new Vector3(-10F,0,0)*scale_factor+CtoC2;
    PivotReachC1.initialiseCircle();

    PivotReachC2= new Circle(); //will be fixed all the time, should be 
    PivotReachC2.PlaneVisible=false;
    PivotReachC2.PointA=PivotC.Point3D+ new Vector3(0,10f,0)*scale_factor-CtoC2;
    PivotReachC2.PointB=PivotC.Point3D+ new Vector3(0,-10f,0)*scale_factor-CtoC2;
    PivotReachC2.PointC=PivotC.Point3D+ new Vector3(-10F,0,0)*scale_factor-CtoC2;
    PivotReachC2.initialiseCircle();

    PlatformVertexD=new Point();
    PlatformVertexD.Point3D=Wrist.Point3D+ new Vector3((3f-10*(float) Math.Sqrt(3f))/6f,0,0)*scale_factor;
    PlatformVertexD.SetupGameObject(1f,0.2f,0.1f); //set the colour

    PlatformVertexE=new Point();
    PlatformVertexE.Point3D=Wrist.Point3D+ new Vector3((-3f+10*(float) Math.Sqrt(3f))/12f,0,(10f-(float) Math.Sqrt(3f))/4f)*scale_factor;
    PlatformVertexE.SetupGameObject(1f,0.2f,0.1f); //set the colour

    PlatformVertexF=new Point();
    PlatformVertexF.Point3D=Wrist.Point3D+ new Vector3((-3f+10*(float) Math.Sqrt(3f))/12f,0,-1f*(10f-(float) Math.Sqrt(3f))/4f)*scale_factor;
    PlatformVertexF.SetupGameObject(1f,0.2f,0.1f); //set the colour


    //initialise a sphere that centre around the wrist with radius = length of lower arm
    LowerArmReachD=new Sphere();
    LowerArmReachD.SphereObj.active = false;
    LowerArmReachD.Centre=PlatformVertexD.Point3D;
    LowerArmReachD.Radius= 21.96040974f*scale_factor; //position of wrist relative to elbow joint
    LowerArmReachD.FindSphere5DbyCandRou();

    LowerArmReachE=new Sphere();
    LowerArmReachE.SphereObj.active = false;
    LowerArmReachE.Centre=PlatformVertexE.Point3D;
    LowerArmReachE.Radius= 21.96040974f*scale_factor; //position of wrist relative to elbow joint
    LowerArmReachE.FindSphere5DbyCandRou();

    LowerArmReachF=new Sphere();
    LowerArmReachF.SphereObj.active = false;
    LowerArmReachF.Centre=PlatformVertexF.Point3D;
    LowerArmReachF.Radius= 21.96040974f*scale_factor; //position of wrist relative to elbow joint
    LowerArmReachF.FindSphere5DbyCandRou();

    IntersectPointDA1_1=new Point();
    IntersectPointDA1_2=new Point();
    IntersectPointDA1_1.SetupGameObject(0,1f,0.1f);
    IntersectPointDA1_2.SetupGameObject(0,1f,0.1f);
    IntersectPointDA1_1.GameObj.active = false;
    IntersectPointDA1_2.GameObj.active = false;

    IntersectPointDB2_1=new Point();
    IntersectPointDB2_2=new Point();
    IntersectPointDB2_1.SetupGameObject(0,1f,0.1f);
    IntersectPointDB2_2.SetupGameObject(0,1f,0.1f);
    IntersectPointDB2_1.GameObj.active = false;
    IntersectPointDB2_2.GameObj.active = false;

    IntersectPointEB1_1=new Point();
    IntersectPointEB1_2=new Point();
    IntersectPointEB1_1.SetupGameObject(0,1f,0.1f);
    IntersectPointEB1_2.SetupGameObject(0,1f,0.1f);
    IntersectPointEB1_1.GameObj.active = false;
    IntersectPointEB1_2.GameObj.active = false;

    IntersectPointEC2_1=new Point();
    IntersectPointEC2_2=new Point();
    IntersectPointEC2_1.SetupGameObject(0,1f,0.1f);
    IntersectPointEC2_2.SetupGameObject(0,1f,0.1f);
    IntersectPointEC2_1.GameObj.active = false;
    IntersectPointEC2_2.GameObj.active = false;

    IntersectPointFC1_1=new Point();
    IntersectPointFC1_2=new Point();
    IntersectPointFC1_1.SetupGameObject(0,1f,0.1f);
    IntersectPointFC1_2.SetupGameObject(0,1f,0.1f);
    IntersectPointFC1_1.GameObj.active = false;
    IntersectPointFC1_2.GameObj.active = false;

    IntersectPointFA2_1=new Point();
    IntersectPointFA2_2=new Point();
    IntersectPointFA2_1.SetupGameObject(0,1f,0.1f);
    IntersectPointFA2_2.SetupGameObject(0,1f,0.1f);
    IntersectPointFA2_1.GameObj.active = false;
    IntersectPointFA2_2.GameObj.active = false;
    }

    // Update is called once per frame
    void Update()
    {
    //take the position of the wrist as the centre of a Sphere class object
    Wrist.UpdatePoint3DfromObject();

    PlatformVertexD.Point3D=Wrist.Point3D+ new Vector3((3f-10*(float) Math.Sqrt(3f))/6f,0,0)*scale_factor;
    PlatformVertexE.Point3D=Wrist.Point3D+ new Vector3((-3f+10*(float) Math.Sqrt(3f))/12f,0,(10f-(float) Math.Sqrt(3f))/4f)*scale_factor;
    PlatformVertexF.Point3D=Wrist.Point3D+ new Vector3((-3f+10*(float) Math.Sqrt(3f))/12f,0,-1f*(10f-(float) Math.Sqrt(3f))/4f)*scale_factor;
    PlatformVertexD.UpdateGameObject();
    PlatformVertexE.UpdateGameObject();
    PlatformVertexF.UpdateGameObject();

    LowerArmReachD.Centre=PlatformVertexD.Point3D;
    LowerArmReachD.FindSphere5DbyCandRou(); //update the 5d expression of the sphere obj
    LowerArmReachE.Centre=PlatformVertexE.Point3D;
    LowerArmReachE.FindSphere5DbyCandRou();
    LowerArmReachF.Centre=PlatformVertexF.Point3D;
    LowerArmReachF.FindSphere5DbyCandRou();


    CSIntersectionDA1=Intersection5D(PivotReachA1.Circle5D,LowerArmReachD.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionDA1*CSIntersectionDA1)>0){//two point intersection
        IntersectPointDA1_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionDA1);
        IntersectPointDA1_1.FindPoint3Dfrom5D();
        IntersectPointDA1_1.UpdateGameObject();
        
        IntersectPointDA1_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionDA1);
        IntersectPointDA1_2.FindPoint3Dfrom5D();
        IntersectPointDA1_2.UpdateGameObject();
        ElbowJointA1.GameObj.transform.position=IntersectPointDA1_2.GameObj.transform.position;
        }
    else{
        Debug.Log("Cannot reach.");
    }
    CSIntersectionDB2=Intersection5D(PivotReachB2.Circle5D,LowerArmReachD.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionDB2*CSIntersectionDB2)>0){
        IntersectPointDB2_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionDB2);
        IntersectPointDB2_1.FindPoint3Dfrom5D();
        IntersectPointDB2_1.UpdateGameObject();
        
        IntersectPointDB2_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionDB2);
        IntersectPointDB2_2.FindPoint3Dfrom5D();
        IntersectPointDB2_2.UpdateGameObject();
        ElbowJointB2.GameObj.transform.position=IntersectPointDB2_2.GameObj.transform.position;
        }
    else{
        Debug.Log("Cannot reach.");
    }
    CSIntersectionEB1=Intersection5D(PivotReachB1.Circle5D,LowerArmReachE.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionEB1*CSIntersectionEB1)>0){
        IntersectPointEB1_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionEB1);
        IntersectPointEB1_1.FindPoint3Dfrom5D();
        IntersectPointEB1_1.UpdateGameObject();
        
        IntersectPointEB1_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionEB1);
        IntersectPointEB1_2.FindPoint3Dfrom5D();
        IntersectPointEB1_2.UpdateGameObject();
        ElbowJointB1.GameObj.transform.position=IntersectPointEB1_2.GameObj.transform.position;
        }
    else{
        Debug.Log("Cannot reach.");
    }
    CSIntersectionEC2=Intersection5D(PivotReachC2.Circle5D,LowerArmReachE.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionEC2*CSIntersectionEC2)>0){
        IntersectPointEC2_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionEC2);
        IntersectPointEC2_1.FindPoint3Dfrom5D();
        IntersectPointEC2_1.UpdateGameObject();
        
        IntersectPointEC2_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionEC2);
        IntersectPointEC2_2.FindPoint3Dfrom5D();
        IntersectPointEC2_2.UpdateGameObject();
        ElbowJointC2.GameObj.transform.position=IntersectPointEC2_2.GameObj.transform.position;
        }
    else{
        Debug.Log("Cannot reach.");
    }
    CSIntersectionFC1=Intersection5D(PivotReachC1.Circle5D,LowerArmReachF.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionFC1*CSIntersectionFC1)>0){
        IntersectPointFC1_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionFC1);
        IntersectPointFC1_1.FindPoint3Dfrom5D();
        IntersectPointFC1_1.UpdateGameObject();
        
        IntersectPointFC1_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionFC1);
        IntersectPointFC1_2.FindPoint3Dfrom5D();
        IntersectPointFC1_2.UpdateGameObject();
        ElbowJointC1.GameObj.transform.position=IntersectPointFC1_2.GameObj.transform.position;
        }
    else{
        Debug.Log("Cannot reach.");
    }
    CSIntersectionFA2=Intersection5D(PivotReachA2.Circle5D,LowerArmReachF.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionFA2*CSIntersectionFA2)>0){
        IntersectPointFA2_1.Shape5D=ExtractPntAfromPntPairs(CSIntersectionFA2);
        IntersectPointFA2_1.FindPoint3Dfrom5D();
        IntersectPointFA2_1.UpdateGameObject();
        
        IntersectPointFA2_2.Shape5D=ExtractPntBfromPntPairs(CSIntersectionFA2);
        IntersectPointFA2_2.FindPoint3Dfrom5D();
        IntersectPointFA2_2.UpdateGameObject();
        ElbowJointA2.GameObj.transform.position=IntersectPointFA2_2.GameObj.transform.position;
        }
    else{
        Debug.Log("<color=red>Error: Cannot reach");
    }

    ElbowJointA.GameObj.transform.position=(ElbowJointA1.GameObj.transform.position+ElbowJointA2.GameObj.transform.position)/2;
    ElbowJointB.GameObj.transform.position=(ElbowJointB1.GameObj.transform.position+ElbowJointB2.GameObj.transform.position)/2;
    ElbowJointC.GameObj.transform.position=(ElbowJointC1.GameObj.transform.position+ElbowJointC2.GameObj.transform.position)/2;

    Vector3 planeNormal=new Vector3(0,1f,0);
    

    //Find the angle for the two Vectors
    Angle_A = Vector3.Angle(Vector3.ProjectOnPlane(ElbowJointA.GameObj.transform.position-Pivot.Point3D, planeNormal), ElbowJointA.GameObj.transform.position-Pivot.Point3D);
    //Debug.Log(Angle_A);
    Angle_B = Vector3.Angle(Vector3.ProjectOnPlane(ElbowJointB.GameObj.transform.position-PivotB.Point3D, planeNormal), ElbowJointB.GameObj.transform.position-PivotB.Point3D);
    //Debug.Log(Angle_B);
    Angle_C = Vector3.Angle(Vector3.ProjectOnPlane(ElbowJointC.GameObj.transform.position-PivotC.Point3D, planeNormal),ElbowJointC.GameObj.transform.position-PivotC.Point3D);
    //Debug.Log(Angle_C);


    } 
}