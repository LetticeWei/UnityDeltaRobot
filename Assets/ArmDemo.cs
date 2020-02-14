using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using CGA;
using static CGA.CGA;
using System;
using static ShapeClasses;
public class ArmDemo : MonoBehaviour
{
    // Start is called before the first frame update
    public static Point Wrist;

    public static Point Pivot;public static Circle PivotReach;public static Point ElbowJoint;public static Sphere LowerArmReach;
    public static Point IntersectPoint1;public static Point IntersectPoint2;public static CGA.CGA CSIntersection;


    public static Point PivotB; public static Circle PivotReachB;public static Point ElbowJointB;public static Sphere LowerArmReach2;
    public static Point IntersectPoint1B;public static Point IntersectPoint2B;public static CGA.CGA CSIntersectionB;

    public static Point PivotC;public static Circle PivotReachC;public static Point ElbowJointC;public static Sphere LowerArmReach3;
    public static Point IntersectPoint1C;public static Point IntersectPoint2C;public static CGA.CGA CSIntersectionC;

    public static Point PlatformVertexE;
    public static Point PlatformVertexF;
    public static Point PlatformVertexG;


    void Start()
    {
    Wrist=new Point();
    Wrist.Point3D=new Vector3(5f/3f*(float) Math.Sqrt(3f),-20,5f);
    Wrist.SetupGameObject(0.1f,0.7f,0.3f); //set the colour

    //Three fixed pivots at ground
    Pivot=new Point();
    Pivot.Point3D=new Vector3(0,0,0f);
    Pivot.SetupGameObject(0.1f,0.3f,0.7f); //set the colour

    PivotB=new Point();
    PivotB.Point3D=new Vector3(0,0,10);
    PivotB.SetupGameObject(0.1f,0.3f,0.7f); 

    PivotC=new Point();
    PivotC.Point3D=new Vector3(5f* (float) Math.Sqrt(3f) ,0,5f);
    PivotC.SetupGameObject(0.1f,0.3f,0.7f); 

    ElbowJoint=new Point();
    ElbowJoint.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointB=new Point();
    ElbowJointB.SetupGameObject(0.7f,0.5f,0.1f); //set the colour
    ElbowJointC=new Point();
    ElbowJointC.SetupGameObject(0.7f,0.5f,0.1f); //set the colour


    //create a circle class object with pivit position as centre, and arm length as radius
    PivotReach= new Circle(); //will be fixed all the time, should be supressed
    PivotReach.PlaneVisible=false;
    PivotReach.PointA=Pivot.Point3D+ new Vector3(0,10f,0);
    PivotReach.PointB=Pivot.Point3D+ new Vector3(0,-10f,0);
    PivotReach.PointC=Pivot.Point3D+ new Vector3(5f,0,5f* (float) Math.Sqrt(3f));
    PivotReach.initialiseCircle();

    PivotReachB= new Circle(); //will be fixed all the time, should be 
    PivotReachB.PlaneVisible=false;
    PivotReachB.PointA=PivotB.Point3D+ new Vector3(0,10f,0);
    PivotReachB.PointB=PivotB.Point3D+ new Vector3(0,-10f,0);
    PivotReachB.PointC=PivotB.Point3D+ new Vector3(5f,0,-5f* (float) Math.Sqrt(3f));
    PivotReachB.initialiseCircle();

    PivotReachC= new Circle(); //will be fixed all the time, should be 
    PivotReachC.PlaneVisible=false;
    PivotReachC.PointA=PivotC.Point3D+ new Vector3(0,10f,0);
    PivotReachC.PointB=PivotC.Point3D+ new Vector3(0,-10f,0);
    PivotReachC.PointC=PivotC.Point3D+ new Vector3(-10F,0,0);
    PivotReachC.initialiseCircle();

    PlatformVertexE=new Point();
    PlatformVertexE.Point3D=Wrist.Point3D+ new Vector3(-2.5f/(float) Math.Sqrt(3f),0,-2.5f);
    PlatformVertexE.SetupGameObject(1f,0.2f,0.1f); //set the colour

    PlatformVertexF=new Point();
    PlatformVertexF.Point3D=Wrist.Point3D+ new Vector3(-2.5f/(float) Math.Sqrt(3f),0,2.5f);
    PlatformVertexF.SetupGameObject(1f,0.2f,0.1f); //set the colour

    PlatformVertexG=new Point();
    PlatformVertexG.Point3D=Wrist.Point3D+ new Vector3(5f/(float) Math.Sqrt(3f),0,0);
    PlatformVertexG.SetupGameObject(1f,0.2f,0.1f); //set the colour

    //initialise a sphere that centre around the wrist with radius = length of lower arm
    LowerArmReach=new Sphere();
    LowerArmReach.Centre=PlatformVertexE.Point3D;
    LowerArmReach.Radius= 20.0f; //position of wrist relative to elbow joint
    LowerArmReach.FindSphere5DbyCandRou();

    LowerArmReach2=new Sphere();
    LowerArmReach2.Centre=PlatformVertexF.Point3D;
    LowerArmReach2.Radius= 20.0f; //position of wrist relative to elbow joint
    LowerArmReach2.FindSphere5DbyCandRou();

    LowerArmReach3=new Sphere();
    LowerArmReach3.Centre=PlatformVertexG.Point3D;
    LowerArmReach3.Radius= 20.0f; //position of wrist relative to elbow joint
    LowerArmReach3.FindSphere5DbyCandRou();


    IntersectPoint1=new Point();
    IntersectPoint2=new Point();
    IntersectPoint1.SetupGameObject(0,1f,0.1f);
    IntersectPoint2.SetupGameObject(0,1f,0.1f);
    IntersectPoint1.GameObj.active = false;
    IntersectPoint2.GameObj.active = false;

    IntersectPoint1B=new Point();
    IntersectPoint2B=new Point();
    IntersectPoint1B.SetupGameObject(0,1f,0.1f);
    IntersectPoint2B.SetupGameObject(0,1f,0.1f);
    IntersectPoint1B.GameObj.active = false;
    IntersectPoint2B.GameObj.active = false;

    IntersectPoint1C=new Point();
    IntersectPoint2C=new Point();
    IntersectPoint1C.SetupGameObject(0,1f,0.1f);
    IntersectPoint2C.SetupGameObject(0,1f,0.1f);
    IntersectPoint1C.GameObj.active = false;
    IntersectPoint2C.GameObj.active = false;

    }

    // Update is called once per frame
    void Update()
    {
    //take the position of the wrist as the centre of a Sphere class object
    Wrist.UpdatePoint3DfromObject();
    PlatformVertexE.Point3D=Wrist.Point3D+ new Vector3(-2.5f/(float) Math.Sqrt(3f),0,-2.5f);
    PlatformVertexF.Point3D=Wrist.Point3D+ new Vector3(-2.5f/(float) Math.Sqrt(3f),0,2.5f);
    PlatformVertexG.Point3D=Wrist.Point3D+ new Vector3(5f/(float) Math.Sqrt(3f),0,0);
    PlatformVertexE.UpdateGameObject();
    PlatformVertexF.UpdateGameObject();
    PlatformVertexG.UpdateGameObject();

    LowerArmReach.Centre=PlatformVertexE.Point3D;
    LowerArmReach.FindSphere5DbyCandRou(); //update the 5d expression of the sphere obj
    LowerArmReach2.Centre=PlatformVertexF.Point3D;
    LowerArmReach2.FindSphere5DbyCandRou();
    LowerArmReach3.Centre=PlatformVertexG.Point3D;
    LowerArmReach3.FindSphere5DbyCandRou();


    CSIntersection=Intersection5D(PivotReach.Circle5D,LowerArmReach.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersection*CSIntersection)>0){//two point intersection
        IntersectPoint1.Shape5D=ExtractPntAfromPntPairs(CSIntersection);
        IntersectPoint1.FindPoint3Dfrom5D();
        IntersectPoint1.UpdateGameObject();
        
        IntersectPoint2.Shape5D=ExtractPntBfromPntPairs(CSIntersection);
        IntersectPoint2.FindPoint3Dfrom5D();
        IntersectPoint2.UpdateGameObject();
        ElbowJoint.GameObj.transform.position=IntersectPoint2.GameObj.transform.position;
    }

        CSIntersectionB=Intersection5D(PivotReachB.Circle5D,LowerArmReach2.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionB*CSIntersectionB)>0){//two point intersection
        IntersectPoint1B.Shape5D=ExtractPntAfromPntPairs(CSIntersectionB);
        IntersectPoint1B.FindPoint3Dfrom5D();
        IntersectPoint1B.UpdateGameObject();
        
        IntersectPoint2B.Shape5D=ExtractPntBfromPntPairs(CSIntersectionB);
        IntersectPoint2B.FindPoint3Dfrom5D();
        IntersectPoint2B.UpdateGameObject();
        ElbowJointB.GameObj.transform.position=IntersectPoint2B.GameObj.transform.position;
    }

        CSIntersectionC=Intersection5D(PivotReachC.Circle5D,LowerArmReach3.Sphere5D);
    if (pnt_to_scalar_pnt(CSIntersectionC*CSIntersectionC)>0){//two point intersection
        IntersectPoint1C.Shape5D=ExtractPntAfromPntPairs(CSIntersectionC);
        IntersectPoint1C.FindPoint3Dfrom5D();
        IntersectPoint1C.UpdateGameObject();
        
        IntersectPoint2C.Shape5D=ExtractPntBfromPntPairs(CSIntersectionC);
        IntersectPoint2C.FindPoint3Dfrom5D();
        IntersectPoint2C.UpdateGameObject();
        ElbowJointC.GameObj.transform.position=IntersectPoint2C.GameObj.transform.position;
    }
    
    Debug.Log(IntersectPoint2C.GameObj.transform.position);

    }
}
