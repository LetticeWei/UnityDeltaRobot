﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotConfigurator : MonoBehaviour
{   
    private GameObject sceneParent, UpperArms,UpperArms_A,UpperArms_B,UpperArms_C,ElbowJoints,ElbowJointA,ElbowJointB,ElbowJointC;
    public IList<Rigidbody> UpperArmRBList,ElbowJointList,LowerArmList,PlatformRBList;
    public IList<HingeJoint> UpperArmHingeJointList;
    public bool use_spring=true, use_motor=false, use_limit=false;

    public float springConst, damperConst, unified_motor_force,limit_min, limit_max, limit_bounciness,limit_bounceMinVelocity;
    public float HingeMoterVelocityA, HingeMoterVelocityB,HingeMoterVelocityC;
    public bool use_gravity=false;
    public float upper_arm_mass, elbow_mass, lower_arm_mass, platform_mass, drag,angular_drag;
    private HingeJoint hingeA,hingeB,hingeC;
    private Rigidbody UpperArmRigbodyA,UpperArmRigbodyB,UpperArmRigbodyC,ElbowRigbodyA,ElbowRigbodyB,ElbowRigbodyC;
    private JointSpring  hingeASpring,hingeBSpring,hingeCSpring;

    public void ConfigHingeJoint_usingSpring(HingeJoint hingeX, float targetPosition){
        hingeX.useMotor=false;
        hingeX.useSpring = use_spring;
        JointSpring hingeXSpring = hingeX.spring;
        hingeXSpring.spring = springConst;
        hingeXSpring.damper = damperConst;
        hingeXSpring.targetPosition =targetPosition;
        hingeX.spring = hingeXSpring;
    }
    public void ConfigHingeJoint_usingMotor(HingeJoint hingeX, float targetVelocity, float force, bool freeSpin=false){
        // Make the hinge motor rotate with 90 degrees per second and a strong force.
        hingeX.useSpring = false;
        hingeX.useMotor = use_motor;
        JointMotor motor = hingeX.motor;
        motor.force = force;
        motor.targetVelocity = targetVelocity;
        motor.freeSpin = freeSpin;
        hingeX.motor = motor;
    }
    

    // Start is called before the first frame update
    void Start()
    {   
        //initialise the parameters
        upper_arm_mass=1f;elbow_mass=1.25f;lower_arm_mass=0.1f;platform_mass=0.02f; drag=0.1f;angular_drag=0.03f;
        unified_motor_force=50f; HingeMoterVelocityA=5; HingeMoterVelocityB=-5;HingeMoterVelocityC=3;
        limit_min=-30f; limit_max=30f; limit_bounciness=10f;limit_bounceMinVelocity=1f;
        //UPPER ARMS 
        sceneParent = GameObject.Find("DeltaRobot1");
        UpperArms = GameObject.Find("UpperArms");

        UpperArms_A = UpperArms.transform.GetChild(0).gameObject;
        UpperArms_B = UpperArms.transform.GetChild(1).gameObject;
        UpperArms_C = UpperArms.transform.GetChild(2).gameObject;

        UpperArmRigbodyA=UpperArms_A.GetComponent<Rigidbody>();
        UpperArmRigbodyB=UpperArms_B.GetComponent<Rigidbody>();
        UpperArmRigbodyC=UpperArms_C.GetComponent<Rigidbody>();
        
        UpperArmRBList = new List<Rigidbody>() {UpperArmRigbodyA,UpperArmRigbodyB,UpperArmRigbodyC};

        hingeA = UpperArms_A.GetComponent<HingeJoint>();
        hingeB = UpperArms_B.GetComponent<HingeJoint>();
        hingeC = UpperArms_C.GetComponent<HingeJoint>();

        UpperArmHingeJointList=new List<HingeJoint>() {hingeA,hingeB,hingeC};

        foreach (HingeJoint hingeX in UpperArmHingeJointList){ConfigHingeJoint_usingSpring(hingeX,36.87f);}
    
        //ELBOWS
        ElbowJoints = GameObject.Find("ElbowJoints");
        ElbowJointA = ElbowJoints.transform.GetChild(0).gameObject;
        ElbowJointB = ElbowJoints.transform.GetChild(1).gameObject;
        ElbowJointC = ElbowJoints.transform.GetChild(2).gameObject;
        ElbowRigbodyA=ElbowJointA.GetComponent<Rigidbody>();
        ElbowRigbodyB=ElbowJointB.GetComponent<Rigidbody>();
        ElbowRigbodyC=ElbowJointC.GetComponent<Rigidbody>();

        ElbowJointList = new List<Rigidbody>() {ElbowRigbodyA,ElbowRigbodyB,ElbowRigbodyC};
        //LOWER ARMS
        GameObject LowerArms1= GameObject.Find("LowerArm1");
        GameObject LowerArms1A = LowerArms1.transform.GetChild(0).gameObject;
        GameObject LowerArms1B = LowerArms1.transform.GetChild(1).gameObject;
        GameObject LowerArms1C = LowerArms1.transform.GetChild(2).gameObject;
        Rigidbody LowerArmsRB1A=LowerArms1A.GetComponent<Rigidbody>();
        Rigidbody LowerArmsRB1B=LowerArms1B.GetComponent<Rigidbody>();
        Rigidbody LowerArmsRB1C=LowerArms1C.GetComponent<Rigidbody>();

        GameObject LowerArms2= GameObject.Find("LowerArm2");
        GameObject LowerArms2A = LowerArms2.transform.GetChild(0).gameObject;
        GameObject LowerArms2B = LowerArms2.transform.GetChild(1).gameObject;
        GameObject LowerArms2C = LowerArms2.transform.GetChild(2).gameObject;
        Rigidbody LowerArmsRB2A=LowerArms2A.GetComponent<Rigidbody>();
        Rigidbody LowerArmsRB2B=LowerArms2B.GetComponent<Rigidbody>();
        Rigidbody LowerArmsRB2C=LowerArms2C.GetComponent<Rigidbody>();

        LowerArmList = new List<Rigidbody>() {LowerArmsRB1A,LowerArmsRB1B,LowerArmsRB1C,LowerArmsRB2A,LowerArmsRB2B,LowerArmsRB2C};

        //PLATFORM
        GameObject Platform= GameObject.Find("Platforms");
        GameObject PlatformD = Platform.transform.GetChild(0).gameObject;
        GameObject PlatformE = Platform.transform.GetChild(1).gameObject;
        GameObject PlatformF = Platform.transform.GetChild(2).gameObject;
        Rigidbody PlatformRigBodD=PlatformD.GetComponent<Rigidbody>();
        Rigidbody PlatformRigBodE=PlatformE.GetComponent<Rigidbody>();
        Rigidbody PlatformRigBodF=PlatformF.GetComponent<Rigidbody>();

        PlatformRBList= new List<Rigidbody>() {PlatformRigBodD,PlatformRigBodE,PlatformRigBodF};
    }

    // Update is called once per frame
    void FixedUpdate()
    {   
        if (use_spring==true && use_motor==false){
            GameObject go = GameObject.Find ("CGA Model Manager");
            ArmDemo2 ArmFrame = go.GetComponent <ArmDemo2> ();
            float Angle_A = ArmFrame.Angle_A;
            JointSpring sprA = hingeA.spring;
            sprA.targetPosition = -Angle_A+36.87f;
            sprA.spring = springConst;
            sprA.damper = damperConst;
            hingeA.spring = sprA;
            hingeA.useSpring = use_spring;


            float Angle_B = ArmFrame.Angle_B;
            JointSpring sprB = hingeB.spring;
            sprB.targetPosition = -Angle_B+36.87f;
            sprB.spring = springConst;
            sprB.damper = damperConst;
            hingeB.spring = sprB;
            hingeB.useSpring = use_spring;

            float Angle_C = ArmFrame.Angle_C;
            JointSpring sprC = hingeC.spring;
            sprC.targetPosition = -Angle_C+36.87f;
            sprC.spring = springConst;
            sprC.damper = damperConst;
            hingeC.spring = sprC;
            hingeC.useSpring = use_spring;
        }
        else if (use_spring==false && use_motor==true){
            ConfigHingeJoint_usingMotor(hingeA, HingeMoterVelocityA, unified_motor_force);
            ConfigHingeJoint_usingMotor(hingeB, HingeMoterVelocityB, unified_motor_force);
            ConfigHingeJoint_usingMotor(hingeC, HingeMoterVelocityC, unified_motor_force);  // some random values for now, will be changed later
        }
        if (use_limit==true){
            foreach(HingeJoint HingeX in UpperArmHingeJointList){ ConfigHingeJoint_limit(HingeX,limit_min,limit_max, limit_bounciness,limit_bounceMinVelocity);}
        }
        updateRigidBodyParam(UpperArmRBList,upper_arm_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(ElbowJointList,elbow_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(LowerArmList,lower_arm_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(PlatformRBList,platform_mass,drag,angular_drag,use_gravity);
    }
    public void ConfigHingeJoint_limit(HingeJoint hinge ,float min, float max, float bounciness, float bounceMinVelocity ){
        JointLimits limits = hinge.limits;
        limits.min = min;
        limits.bounciness = bounciness;
        limits.bounceMinVelocity = bounceMinVelocity;
        limits.max = max;
        hinge.limits = limits;
        hinge.useLimits = true;
    }
    public void updateRigidBodyParam(IList<Rigidbody> RigBodList,float desired_mass, float desired_drag, float desired_angularDrag,bool use_gravity_bool){
        foreach (Rigidbody RigidBody in RigBodList){RigidBody.mass=desired_mass;RigidBody.drag=desired_drag; RigidBody.angularDrag =desired_angularDrag; RigidBody.useGravity=use_gravity_bool;}
    }
}