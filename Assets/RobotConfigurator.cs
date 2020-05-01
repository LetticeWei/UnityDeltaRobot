﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotConfigurator : MonoBehaviour
{   
    private GameObject UpperArms,UpperArms_A,UpperArms_B,UpperArms_C,ElbowJoints,ElbowJointA,ElbowJointB,ElbowJointC;
    private GameObject Platforms, Platform_D,Platform_E,Platform_F,Bases, Base_A,Base_C,Base_B;

    public IList<Rigidbody> UpperArmRBList,ElbowJointList,LowerArmList,PlatformRBList;
    public IList<HingeJoint> UpperArmHingeJointList;
    public bool use_spring, use_motor, use_limit;

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
        hingeX.useMotor = true;
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
        
        upper_arm_mass=1f;elbow_mass=0.01f;lower_arm_mass=0.01f;platform_mass=0.01f; drag=0f;angular_drag=0f;
        unified_motor_force=300f; HingeMoterVelocityA=5; HingeMoterVelocityB=-5;HingeMoterVelocityC=3;
        limit_min=-30f; limit_max=30f; limit_bounciness=10f;limit_bounceMinVelocity=1f;
        use_spring=false;use_motor=true;use_limit= false;
        //UPPER ARMS 
        UpperArms=transform.GetChild(1).gameObject;

        UpperArms_A = UpperArms.transform.GetChild(0).gameObject;
        UpperArms_B = UpperArms.transform.GetChild(1).gameObject;
        UpperArms_C = UpperArms.transform.GetChild(2).gameObject;

        //Platforms
        Platforms=transform.GetChild(5).gameObject;

        Platform_D=Platforms.transform.GetChild(0).gameObject;
        Platform_F=Platforms.transform.GetChild(1).gameObject;
        Platform_E=Platforms.transform.GetChild(2).gameObject;

        Bases=transform.GetChild(0).gameObject;
        Base_A=Bases.transform.GetChild(0).gameObject;
        Base_C=Bases.transform.GetChild(1).gameObject;
        Base_B=Bases.transform.GetChild(2).gameObject;

        UpperArmRigbodyA=UpperArms_A.GetComponent<Rigidbody>();
        UpperArmRigbodyB=UpperArms_B.GetComponent<Rigidbody>();
        UpperArmRigbodyC=UpperArms_C.GetComponent<Rigidbody>();
        
        UpperArmRBList = new List<Rigidbody>() {UpperArmRigbodyA,UpperArmRigbodyB,UpperArmRigbodyC};

        hingeA = UpperArms_A.GetComponent<HingeJoint>();
        hingeB = UpperArms_B.GetComponent<HingeJoint>();
        hingeC = UpperArms_C.GetComponent<HingeJoint>();

        UpperArmHingeJointList=new List<HingeJoint>() {hingeA,hingeB,hingeC};

        if (use_spring==true && use_motor==false){
        foreach (HingeJoint hingeX in UpperArmHingeJointList){ConfigHingeJoint_usingSpring(hingeX,36.87f);}}
        else if (use_spring==false && use_motor==true){
        foreach (HingeJoint hingeX in UpperArmHingeJointList){
            ConfigHingeJoint_usingMotor(hingeX, 0f, unified_motor_force);}}
        
    
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

        //initialise last_error
        last_error=new Vector3(0,0,0);
        accum_error=new Vector3(0,0,0);
    }

    
    public Vector3 currentposition, end_point_error,last_error, accum_error;
    // Update is called once per frame
    void FixedUpdate()
{       GameObject CGA_Library_capsule = GameObject.Find ("CGA Library");
        DeltaRobotClass DeltaRobotFile = CGA_Library_capsule.GetComponent <DeltaRobotClass> ();
        DeltaRobot theRobot= DeltaRobotFile.Robot1;
        
        
        currentposition=  (1f/3f) * (Platform_D.transform.position+ Platform_F.transform.position+ Platform_E.transform.position
                                    - Base_A.transform.position- Base_B.transform.position- Base_C.transform.position);
        end_point_error= theRobot.end_point-currentposition;



        if (use_spring==true && use_motor==false){
            GameObject CGA_Model_Manager = GameObject.Find ("CGA Model Manager");
            ArmDemo2 ArmFrame = CGA_Model_Manager.GetComponent <ArmDemo2> ();
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

            float[] dtheta_dt_l = theRobot.dtheta_dt_l;
            // float[] dtheta_dt_l=new float[]{0,0,0};
            float k_p0=700f;
            float k_p=70f; 
            float k_d= 0f, k_i= 0f;
            Vector3 derivative_error=end_point_error-last_error;
            accum_error+=end_point_error;

            Vector3 input_velocity= k_p*end_point_error + k_d*derivative_error+ k_i * accum_error;
            // Vector3 input_velocity= k_p*end_point_error;

            float [,] inv_jacob=theRobot.differential_inverse_kinematics();

            // for(int i=0;i<3;i++){
            //     dtheta_dt_l[i]=inv_jacob[i,0]*input_velocity.x+inv_jacob[i,1]*input_velocity.y+inv_jacob[i,2]*input_velocity.z;
            // }

            HingeMoterVelocityA =k_p0* dtheta_dt_l[0];
            hingeA.useSpring = false;
            hingeA.useMotor = true;
            JointMotor temp_motor = hingeA.motor;
            temp_motor.force = unified_motor_force;
            temp_motor.targetVelocity = HingeMoterVelocityA;
            temp_motor.freeSpin = true;
            hingeA.motor = temp_motor;

            HingeMoterVelocityB = k_p0*dtheta_dt_l[1];
            // ConfigHingeJoint_usingMotor(hingeB, HingeMoterVelocityB, unified_motor_force);
            hingeB.useSpring = false;
            hingeB.useMotor = true;
            temp_motor = hingeB.motor;
            temp_motor.force = unified_motor_force;
            temp_motor.targetVelocity = HingeMoterVelocityB;
            temp_motor.freeSpin = true;
            hingeB.motor = temp_motor;

            HingeMoterVelocityC = k_p0*dtheta_dt_l[2];
            // ConfigHingeJoint_usingMotor(hingeC, HingeMoterVelocityC, unified_motor_force); 
            hingeC.useSpring = false;
            hingeC.useMotor = true;
            temp_motor = hingeC.motor;
            temp_motor.force = unified_motor_force;
            temp_motor.targetVelocity = HingeMoterVelocityC;
            temp_motor.freeSpin = true;
            hingeC.motor = temp_motor;

        }
        if (use_limit==true){
            foreach(HingeJoint HingeX in UpperArmHingeJointList){ ConfigHingeJoint_limit(HingeX,limit_min,limit_max, limit_bounciness,limit_bounceMinVelocity);}
        }
        updateRigidBodyParam(UpperArmRBList,upper_arm_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(ElbowJointList,elbow_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(LowerArmList,lower_arm_mass,drag,angular_drag,use_gravity);
        updateRigidBodyParam(PlatformRBList,platform_mass,drag,angular_drag,use_gravity);
        last_error=end_point_error;
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