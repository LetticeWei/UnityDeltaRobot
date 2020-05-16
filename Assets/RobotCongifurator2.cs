using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotCongifurator2 : MonoBehaviour
{
    public GameObject UpperArms;
    public GameObject[] UpperArms_l;
    public Rigidbody[] UpperArmRigbody_l;
    public HingeJoint[] UpperArmHingeJointList;
    public float unified_motor_force;
    public DeltaRobot theRobot;
    

    // Start is called before the first frame update
    void Start()
    {   //initialise the parameters
        unified_motor_force=300f;
        
        //Find the framework
        GameObject CGA_Library_capsule = GameObject.Find ("CGA Library");
        DeltaRobotClass DeltaRobotFile = CGA_Library_capsule.GetComponent <DeltaRobotClass> ();
        theRobot= DeltaRobotFile.Robot1;

        //Find the upper arm models.
        UpperArms=transform.GetChild(3).gameObject;
        UpperArms_l = new GameObject[3];
        UpperArmRigbody_l=new Rigidbody[3];
        UpperArmHingeJointList= new HingeJoint[3];
        for (int i = 0; i < 3; i++){
            UpperArms_l[i]=UpperArms.transform.GetChild(i).gameObject;
            UpperArmRigbody_l[i]=UpperArms_l[i].GetComponent<Rigidbody>();
            UpperArmHingeJointList[i] = UpperArmRigbody_l[i].GetComponents<HingeJoint>()[0];
        } 
    }

    // Update is called once per frame
    void FixedUpdate()
{      float k_p0=1000f;
        float[] dtheta_dt_l = theRobot.dtheta_dt_l;
        for (int i = 0; i < 3; i++){
            float HingeMoterVelocity=k_p0* dtheta_dt_l[i];
            UpperArmHingeJointList[i].useSpring = false;
            UpperArmHingeJointList[i].useMotor = true;
            JointMotor temp_motor = UpperArmHingeJointList[i].motor;
            temp_motor.force = unified_motor_force;
            temp_motor.targetVelocity = HingeMoterVelocity;
            temp_motor.freeSpin = true;
            UpperArmHingeJointList[i].motor = temp_motor;
        }
    }
}
