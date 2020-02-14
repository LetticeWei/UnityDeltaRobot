using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
public class SpringSinu : MonoBehaviour
{
    // Start is called before the first frame update
    private static HingeJoint hinge;
    private static JointSpring  hingeSpring;

    void Start()
    {
        hinge = GetComponent<HingeJoint>();
        hingeSpring = hinge.spring;
        hingeSpring.spring = 5000;
        hingeSpring.damper = 4000;
        hingeSpring.targetPosition =36.86993f;
        hinge.spring = hingeSpring;
        hinge.useSpring = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {   
        GameObject go = GameObject.Find ("CGA Model Manager");
        ArmDemo2 ArmFrame = go.GetComponent <ArmDemo2> ();
        float Angle_A = ArmFrame.Angle_A;
        
        JointSpring spr = hinge.spring;
        spr.targetPosition = -Angle_A+36.86993f;
        hinge.spring = spr;
        
    }
}
