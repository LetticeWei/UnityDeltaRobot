using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
public class SpringSinu3 : MonoBehaviour
{
    // Start is called before the first frame update
    private static HingeJoint hinge;
    private static JointSpring  hingeSpring;

    public float countdown=3f;
    public int round=0;

     void Start()
    {
        hinge = GetComponent<HingeJoint>();
        hingeSpring = hinge.spring;
        hingeSpring.spring = 5000;
        hingeSpring.damper = 4000;
        hingeSpring.targetPosition =36.87f;
        hinge.spring = hingeSpring;
        hinge.useSpring = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {   
        GameObject go = GameObject.Find ("CGA Model Manager");
        ArmDemo2 ArmFrame = go.GetComponent <ArmDemo2> ();
        float Angle_C = ArmFrame.Angle_C;
        
        JointSpring spr = hinge.spring;
        spr.targetPosition = -Angle_C+36.87F;
        hinge.spring = spr;
        
    }
}
