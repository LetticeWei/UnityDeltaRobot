using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
public class SpringSinu2 : MonoBehaviour
{
    // Start is called before the first frame update
    private static HingeJoint hinge;
    private static JointSpring  hingeSpring;

    public float countdown=5f;
    public int round=0;
    void Start()
    {
        hinge = GetComponent<HingeJoint>();
        hingeSpring = hinge.spring;
        hingeSpring.spring = 50;
        hingeSpring.damper = 30;
        hingeSpring.targetPosition =25f;
        hinge.spring = hingeSpring;
        hinge.useSpring = true;
    }

    // Update is called once per frame
    void FixedUpdate()
    {   
        countdown-=Time.deltaTime;
        if(countdown<=0.0f){
            hingeSpring.targetPosition = -1f*hingeSpring.targetPosition;
            JointSpring spr = hinge.spring;
            spr.targetPosition =hinge.spring.targetPosition*-1f;
            hinge.spring = spr;
            countdown=10.0f;
        }
    }
}
