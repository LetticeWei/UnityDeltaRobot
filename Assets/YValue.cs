using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class YValue : MonoBehaviour
{
    // Start is called before the first frame update
    public float yvalue;
    public static YValue ins;
    void Awake(){
        ins=this;
    }
}
