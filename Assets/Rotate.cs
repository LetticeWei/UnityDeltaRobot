using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotate : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        this.transform.RotateAround(this.transform.parent.position+new Vector3((float) 5f/1.7320508075f, 0f,5f),new Vector3(0,1f,0), -120f);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
