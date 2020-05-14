using System.Collections;
using System.Collections.Generic;
using UnityEngine;
[RequireComponent(typeof(MeshFilter))]
public class ProcedualMesh : MonoBehaviour
{   
    Mesh mesh;
    Vector3[] vertices;
    int[] triangles;
    //grid settings
    public float cellSize;
    public Vector3 gridoffsets;
    public int gridSize;

    void Awake(){
        mesh=GetComponent<MeshFilter>().mesh;
    }
    // Start is called before the first frame update
    void Start(){
        MakeDiscreteProceduralGrid();
        CreateMesh();
        // MakeBasePlane();
        // CreateMesh();
    }
    void Update(){
        // MakePlatform();
        // CreateMesh();
        // MakeContinuousProceduralGrid();
        // CreateMesh();
    }
    // void MakeBasePlane(){
    //     vertices=new Vector3[]{new Vector3(-1.193f,0,-2.0669f),new Vector3(-1.193f,0,2.0669f),new Vector3(2.3867f,0,0)};
    //     triangles=new int[]{0,1,2};
    // }
    // void MakePlatform(){
    //     GameObject CGA_Library_capsule = GameObject.Find ("CGA Library");
    //     DeltaRobotClass DeltaRobotFile = CGA_Library_capsule.GetComponent <DeltaRobotClass> ();
    //     DeltaRobot theRobot= DeltaRobotFile.Robot1;
    //     vertices=new Vector3[]{theRobot.Platform_l[0].Point3D,theRobot.Platform_l[1].Point3D,theRobot.Platform_l[2].Point3D};
    //     triangles=new int[]{0,1,2};
    // }
    void MakeDiscreteProceduralGrid(){
        //set array sizes
        vertices=new Vector3[gridSize*gridSize*4];
        triangles=new int[gridSize*gridSize*6];
        //set tracker integers
        int v=0;
        int t=0;
        //set vertex offset
        float vertexOffset=cellSize*0.5f;

        for(int x=0;x<gridSize;x++){
            for(int y=0;y<gridSize;y++){
                //populate the vertices and triangles arrays
                Vector3 cellOffset = new Vector3(x*cellSize,0,y*cellSize);
                vertices[v]=new Vector3(-vertexOffset,x+y,-vertexOffset)+cellOffset+gridoffsets;
                vertices[v+1]=new Vector3(-vertexOffset,x+y,vertexOffset)+cellOffset+gridoffsets;
                vertices[v+2]=new Vector3(vertexOffset,x+y,-vertexOffset)+cellOffset+gridoffsets;
                vertices[v+3]=new Vector3(vertexOffset,x+y,vertexOffset)+cellOffset+gridoffsets;
                triangles[t]=v;
                triangles[t+1]=triangles[t+4]=v+1;
                triangles[t+2]=triangles[t+3]=v+2;
                triangles[t+5]=v+3;
                v+=4;
                t+=6;
            }
        }
    }

     void MakeContinuousProceduralGrid(){
        //set array sizes
        vertices=new Vector3[(gridSize+1)*(gridSize+1)];
        triangles=new int[gridSize*gridSize*6];
        //set tracker integers
        int v=0;
        int t=0;
        //set vertex offset
        float vertexOffset=cellSize*0.5f;

        for(int x=0;x<=gridSize;x++){
            for(int y=0;y<=gridSize;y++){
                vertices[v]= new Vector3 ((x*cellSize-vertexOffset), (x+y)*0.2f , (y*cellSize-vertexOffset));
                v++;
            }
        }
        //reset vertext tracker
        v=0; 
        //setting each cell's triangles
        for(int x=0;x<gridSize;x++){
            for(int y=0;y<gridSize;y++){
                triangles[t]=v;
                triangles[t+1]=triangles[t+4]=v+1;
                triangles[t+2]=triangles[t+3]=v+(gridSize+1);
                triangles[t+5]=v+(gridSize+1)+1;
                v++;
                t+=6;
            }
            v++;
        }
    }


    void CreateMesh(){
        mesh.Clear();
        mesh.vertices=vertices;
        mesh.triangles=triangles;
        mesh.RecalculateNormals();
    }
    // Update is called once per frame

}
