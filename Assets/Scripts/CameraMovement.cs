using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    private float speed = 6;
    private float vSpeed = 3;
    public int hMouseSpeed = 3;
    public int vMouseSpeed = 3;
    private bool invertY = true;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 normalForward = fixYAxis(gameObject.transform.forward);
        Vector3 normalRight = fixYAxis(gameObject.transform.right);
        
        if(Input.GetKeyDown("i")){
            invertY = !invertY;
            print("Y inverted: " + (!invertY).ToString());
        }

        if(Input.GetKeyDown("=")){
            IncreaseSpeed();
        }

        if(Input.GetKeyDown("-")){
            DecreaseSpeed();
        }

        if (Input.GetKey("w"))
        {
            gameObject.transform.Translate(normalForward * Time.deltaTime*speed, Space.World);
        }
        if (Input.GetKey("s"))
        {
            gameObject.transform.Translate(-normalForward * Time.deltaTime*speed, Space.World);
        }
        if (Input.GetKey("a"))
        {
            gameObject.transform.Translate(-normalRight * Time.deltaTime*speed, Space.World);
        }
        if (Input.GetKey("d"))
        {
            gameObject.transform.Translate(normalRight * Time.deltaTime*speed, Space.World);
        }
        if (Input.GetKey(KeyCode.LeftShift)|| Input.GetKey(KeyCode.RightShift))
        {
            gameObject.transform.Translate(Vector3.up * Time.deltaTime*vSpeed, Space.World);
        }
        if (Input.GetKey(KeyCode.Space))
        {
            gameObject.transform.Translate(Vector3.down * Time.deltaTime*vSpeed, Space.World);
        }
        if(Input.GetMouseButton(1))
        {
            if(Cursor.lockState == CursorLockMode.None){
                Cursor.lockState = CursorLockMode.Locked;
            }

            float h = hMouseSpeed * Input.GetAxis("Mouse X");
            float v = vMouseSpeed * Input.GetAxis("Mouse Y");

            if(invertY){
                v = -v;
            }

            //Ensures the Y axis rotation is absolute while the X is relative
            gameObject.transform.Rotate(0, h, 0, Space.World);
            gameObject.transform.Rotate(v, 0, 0);
        }else{
            if(Cursor.lockState == CursorLockMode.Locked){
                Cursor.lockState = CursorLockMode.None;
            }
        }
    }

    //Removes the Y component of a vector and normalizes it to prevent drift up and down when moving
    Vector3 fixYAxis(Vector3 vector)
    {   
        Vector3 returnee = new Vector3(vector.x, 0, vector.z);
        returnee.Normalize();
        return returnee;

    }
    public void IncreaseSpeed()
    {
        vSpeed++;
        speed +=2;
    }
    public void DecreaseSpeed()
    {
        if(speed>2){
            speed-=2;
            vSpeed--;
        }
    }
}
