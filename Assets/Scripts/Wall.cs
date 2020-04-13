using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wall : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void Awake() {
        if(transform.eulerAngles.y == 90) {
            transform.localScale = new Vector3(transform.localScale.z, transform.localScale.y, transform.localScale.x);

            transform.eulerAngles = new Vector3(0,0,0);
        }
    }
}
