using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public const float DESIRED_VELOCITY = 5f;

    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    private Vector3 WallForce = Vector3.zero;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();

    private bool viewpath = false, viewneighbors = true;

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                perceivedNeighbors.Clear();
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization

        if (viewpath)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }

        if (viewneighbors)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
        //path = new List<Vector3>() { destination };
        //nma.SetDestination(destination);
        nma.enabled = false;

    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Part 1

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;

        // force += CalculateGoalForce();
        force += CalculateAgentForce();
        force += WallForce*25f;

        force += CalculateWallFollowForce();
        // force += CalculateLeaderForce();
        //force += CalculateSpiralForce();

        WallForce = Vector3.zero;
        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
        
    }
    
    private Vector3 CalculateGoalForce()
    {
        Vector3 velocity = GetVelocity();
        Vector3 direction = (path[0] - transform.position).normalized;

        Vector3 force = mass*(DESIRED_VELOCITY*direction-velocity);

        return force;
    }

    private Vector3 CalculateAgentForce()
    {
        Vector3 force = Vector3.zero;

        //Remove inactive agents
        int count = perceivedNeighbors.RemoveWhere(g => !g.activeSelf);
        // if(count > 0){
        //     print("Removed " + count + " agents\n");
        // }

        foreach(var agent in perceivedNeighbors){
            float distance = -Vector3.Distance(transform.position, agent.transform.position);
            Vector3 direction = (transform.position - agent.transform.position).normalized;

            float agentRadii = agent.gameObject.GetComponent<Agent>().radius + radius;
            float agentOverLap = Math.Max(agentRadii - Vector3.Distance(transform.position, agent.transform.position), 0);

            //Psychological force
            force += Parameters.A * Mathf.Exp(distance / Parameters.B) * direction;

            //Non-Penetration force
            force += agentOverLap * Parameters.k * distance *direction;

            //sliding friction force
            Vector3 tang = Vector3.Cross(Vector3.up, direction);
            force += agentOverLap * Parameters.Kappa * Vector3.Dot(rb.velocity - agent.gameObject.GetComponent<Rigidbody>().velocity, tang) * tang;

        }

        return force;
    }
    
    private void CalculateWallForce(Collision collision)
    {
        Vector3 force = Vector3.zero;
        Vector3 direction = -(collision.contacts[0].point - transform.position).normalized;
        //Debug.DrawLine(collision.transform.position, transform.position, Color.magenta);
        
        float magnitude = 1.0f;

        WallForce += direction*magnitude*mass;
        return;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            perceivedNeighbors.Add(other.gameObject);
        }
    }
    
    public void OnTriggerExit(Collider other)
    {
        if (perceivedNeighbors.Contains(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.tag == "wall") {
            lastCollidedWall = collision.gameObject;
        }

        if(WallManager.IsWall(collision.gameObject))
        {
            //lastCollidedWall = collision.gameObject;

            CalculateWallForce(collision);
            //print(WallForce);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        
    }

    #endregion

    #region Part 2

    #region Single-Agent Behaviors

        //saves the location of the last wall the agent has collided with
        GameObject lastCollidedWall = null;
        float maxWallDist;

        private Vector3 CalculateWallFollowForce() {
            if(lastCollidedWall == null) {
                return Vector3.zero;
            }

            Vector3 separation = transform.position - lastCollidedWall.transform.position, direction = new Vector3(0,0,0);

            /*determine result direction based on separation direction, clockwise result
            imagine a square split into 4 based on diagonals.
            
              \  up  /
               \ -> /
              ^ \  /
              |  \/ right
            left /\   |
                /  \  V
               / <- \
              / down \

            up:     (1,0,0)     z > |x|
            right:  (0,0,-1)    x > |z|
            down:   (-1,0,0)    z < -|x|
            left:   (0,0,1)     x < -|z|

            added parameters for inward and forward force (vector is normalized later anyway)
             - [discontinued, now use constant force towards center] inward force helps agents stick on walls
             - forward force is what makes them crawl

            */

            int inwardF = 0, forwardF = 2;

            //adjust for rectangular (instead of square/cube) walls
            float x = separation.x * lastCollidedWall.transform.localScale.z;
            float z = separation.z * lastCollidedWall.transform.localScale.x;



            if(z >= Math.Abs(x)) {
                direction = new Vector3(forwardF,0,-1*inwardF);
            }
            else if(x > Math.Abs(z)) {
                direction = new Vector3(-1*inwardF,0,-1*forwardF);
            }
            else if(z < -1*Math.Abs(x)) {
                direction = new Vector3(-1*forwardF,0,inwardF);
            }
            else if(x < -1*Math.Abs(z)) {
                direction = new Vector3(inwardF,0,forwardF);
            }

            direction = direction.normalized;

            
            //don't apply force if already moving in the right direction
            float directionalVelocity = Vector3.Dot(rb.velocity, direction);
            if(directionalVelocity > 0.6f) {
                direction = Vector3.zero;
            }

            //add inward force
            Vector3 inF = separation.normalized * -0.09f;

            //full inward force should be applied at a little past the corner
            //corner is distance sqrt((x scale/2)^2 + (z scale/2)^2)+radius, more or less
            //we don't need to be exact here, so using squares should suffice?
            //inF *= separation.magnitude

            
            Vector3 finalForce = direction+inF;
            
            Debug.DrawLine(transform.position, lastCollidedWall.transform.position, Color.red);
            Debug.DrawLine(transform.position, transform.position+finalForce*5, Color.green);

            return mass*(finalForce + inF);
        }


        private Vector3 CalculateSpiralForce(){
            Vector3 toCenter = Vector3.zero - new Vector3(transform.position.x, 0, transform.position.z);
            Vector3 tang = Vector3.Cross(toCenter, Vector3.up);

            Vector3 newForce = tang;
            newForce += toCenter;

            Debug.DrawLine(transform.position, transform.position+tang, Color.magenta);
            Debug.DrawLine(transform.position, transform.position+toCenter, Color.green);

            return newForce * mass;
        }

    #endregion

    #region Group Behaviors

        #region Crowd Following

        #endregion

        #region Leader Following
        public Agent leader;
        bool hasLeader;

        public void setLeader(Agent a) {
            hasLeader = true;
            leader = a;
        }
        public void removeLeader(Agent a) {
            hasLeader = false;
        }
        private Vector3 CalculateLeaderForce() {
            if(!hasLeader) {
                return Vector3.zero;
            }
            
            Vector3 posDif = transform.position - leader.transform.position;


            /*
            want a vaguely logarithmic-shaped curve:
                when close to the leader, we want to move out of the way
                when far from the leader, we want to move closer
            
            approximation will use the properties of xy = c: asymptotic stuff
                we'd like to use the 4-th quadrant portion of xy = -1
                x: distance
                y: velocity

            we want to move the graph up so we get a y-intercept and our agents don't fly away from the leader at the speed of sound
                say we want to have a force of 1 away when just about touching
                similarly, say we want to have an asymptotic force of about 1

            more or less what we want: 
                y = -2/(x+1) + 1

            */

            float dist = Math.Abs(posDif.magnitude) - radius - leader.radius;
            float forceMult = -2 / dist + 1;

            return posDif.normalized * mass * forceMult;
        }
        #endregion

    #endregion

    

    #endregion
}
