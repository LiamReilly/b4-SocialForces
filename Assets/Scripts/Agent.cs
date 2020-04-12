using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public const float DESIRED_VELOCITY = 1f;

    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
    private Vector3 WallForce = Vector3.zero;

    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();

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

        if (false)
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

        if (true)
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

    #region Incomplete Functions

    private Vector3 ComputeForce()
    {
        var force = Vector3.zero;

        force += CalculateGoalForce();
        force += CalculateAgentForce();
        force += WallForce*25f;

        //force += CalculateLeaderForce();
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
        if(count > 0){
            print("Removed " + count + " agents\n");
        }

        foreach(var agent in perceivedNeighbors){
            float distance = -Vector3.Distance(transform.position, agent.transform.position);
            Vector3 direction = transform.position - agent.transform.position;
            force += Mathf.Exp(distance)*direction.normalized;

            //sliding friction force
            float Agentradii = agent.gameObject.GetComponent<Agent>().radius + radius;
            float AgentOverLap = Agentradii - Vector3.Distance(transform.position, agent.transform.position);
            Vector3 tang = Vector3.Cross(Vector3.up, direction.normalized);
            if (AgentOverLap < 0)
            {
                AgentOverLap = 0;
            }
            force += AgentOverLap * Parameters.Kappa * Vector3.Dot(rb.velocity - agent.gameObject.GetComponent<Rigidbody>().velocity, tang) * tang;

        }

        return force;
    }
    
    private void CalculateWallForce(Collision collision)
    {
        Vector3 force = Vector3.zero;
        Vector3 direction = -(collision.contacts[0].point - transform.position).normalized;
        //Debug.DrawLine(collision.transform.position, transform.position, Color.magenta);
        WallForce += direction;
        return;
    }

    #region Single-Agent Behaviors

    #endregion

    #region Group Behaviors

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
        
        Vector3 direction = transform.position - leader.transform.position;


        //the strength of the force, we want it to be just a component of the force
        //so the agents slowly converge towards the leader while maintaining some level of autonomy
        float forceStrength = 0.1f;

        return direction.normalized * forceStrength;
    }

    #endregion

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
        if(WallManager.IsWall(collision.gameObject))
        {
            CalculateWallForce(collision);
            //print(WallForce);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        
    }

    #endregion
}
