using System.Collections;
using System.Collections.Generic;
using UnityEngine;



[RequireComponent(typeof(Animator))]
public class AgentMovement : MonoBehaviour {

    //Define o animator
    protected Animator animator;
    protected UnityEngine.AI.NavMeshAgent nav;
    private MovementOperations mO;
    // Use this for initialization
    private Command command;
    public float distanceToReach = 0.2f;

    private Vector3 previousPosition;
    private int countUpdate;
    private Transform agentSpine;
    private Hand[] hands;

    void Awake()
    {
        animator = GetComponent<Animator>();
        nav = GetComponent<UnityEngine.AI.NavMeshAgent>();
        mO = GetComponent<MovementOperations>();
        command = null;
        agentSpine = animator.GetBoneTransform(HumanBodyBones.Spine);
    }
    void Start () {       

        nav.updateRotation = false;
        nav.updatePosition = true;
        previousPosition = new Vector3();
        countUpdate = 0;
        hands = GetComponent<AgentInteraction>().getHands();
        Debug.Log("RHS>>> " + this.name + " ready to receive Movimentation Commands.");

    }

    // Update is called once per frame
    void Update()
    {
        if (command != null)
        {
            managerNavMeshStopDistance(command);
            switch (command.getAction())
            {
                case Action.Move:

                    switch (command.getActionStateID())
                    {
                        case (int)Move.Start:
                            Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Started!");
                            command.next();
                        break;
                        case (int)Move.Position:
                            nav.enabled = true;
                            nav.isStopped = false;
                            Vector3 destination = command.getNearestDesiredLocation(transform.position);
                            nav.SetDestination(destination);

                            Vector2 pos1 = new Vector2(transform.position.x, transform.position.z);
                            Vector2 pos2 = new Vector2(destination.x,destination.z);
                           
                            float distance = (pos1 - pos2).magnitude;
                            if (distance > nav.stoppingDistance )
                            {
                                Vector3 aux = nav.desiredVelocity;
                                
                                mO.Move(aux, false, false);
                                if (!isV3Zero(previousPosition))
                                {                                    
                                    Vector3 curMove = transform.position - previousPosition;
                                    float velocity = curMove.magnitude / Time.deltaTime;
                                    if (velocity == 0 && countUpdate >20)
                                    {
                                        pos1 = new Vector2(agentSpine.position.x, agentSpine.position.z);

                                        float spineDistance = (pos1 - pos2).magnitude;
                                        if ( spineDistance < nav.stoppingDistance+distanceToReach)
                                        {
                                            command.next();
                                        }
                                        else
                                        {
                                            Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Failed! Position is not reachable.");
                                            command.fail();
                                        }
                                        previousPosition = Vector3.zero;
                                        countUpdate = 0;
                                    }
                                    else
                                    {
                                        countUpdate++;
                                        previousPosition = transform.position;
                                    }
                                }else
                                {
                                    previousPosition = transform.position;
                                }
                                
                            }
                            else
                            {
                                countUpdate = 0;
                                mO.Move(Vector3.zero, false, false);
                                previousPosition = Vector3.zero;
                                command.next();
                            }

                            break;
                        case (int)Move.Turn:
                            if (turn(transform, command.getNearestDesiredPosition(transform.position)))
                            {
                                Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Success!");
                                command.success();
                            }
                            break;
                        case (int)Move.End:
                            mO.Move(Vector3.zero, false, false);
                            //nav.enabled = false;
                            nav.velocity = Vector3.zero;
                            nav.isStopped = true;
                            break;
                    }
                    
                    break;
                case Action.Turn:                    
                    switch (command.getActionStateID())
                    {
                        case (int)Turn.Start:
                            Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Started!");
                            foreach (Hand hand in hands)
                            {
                                if (command.getReference() != null)
                                {
                                    if (GameObject.ReferenceEquals(hand.objInHand, command.getReference()))
                                    {
                                        Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Failed! Object is in hand of the robot.");
                                        command.fail();
                                        break;
                                    }

                                }
                            }
                            if (!command.isFail()) {
                                command.next();
                            }
                            break;
                        case (int)Turn.Position:
                            if (turn(transform,command.getNearestDesiredPosition(transform.position)))
                            {
                                Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Success!");
                                command.success();
                            }
                            break;
                        case (int)Turn.End:
                            mO.Move(Vector3.zero, false, false);
                            break;
                    }
                    break;
               case Action.Rotate:
                    switch (command.getActionStateID())
                    {
                        case (int)Rotate.Start:
                            Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Started!");
                            command.next();
                            break;
                        case (int)Rotate.Position:

                            float angleToRotate = command.getAngleRotation();
                            var x = Mathf.Cos(angleToRotate *  Mathf.Deg2Rad);
                            var y = Mathf.Sin(angleToRotate * Mathf.Deg2Rad);

                            Vector3 directionToTarget = new Vector3(y, 0f, x);
 
                            Vector3 vectorTransform = transform.forward; 
                            float angle = Vector3.Angle(vectorTransform, directionToTarget);
                            float arcTan = Mathf.Atan2(directionToTarget.x, directionToTarget.z);
                            if (angle > 1)
                            {
                                Vector3 aux = new Vector3(Mathf.Sin(arcTan), 0, Mathf.Cos(arcTan));
                                mO.Move(aux, false, false);
                            }
                            else
                            {
                                mO.Move(Vector3.zero, false, false);
                                Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Success!");
                                command.success();
                            }
                            break;
                        case (int)Rotate.End:
                            mO.Move(Vector3.zero, false, false);
                            break;
                    }
                    break;
                default:
                    Debug.Log("Command>>> " + this.name + " command " + command.getId() + " Failed! ERROR!");
                    break;
            }
        }
    }

  
    private bool turn(Transform from, Vector3 to )
    {
        Vector3 directionToTarget = to - from.position;
        directionToTarget.y = 0;
        Vector3 vectorTransform = from.forward; //- transform.right;
        vectorTransform.y = 0;
        float angle = Vector3.Angle(vectorTransform, directionToTarget);
        float arcTan = Mathf.Atan2(directionToTarget.x, directionToTarget.z);
        if (angle > 1)
        {
            Vector3 aux = new Vector3(Mathf.Sin(arcTan), 0, Mathf.Cos(arcTan));
            mO.Move(aux, false, false);
            return false;
        }
        else
        {
            mO.Move(Vector3.zero, false, false);
            return true;
        }
    }

    public bool sendCommand(Command command)
    {
        this.command = command;
        Debug.Log("Command>>> " + this.name + " received command " + command.getStringCommand());
        return true;
    }

    public bool isV3Zero(Vector3 a)
    {
        return Vector3.SqrMagnitude(Vector3.zero - a) < 0.0001;
    }

    private void managerNavMeshStopDistance(Command command)
    {
        if (nav != null )
        {
            if (command.getReference() != null)
            {
                switch (Constants.getTypeOfTag(command.getReference().tag))
                {
                    case Constants.TAG_DOOR:
                        nav.stoppingDistance = Constants.DIST_DOOR;
                        break;
                    case Constants.TAG_OBJECT:
                        nav.stoppingDistance = Constants.DIST_OBJECT;
                        break;
                    case Constants.TAG_DRAWER:
                        nav.stoppingDistance = Constants.DIST_DRAWER;
                        break;
                    case Constants.TAG_HUMAN:
                        nav.stoppingDistance = Constants.DIST_HUMAN;
                        break;
                    case Constants.TAG_SWITCH:
                        nav.stoppingDistance = Constants.DIST_SWITCH;
                        break;
                    case Constants.TAG_TAP:
                        nav.stoppingDistance = Constants.DIST_TAP;
                        break;
                    case Constants.TAG_WATER:
                        nav.stoppingDistance = Constants.DIST_TAP;
                        break;
                    default:
                        nav.stoppingDistance = Constants.DIST_DEFAULT;
                        break;
                }
            }else
            {
                nav.stoppingDistance = Constants.DIST_DEFAULT;
            }
        }
    }
}
