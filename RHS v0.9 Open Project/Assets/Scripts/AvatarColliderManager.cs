using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System;


[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(Animator))]
public class AvatarColliderManager : MonoBehaviour {

    private NavMeshAgent nMA;
    private  Transform anklePosition;
    public float distOfGround = -0.035f;
    

    public float initNMABaseOffset = 0f;
    public float initNMARadius = 0.22f;
    public float initNMAHeight = 1.8f;

    private float initDiffValue;


    private Transform agentRightFoot;
    private Transform agentLeftFoot;
    private Animator animator;


    // Use this for initialization
    void Start () {
        anklePosition = transform.Find(Constants.POS_ANKLE);
        animator = GetComponent<Animator>();
        nMA = GetComponent<NavMeshAgent>();
        nMA.baseOffset = initNMABaseOffset;
        nMA.radius = initNMARadius;
        nMA.height = initNMAHeight;
        initDiffValue = anklePosition.localPosition.y- initNMABaseOffset;
        agentRightFoot = animator.GetBoneTransform(HumanBodyBones.RightFoot);
        agentLeftFoot = animator.GetBoneTransform(HumanBodyBones.LeftFoot);        
    }
	
	// Update is called once per frame
	void Update () {
        float auxBaseOffset = initNMABaseOffset - anklePosition.localPosition.y;
        nMA.baseOffset = auxBaseOffset+initDiffValue;
        nMA.height = initNMAHeight - anklePosition.localPosition.y/2;
    }
}
