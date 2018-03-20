using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OntSenseCSharpAPI;

//public enum ObjectStatus {None, On, Off, Opened,Closed}
public class Status : MonoBehaviour {

    public  PhysicalState status = PhysicalState.noneState;
    public float speed = 2f;
    // Use this for initialization

    public bool close()
    {
        status = PhysicalState.closeState;
        return true;
    }

    public bool open()
    {
        status = PhysicalState.openState;
        return true;
    }

    public bool on()
    {
        status = PhysicalState.onState;
        return true;
    }

    public bool off()
    {
        status = PhysicalState.offState;
        return true;
    }

    public bool trigger()
    {
        if (status == PhysicalState.closeState)
        {
            status = PhysicalState.openState;
        }
        else if(status == PhysicalState.openState)
        {
            status = PhysicalState.closeState;
        }else
        if (status == PhysicalState.onState)
        {
            status = PhysicalState.offState;
        }else 
        if (status == PhysicalState.offState)
        {
            status = PhysicalState.onState;
        }

        return true;
    }

    public string getStringStatus()
    {
        return status.ToString();
    }

    public PhysicalState getStatus()
    {
        return status;
    }

    public virtual void turnOnOpen()
    {

    }
    public virtual void turnOffClose()
    {

    }
}
