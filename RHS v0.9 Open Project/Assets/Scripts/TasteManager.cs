using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class TasteManager : MonoBehaviour {

    public Transform tasteSensor;
    private CaptureTaste captureTaste;
    private HashSet<GameObject> gameObjects;
    private HashSet<GameObject> updatedElementsList;
    public int refreshRate = 300;
    private int count;
    // Use this for initialization
    void Start()
    {
        if (tasteSensor == null)
        {
            Debug.Log("Error! Taste Sensor is missing!");
        }
        else
        {
            captureTaste = tasteSensor.GetComponent<CaptureTaste>();
        }
        count = 0;
        gameObjects = new HashSet<GameObject>();
        updatedElementsList = new HashSet<GameObject>();
        Debug.Log("RHS>>> " + this.name + " taste was configured with success.");
    }

    void Update()
    {
        if (captureTaste != null)
        {
            if (count < refreshRate)
            {
                GameObject auxGameObject = captureTaste.getActTaste();
                if (auxGameObject != null && !auxGameObject.tag.Equals(Constants.TAG_BODYPART))
                {
                    gameObjects.Add(auxGameObject);
                }
                count++;
            }
            else
            {
                updatedElementsList = new HashSet<GameObject>(gameObjects);
                gameObjects = new HashSet<GameObject>();
                count = 0;
            }
        }
    }

    public string getAllSmellStrings()
    {
        string aux = "";
        foreach (GameObject smell in gameObjects)
        {
            aux += " " + smell.name;
        }
        return aux;
    }

    public List<GameObject> getListOfElements()
    {
        return updatedElementsList.ToList();
    }
}
