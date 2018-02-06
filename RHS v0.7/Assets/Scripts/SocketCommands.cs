
   using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System;
using System.Net;
using System.IO;
using ProtoBuf;
using System.Linq;
using UnityEngine.UI;

public enum CommandType
{
    ActivateLeft, ActivateRight, DeactivateLeft, DeactivateRight,
    HeadReset, LeaveLeft, LeaveRight, LookAt, LookFor, Move,
    Rotate, SmellLeft, SmellRight, Speech, TakeLeft, TakeRight,
    TasteLeft, TasteRight, Turn, CancelCommands
}


public class SocketCommands : MonoBehaviour
{    
    private ServerClient client;
    private List<ServerClient> disconnectList;
    private List<Command> runningCommands;
    //public GameObject messageContainer;
    //public GameObject messagePrefab;

    public int port = 6321;
    private TcpListener server;
    private bool serverStarted;
    private SimulatorCommandsManager scm;
    private UniqueIdDistributor uid;
    


    static readonly IDictionary<int, Type> typeLookup = new Dictionary<int, Type>
    {
        {1, typeof(CommandWithId)}, {2, typeof(CommandWithAngle)}, {3, typeof(CommandWithoutPar)},
        { 4, typeof(CommandWithPosition)}, {5, typeof(CommandWithString)}, {6, typeof(Response)}
    };

    private void Start()
    {
        scm = transform.GetComponent<SimulatorCommandsManager>();
        uid = transform.GetComponent<UniqueIdDistributor>();
        disconnectList = new List<ServerClient>();
        runningCommands = new List<Command>();
        try
        {
            server = new TcpListener(IPAddress.Any, port);
            server.Start();

            startListening();
            serverStarted = true;
            Debug.Log("System>>> Server has been started on port " + port.ToString());
        }
        catch (Exception e)
        {
            Debug.Log("System>>> Socket Error:" + e.Message);
        }
    }



    private void Update()
    {
        if (!serverStarted)
            return;
        if (client != null)
        {
            //Is the client still connected?
            if (!isConnected(client.tcp))
            {
                client.tcp.Close();
                disconnectList.Add(client);
            }
            else
            {
                NetworkStream s = client.tcp.GetStream();
                if (s.DataAvailable)
                {
                    //StreamReader reader = new StreamReader(s, true);
                    // string data = reader.ReadLine();
                    object data;
                    if (Serializer.NonGeneric.TryDeserializeWithLengthPrefix(s, PrefixStyle.Base128, field => typeLookup[field], out data))
                    {
                        onIncomingData(client, data);
                    }
                    /*if (data != null)
                    {
                        onIncomingData(c, data);
                    }*/
                }
                List<Command> auxList = new List<Command>(runningCommands);
                foreach(Command c in auxList)
                {
                    Response response;
                    switch (c.getCommandStatus())
                    {
                        case CommandStatus.Success:
                             response= new Response { idCommand = c.getId(), executed = true };
                            sendResponse(response);
                            runningCommands.Remove(c);
                            break;
                        case CommandStatus.Fail:
                             response = new Response { idCommand = c.getId(), executed = false };
                            runningCommands.Remove(c);
                            sendResponse(response);
                            break;
                        default:
                            break;
                    }
                }
                
            }
            //Check for message from the client


            for (int i = 0; i < disconnectList.Count - 1; i++)
            {

                disconnectList.RemoveAt(i);
            }
        }
    }

    private bool isConnected(TcpClient c)
    {
        try
        {
            if (c != null && c.Client != null && c.Client.Connected)
            {
                if (c.Client.Poll(0, SelectMode.SelectRead))
                {
                    return !(c.Client.Receive(new byte[1], SocketFlags.Peek) == 0);
                }
                return true;
            }
            return false;
        }
        catch
        {
            return false;
        }
    }

    private void startListening()
    {
        server.BeginAcceptTcpClient(acceptTcpClient, server);
    }

    private void acceptTcpClient(IAsyncResult ar)
    {
        TcpListener listener = (TcpListener)ar.AsyncState;

        client = new ServerClient(listener.EndAcceptTcpClient(ar));
        startListening();

        // Send a message to everyone, say somene has connected
        Debug.Log("System>>> Client connected.");
        //broadcast(clients[clients.Count-1].clientName + " has connected",clients);

        //broadcast("%NAME",new List<ServerClient>() { clients[clients.Count - 1] });
    }

    private void onIncomingData(ServerClient c, object data)
    {
        /*if (data.Contains("&NAME"))
        {
            c.clientName = data.Split('|')[1];
            broadcast(c.clientName + " has connected!",clients);
            return;
        }*/
        Debug.Log("RHS>>> socket incoming data.");
        processCommand(data);
        //sendResponse();
    }

    private void processCommand(object data)
    {
        string idCommand = "";
        //Tanto faz o valor inicial (gambiarra)
        Action action = Action.Rotate;
        Hands hand = Hands.Right;
        bool itsValid = true;
        if (data.GetType() == typeof(CommandWithId))
        {
            CommandWithId cWId = (CommandWithId)data;
            idCommand = cWId.commandId;
            GameObject gO = getGameObjectById(cWId.objectId);
            if (gO != null)
            {
                switch (cWId.commandType)
                {
                    case CommandType.ActivateLeft:
                        hand = Hands.Left;
                        action = Action.Activate;
                        break;
                    case CommandType.ActivateRight:
                        action = Action.Activate;
                        break;
                    case CommandType.DeactivateLeft:
                        hand = Hands.Left;
                        action = Action.Deactivate;
                        break;
                    case CommandType.DeactivateRight:
                        action = Action.Deactivate;
                        break;
                    case CommandType.LeaveLeft:
                        hand = Hands.Left;
                        action = Action.Release;
                        break;
                    case CommandType.LeaveRight:
                        action = Action.Release;
                        break;
                    case CommandType.LookAt:
                        action = Action.HeadFocus;
                        break;
                    case CommandType.Move:
                        action = Action.Move;
                        break;
                    case CommandType.TakeLeft:
                        hand = Hands.Left;
                        action = Action.Take;
                        break;
                    case CommandType.TakeRight:
                        action = Action.Take;
                        break;
                    case CommandType.Turn:
                        action = Action.Turn;
                        break;
                    default:
                        itsValid = false;
                        break;
                }
                if (itsValid)
                {
                    runningCommands.Add(scm.sendCommand(idCommand, hand, action, gO.transform));
                }
            }
            else
            {
                itsValid = false;
            }
                       

        }
        else if (data.GetType() == typeof(CommandWithAngle))
        {
            CommandWithAngle cWAngle = (CommandWithAngle)data;
            idCommand = cWAngle.commandId;
            float angle = cWAngle.angle;
            itsValid = true;
            switch (cWAngle.commandType)
            {
                case CommandType.Rotate:
                    action = Action.Rotate;
                    break;
                default:
                    itsValid = false;
                    break;
            }
            if (itsValid)
            {
                runningCommands.Add(scm.sendCommand(idCommand, action, angle));
            }
        }
        else if (data.GetType() == typeof(CommandWithoutPar))
        {
            CommandWithoutPar cWPar = (CommandWithoutPar)data;
            idCommand = cWPar.commandId;
            bool isAHandCommand = true;
            switch (cWPar.commandType)
            {
                case CommandType.HeadReset:
                    action = Action.HeadReset;
                    isAHandCommand = false;
                    break;
                case CommandType.TasteLeft:
                    action = Action.Taste;
                    hand = Hands.Left;
                    break;
                case CommandType.TasteRight:
                    hand = Hands.Right;
                    action = Action.Taste;
                    break;
                case CommandType.SmellLeft:
                    hand = Hands.Left;
                    action = Action.Smell;
                    break;
                case CommandType.SmellRight:
                    hand = Hands.Right;
                    action = Action.Smell;
                    break;
                case CommandType.CancelCommands:
                    action = Action.Cancel;
                    isAHandCommand = false;
                    break;
                default:
                    itsValid = false;
                    break;
            }
            if (itsValid)
            {
                if(isAHandCommand)
                    runningCommands.Add(scm.sendCommand(idCommand,hand, action));
                else
                    runningCommands.Add(scm.sendCommand(idCommand, action));
            }
        }
        else if (data.GetType() == typeof(CommandWithPosition))
        {
            CommandWithPosition cWPosition = (CommandWithPosition)data;
            idCommand = cWPosition.commandId;
            hand = Hands.Right;
            bool isCommWithHand = false;

            switch (cWPosition.commandType)
            {                
                case CommandType.LeaveLeft:
                    hand = Hands.Left;
                    isCommWithHand = true;
                    action = Action.Release;
                    break;
                case CommandType.LeaveRight:
                    isCommWithHand = true;
                    action = Action.Release;
                    break;
                case CommandType.LookAt:
                    action = Action.HeadFocus;
                    break;
                case CommandType.Move:
                    action = Action.Move;
                    break;
                case CommandType.Turn:
                    action = Action.Turn;
                    break;
                default:
                    itsValid = false;
                    break;
            }
            if (itsValid)
            {
                Vector3 auxPosition = new Vector3(cWPosition.position3.x, cWPosition.position3.y, cWPosition.position3.z);
                runningCommands.Add(scm.sendCommand(idCommand, hand, action, auxPosition));
               
            }
        }
        else if (data.GetType() == typeof(CommandWithString))
        {
            CommandWithString cWString = (CommandWithString)data;
            idCommand = cWString.commandId;
            hand = Hands.Right;

            switch (cWString.commandType)
            {

                case CommandType.Speech:
                    action = Action.Speak;
                    break;
                case CommandType.LookFor:
                    action = Action.LookFor;
                    break;
                default:
                    itsValid = false;
                    break;
            }
            if (itsValid)
            {
                string str = cWString.str;
                runningCommands.Add(scm.sendCommand(idCommand,  action, str));
            }
        }
        else
        {
            itsValid = false;
            idCommand = data.ToString();
        }
        if (!itsValid)
        {
            Response response = new Response { idCommand = idCommand, executed = false };
            sendResponse(response);
        }
    }

    private GameObject getGameObjectById(int id)
    {
        GameObject gO = null;
        if (uid.isValidId(id)){
            gO = uid.getGameObjectById(id);
            if (scm.getAllPerceivedElements().Contains(gO))
            {
                return gO;
            }else
            {
                return null;
            }
        }
        return null;
    }

    private void sendResponse(Response data)
    {
        try
        {
            Type type = data.GetType();
            int field = typeLookup.Single(pair => pair.Value == type).Key;
            Serializer.NonGeneric.SerializeWithLengthPrefix(client.tcp.GetStream(), data, PrefixStyle.Base128, field);
            client.tcp.GetStream().Flush();
            if(data.executed)
                Debug.Log("RHS>>>  response sended to cliente: " + data.idCommand + " command finalized with success.");
            else
                Debug.Log("RHS>>>  response sended to cliente: " + data.idCommand + " command finalized with fail.");
        }
        catch (Exception e)
        {
            Debug.Log("System>>> Write error: " + e.Message + " to client " + client.clientName);
        }
    }
    
   /* public void sendCommand(string )
    {
        string typeParameter = Command.DictActions[Action.Activate].typeParameter;
        if (typeParameter.Equals(Constants.PAR_ROTATION))
        {
            scm.sendCommand(getSelectedActionItem(), sliderRotation.value);
        }
        else if (typeParameter.Equals(Constants.PAR_STRING))
        {
            scm.sendCommand(getSelectedActionItem(), inputFieldLookFor.text);
        }
        else if (typeParameter.Equals(Constants.PAR_NULL))
        {
            scm.sendCommand(getSelectedActionItem());
        }
        else
        {
            Hands hand = Hands.Right;
            if (groupToggleHand.isActiveAndEnabled && groupToggleHand.GetActive().name.Equals(Constants.TGGL_LEFT))
            {
                hand = Hands.Left;
            }
            if (getSelectedActionItem() == Action.Taste)
            {
                scm.sendCommand(hand, Action.Taste);
            }
            else
            {
                Transform auxTransform = getListOfGameObjects()[dropdownElements.value].transform;
                scm.sendCommand(hand, getSelectedActionItem(), auxTransform);
            }
        }
    }*/
    
}
[Serializable]
[ProtoContract]
public class ServerClient
{
    [ProtoMember(1)]
    public TcpClient tcp;
    [ProtoMember(2)]
    public string clientName;

    public ServerClient(TcpClient clientSocket)
    {
        clientName = "Driver";
        tcp = clientSocket;
    }
}


[ProtoContract]
class CommandWithId
{
    [ProtoMember(1)]
    public string commandId { get; set; }
    [ProtoMember(2)]
    public CommandType commandType { get; set; }
    [ProtoMember(3)]
    public int objectId { get; set; }

    public override string ToString()
    {
        return "CommandId: " + commandId + "\nCommand Type: " + commandType + "\nObjectId: " + objectId;
    }
}

[Serializable]
[ProtoContract]
class CommandWithPosition
{
    [ProtoMember(1)]
    public string commandId { get; set; }
    [ProtoMember(2)]
    public CommandType commandType { get; set; }
    [ProtoMember(3)]
    public Position3 position3 { get; set; }

    public override string ToString()
    {
        return "CommandId: " + commandId + "\nCommand Type: " + commandType + "\nObjectId: " + position3;
    }
}
[Serializable]
[ProtoContract]
class CommandWithString
{
    [ProtoMember(1)]
    public string commandId { get; set; }
    [ProtoMember(2)]
    public CommandType commandType { get; set; }
    [ProtoMember(3)]
    public string str { get; set; }

    public override string ToString()
    {
        return "CommandId: " + commandId + "\nCommand Type: " + commandType + "\nString: " + str;
    }
}
[Serializable]
[ProtoContract]
class CommandWithAngle
{
    [ProtoMember(1)]
    public string commandId { get; set; }
    [ProtoMember(2)]
    public CommandType commandType { get; set; }
    [ProtoMember(3)]
    public float angle { get; set; }

    public override string ToString()
    {
        return "CommandId: " + commandId + "\nCommand Type: " + commandType + "\nAngle: " + angle;
    }
}
[Serializable]
[ProtoContract]
class CommandWithoutPar
{
    [ProtoMember(1)]
    public string commandId { get; set; }
    [ProtoMember(2)]
    public CommandType commandType { get; set; }

    public override string ToString()
    {
        return "CommandId: " + commandId + "\nCommand Type: " + commandType;
    }
}
[Serializable]
[ProtoContract]
class Position3
{
    [ProtoMember(1)]
    public float x { get; set; }
    [ProtoMember(2)]
    public float y { get; set; }
    [ProtoMember(3)]
    public float z { get; set; }

    public override string ToString()
    {
        return "Position: " + x + ", " + y + ", " + z;
    }
}


[Serializable]
[ProtoContract]
class Response
{
    [ProtoMember(1)]
    public string idCommand { get; set; }
    [ProtoMember(2)]
    public bool executed { get; set; }

    public override string ToString()
    {
        string auxString = "success";
        if (!executed)
        {
            auxString = "fail";
        }
        return "Command Id: " + idCommand + " " + auxString + "!";
    }
}


