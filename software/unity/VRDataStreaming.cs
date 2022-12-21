using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Networking;


/**
 * Json serialization class
 * 
 * The first letter stands for what's being tracked. 
 * H: head, R: right head, L: left hand
 * 
 * The second letter stands for either P: postion or R: rotation
 * 
 * The third letter stands for the one axis in 3D - x, y, z
 * 
 * Abbreviation is used for reducing bytes in the package.
 */
[Serializable]
public class JsonStreamData
{
    public float HPX;
    public float HPY;
    public float HPZ;
    public float HRX;
    public float HRY;
    public float HRZ;
    public float LPX;
    public float LPY;
    public float LPZ;
    public float LRX;
    public float LRY;
    public float LRZ;
    public float RPX;
    public float RPY;
    public float RPZ;
    public float RRX;
    public float RRY;
    public float RRZ;
}

[Serializable]
public class JsonStreamResult
{
    public float[] chest;
    public float[] head;
    public float[] left_hand;
    public float[] left_lower_arm;
    public float[] left_lower_leg;
    public float[] left_upper_arm;
    public float[] left_upper_leg;
    public float[] right_hand;
    public float[] right_lower_arm;
    public float[] right_lower_leg;
    public float[] right_upper_arm;
    public float[] right_upper_leg;
    public float[] waist;
}

public class VRDataStreaming : MonoBehaviour
{
    public Transform HMD;
    public Transform LeftCtrl;
    public Transform RightCtrl;

    public Transform rightUpperLeg;
    public Transform leftUpperLeg;

    public Transform rightLowerLeg;
    public Transform leftLowerLeg;

    public Transform rightUpperArm;
    public Transform leftUpperArm;

    public Transform rightLowerArm;
    public Transform leftLowerArm;

    public Transform head;
    public Transform waist;
    public Transform chest;

    public Transform avatar;

    JsonStreamResult poseData;

    bool uploadFinished = true;
    bool requestFinished = true;

    // Start is called before the first frame update
    void Start()
    {

        rightUpperLeg = GameObject.Find("RightUpperLeg").GetComponent<Transform>();
        leftUpperLeg = GameObject.Find("LeftUpperLeg").GetComponent<Transform>();

        rightLowerLeg = GameObject.Find("RightLowerLeg").GetComponent<Transform>();
        leftLowerLeg = GameObject.Find("LeftLowerLeg").GetComponent<Transform>();

        rightUpperArm = GameObject.Find("RightUpperArm").GetComponent<Transform>();
        leftUpperArm = GameObject.Find("LeftUpperArm").GetComponent<Transform>();

        rightLowerArm = GameObject.Find("RightLowerArm").GetComponent<Transform>();
        leftLowerArm = GameObject.Find("LeftLowerArm").GetComponent<Transform>();

        head = GameObject.Find("Head").GetComponent<Transform>();
        waist = GameObject.Find("Spine").GetComponent<Transform>();
        chest = GameObject.Find("Chest").GetComponent<Transform>();
    }
        // Update is called once per frame
    void Update()
    {

        JsonStreamData jsonStreamData = new JsonStreamData();
        jsonStreamData.HPX = HMD.position.x;
        jsonStreamData.HPY = HMD.position.y;
        jsonStreamData.HPZ = HMD.position.z;
        jsonStreamData.HRX = HMD.localRotation.eulerAngles.x;
        jsonStreamData.HRY = HMD.localRotation.eulerAngles.y;
        jsonStreamData.HRZ = HMD.localRotation.eulerAngles.z;
        jsonStreamData.LPX = LeftCtrl.position.x;
        jsonStreamData.LPY = LeftCtrl.position.y;
        jsonStreamData.LPZ = LeftCtrl.position.z;
        jsonStreamData.LRX = LeftCtrl.localRotation.eulerAngles.x;
        jsonStreamData.LRY = LeftCtrl.localRotation.eulerAngles.y;
        jsonStreamData.LRZ = LeftCtrl.localRotation.eulerAngles.z;
        jsonStreamData.RPX = RightCtrl.position.x;
        jsonStreamData.RPY = RightCtrl.position.y;
        jsonStreamData.RPZ = RightCtrl.position.z;
        jsonStreamData.RRX = RightCtrl.localRotation.eulerAngles.x;
        jsonStreamData.RRY = RightCtrl.localRotation.eulerAngles.y;
        jsonStreamData.RRZ = RightCtrl.localRotation.eulerAngles.z;

        if (uploadFinished)
        {
            uploadFinished = false;
            StartCoroutine(Upload(jsonStreamData));
        }

        if (requestFinished)
        {
            requestFinished = false;
            StartCoroutine(Request());
        }

    }

    IEnumerator Upload(JsonStreamData jsonStreamData)
    {

        string streamOut = JsonUtility.ToJson(jsonStreamData);
        Debug.Log(streamOut);

        List<IMultipartFormSection> formData = new List<IMultipartFormSection>();
        formData.Add(new MultipartFormDataSection("tracking", streamOut));

        UnityWebRequest www = UnityWebRequest.Post("http://192.168.1.100:5000" + "/unity-runtime/headset-data", formData);
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success)
        {
            Debug.Log(www.error);
        }
        else
        {
            Debug.Log("Form upload complete!");
        }

        uploadFinished = true;

        www.Dispose();
    }

    IEnumerator Request()
    {
        UnityWebRequest www = UnityWebRequest.Get("http://192.168.1.100:5000" + "/unity-runtime/pose-data");
        yield return www.SendWebRequest();

        if (www.result != UnityWebRequest.Result.Success)
        {
            Debug.Log("HTTP ERROR");
            Debug.Log(www.error);
        }
        else
        {

            poseData = JsonUtility.FromJson<JsonStreamResult>(www.downloadHandler.text);
            Debug.Log(JsonUtility.ToJson(poseData));

            // Or retrieve results as binary data
            byte[] results = www.downloadHandler.data;

            rightUpperLeg.localRotation = Quaternion.Euler(poseData.right_upper_leg[0], poseData.right_upper_leg[1], poseData.right_upper_leg[2]);
            leftUpperLeg.localRotation = Quaternion.Euler(poseData.left_upper_leg[0], poseData.left_upper_leg[1], poseData.left_upper_leg[2]);
            
            rightLowerLeg.localRotation = Quaternion.Euler(poseData.right_lower_leg[0], poseData.right_lower_leg[1], poseData.right_lower_leg[2]);
            leftLowerLeg.localRotation = Quaternion.Euler(poseData.left_lower_leg[0], poseData.left_lower_leg[1], poseData.left_lower_leg[2]);

            rightUpperArm.localRotation = Quaternion.Euler(poseData.right_upper_arm[0], poseData.right_upper_arm[1], poseData.right_upper_arm[2]);
            leftUpperArm.localRotation = Quaternion.Euler(poseData.left_upper_arm[0], poseData.left_upper_arm[1], poseData.left_upper_arm[2]);

            rightLowerArm.localRotation = Quaternion.Euler(poseData.right_lower_arm[0], poseData.right_lower_arm[1], poseData.right_lower_arm[2]);
            leftLowerArm.localRotation = Quaternion.Euler(poseData.left_lower_arm[0], poseData.left_lower_arm[1], poseData.left_lower_arm[2]);

            head.localRotation = Quaternion.Euler(poseData.head[0], poseData.head[1], poseData.head[2]);
            waist.localRotation = Quaternion.Euler(poseData.waist[0], poseData.waist[1], poseData.waist[2]);
            chest.localRotation = Quaternion.Euler(poseData.chest[0], poseData.chest[1], poseData.chest[2]);

            // --------------------another method----------------------
            leftUpperLeg.localRotation = Quaternion.Euler(-180, 0, 0);
            rightUpperLeg.localRotation = Quaternion.Euler(-180, 0, 0);
            leftLowerLeg.localRotation = Quaternion.Euler(0, 0, 0);
            rightLowerLeg.localRotation = Quaternion.Euler(0, 0, 0);
            leftUpperArm.localRotation = Quaternion.Euler(-75, 81, -71);
            leftLowerArm.localRotation = Quaternion.Euler(0, 0, 0);
            rightUpperArm.localRotation = Quaternion.Euler(-75, -81, 71);
            rightLowerArm.localRotation = Quaternion.Euler(0, 0, 0);
            head.localRotation = Quaternion.Euler(0, 0, 0);
            waist.localRotation = Quaternion.Euler(0, 0, 0);
            chest.localRotation = Quaternion.Euler(0, 0, 0);

            leftUpperLeg.Rotate(poseData.left_upper_leg[0], poseData.left_upper_leg[1], poseData.left_upper_leg[2],Space.World);
            rightUpperLeg.Rotate(poseData.right_upper_leg[0], poseData.right_upper_leg[1], poseData.right_upper_leg[2],Space.World);

            leftLowerLeg.Rotate(poseData.left_lower_leg[0], poseData.left_lower_leg[1], poseData.left_lower_leg[2], Space.World);
            rightLowerLeg.Rotate(poseData.right_lower_leg[0], poseData.right_lower_leg[1], poseData.right_lower_leg[2], Space.World);

            rightUpperArm.Rotate(poseData.right_upper_arm[0], poseData.right_upper_arm[1], poseData.right_upper_arm[2], Space.World);
            leftUpperArm.Rotate(poseData.left_upper_arm[0], poseData.left_upper_arm[1], poseData.left_upper_arm[2], Space.World);

            rightLowerArm.Rotate(poseData.right_lower_arm[0], poseData.right_lower_arm[1], poseData.right_lower_arm[2], Space.World);
            leftLowerArm.Rotate(poseData.left_lower_arm[0], poseData.left_lower_arm[1], poseData.left_lower_arm[2], Space.World);

            head.Rotate(poseData.head[0], poseData.head[1], poseData.head[2], Space.World);
            waist.Rotate(poseData.waist[0], poseData.waist[1], poseData.waist[2], Space.World);
            chest.Rotate(poseData.chest[0], poseData.chest[1], poseData.chest[2], Space.World);

        }

        requestFinished = true;
        www.Dispose();
    }
    
}
