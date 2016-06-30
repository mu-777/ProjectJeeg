using UnityEngine;
using System.Collections;

public class MyUtils : MonoBehaviour {
    public static GameObject cloneGameObject(GameObject baseGameObject, GameObject parentGameObject) {
        GameObject ret = new GameObject();
        ret.transform.position = baseGameObject.transform.position;
        ret.transform.rotation = baseGameObject.transform.rotation;
        if (parentGameObject == null) {
            ret.transform.parent = null;
        } else {
            ret.transform.parent = parentGameObject.transform;
        }
        return ret;
    }

    public static Transform cloneTransform(Vector3 pos, Quaternion rot, Transform parent = null) {
        Transform ret = new GameObject().transform;
        ret.position = pos;
        ret.rotation = rot;
        ret.parent = parent;
        return ret;
    }

    public static void deepCopyTransform(Transform to, Vector3 pos, Quaternion rot, Transform parent = null) {
        to.position = pos;
        to.rotation = rot;
        to.parent = parent;
    }

    public static void LookAtExtended(Transform focusedTF, Transform targetTF, Vector3 forward) {
        Quaternion lookrot = Quaternion.LookRotation(targetTF.position - focusedTF.position);
        focusedTF.rotation = lookrot * Quaternion.FromToRotation(forward, Vector3.forward);
    }

}
