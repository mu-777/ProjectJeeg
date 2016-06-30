using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

public class RobotTransformVisualizer : MonoBehaviour {

    public GameObject robotBase;
    public List<GameObject> joints;
    public GameObject robotTip;

    [Range(-90.0f, 90.0f)]
    public List<float> angles;

    private List<float> linkLenghes = new List<float>();
    private List<Quaternion> initRotations = new List<Quaternion>();

    // Use this for initialization
    void Start() {
        List<GameObject> allJoints = new List<GameObject>(this.joints);
        allJoints.Insert(0, this.robotBase);
        allJoints.Add(robotTip);

        allJoints.Aggregate((prev, curr) => {
            this.linkLenghes.Add(Vector3.Distance(prev.transform.position, curr.transform.position));
            return curr;
        });

        foreach (var joint in this.joints) {
            initRotations.Add(joint.transform.localRotation);
        }
        //initRotations = (List<Quaternion>)this.joints.Select(x => x.transform.localRotation);
    }

    // Update is called once per frame
    void Update() {
        if (this.angles.Count < this.joints.Count) {
            return;
        }
        MoveEachJoints();
        CalcTipPosFromTransformMatrix();
    }

    private void MoveEachJoints() {
        for (int i = 0; i < this.joints.Count; i++) {
            this.joints[i].transform.localRotation = this.initRotations[i] * Quaternion.AngleAxis(this.angles[i], Vector3.up);
        }
    }
    private void CalcTipPosFromTransformMatrix() {
        Func<int, float> l = (int i) => {
            return this.linkLenghes[i];
        };
        Func<int, float> th = (int i) => {
            return this.angles[i - 1] * Mathf.Deg2Rad;
        };
        Func<int, float> s = (int i) => {
            return Mathf.Sin(th(i));
        };
        Func<int, float> c = (int i) => {
            return Mathf.Cos(th(i));
        };


        if (this.linkLenghes.Count < this.joints.Count + 1) {
            print("linkLenghes: " + linkLenghes.Count);
            return;
        }

        float x = l(1) * s(1) + l(2) * s(1) * c(2) + l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) + l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - c(1) * s(4));
        float y = -l(2) * s(2) - l(3) * s(2) * c(3) + l(4) * (-s(2) * c(3) * s(4) + c(2) * s(4));
        float z = l(0) + l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * c(4) + c(1) * s(2) * s(4));
        print(x + ", " + y + ", " + z);
        this.transform.localPosition = new Vector3(x, y, z);
    }

}
