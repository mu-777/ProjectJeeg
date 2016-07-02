using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;


public class RobotController : MonoBehaviour {

    [System.Serializable]
    public class Robot {

        public GameObject robotBase;
        public List<GameObject> joints;
        public GameObject robotTip;
        public List<float> linkLenghes { get; private set; }
        public List<Quaternion> initRotations { get; private set; }

        public Robot() {
            this.linkLenghes =  new List<float>();
            this.initRotations = new List<Quaternion>();

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

        }
        public void setJointAngle(int idx, float ang) {
            this.joints[idx].transform.localRotation = this.initRotations[idx] * Quaternion.AngleAxis(ang, Vector3.up);
        }

        public void setJointAngles(List<float> angles) {
            if (angles.Count < this.joints.Count) {
                return;
            }
            for (int i = 0; i < this.joints.Count; i++) {
                this.setJointAngle(i, angles[i]);
            }
        }
        

    }

    public class Controller {
        public List<float> linkLenghes { get; private set; }
        public Controller(List<float> linkLenghes) {
            this.linkLenghes = linkLenghes;
        }

        public Vector3 getEEPosition(List<float> angles) {
            Func<int, float> l = (int i) => {
                return this.linkLenghes[i];
            };
            Func<int, float> th = (int i) => {
                return angles[i - 1] * Mathf.Deg2Rad;
            };
            Func<int, float> s = (int i) => {
                return Mathf.Sin(th(i));
            };
            Func<int, float> c = (int i) => {
                return Mathf.Cos(th(i));
            };

            float x = l(1) * s(1) + l(2) * s(1) * c(2) + l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) + l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - s(1) * s(2) * s(4));
            float y = -l(2) * s(2) - l(3) * s(2) * c(3) - l(4) * (s(2) * c(3) * c(4) + c(2) * s(4));
            float z = l(0) + l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * c(4) - c(1) * s(2) * s(4));
            print(x + ", " + y + ", " + z);
            return new Vector3(x, -y, z);
        }

        public List<List<float>> getJacobi(List<float> angles) {
            Func<int, float> l = (int i) => {
                return this.linkLenghes[i];
            };
            Func<int, float> th = (int i) => {
                return angles[i - 1] * Mathf.Deg2Rad;
            };
            Func<int, float> s = (int i) => {
                return Mathf.Sin(th(i));
            };
            Func<int, float> c = (int i) => {
                return Mathf.Cos(th(i));
            };


            var m =  new List<List<float>>();
            m[0][0] = l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * s(4) - c(1) * s(2) * s(4));
            m[0][1] = -l(2) * s(1) * s(2) - l(3) * s(1) * s(2) * c(3) - l(4) * (s(1) * s(2) * c(3) * c(4) + s(1) * c(2) * s(4));
            m[0][2] = -l(3) * (s(1) * c(2) * s(3) - c(1) * c(3)) - l(4) * (s(1) * c(2) * s(3) * c(4) - c(1) * c(3) * c(4));
            m[0][3] = -l(4) * (s(1) * c(2) * c(3) * s(4) + c(1) * s(3) * s(4) + s(1) * s(2) * c(4));

            m[1][0] = 0.0f;
            m[1][1] = -l(2) * c(2) - l(3) * c(2) * c(3) - l(4) * (c(2) * c(3) * c(4) - s(2) * s(4));
            m[1][2] = l(3) * s(2) * s(3) + l(4) * s(2) * s(3) * c(4);
            m[1][3] = l(4) * (s(2) * c(3) * s(4) - c(2) * c(4));

            m[2][0] = -l(1) * s(1) - l(2) * s(1) * c(2) - l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) - l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - s(1) * s(2) * s(4));
            m[2][1] = -l(2) * c(1) * s(2) - l(3) * c(1) * s(2) * c(3) - l(4) * (c(1) * c(2) * s(3) * c(4) + c(1) * c(2) * s(4));
            m[2][2] = -l(3) * (c(1) * c(2) * s(3) + s(1) * c(3)) - l(4) * (c(1) * c(2) * s(3) * c(4) + s(1) * c(3) * c(4));
            m[2][3] = -l(4) * (c(1) * c(2) * c(3) * s(4) - s(1) * s(3) * s(4) + c(1) * s(2) * s(4));

            return m;
        }

    }

    public bool startButton = false;
    public GameObject target;
    public Robot robot;


    // Use this for initialization
    void Start() {

    }

    // Update is called once per frame
    void Update() {
        if (!this.startButton) {
            return;
        }


    }
}












