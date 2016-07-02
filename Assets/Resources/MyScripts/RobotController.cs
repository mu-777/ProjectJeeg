using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;


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

        public List<float> getAngles() {
            var ret = new List<float>();
            for (int i = 0; i < this.joints.Count; i++) {
                ret.Add((Quaternion.Inverse(this.initRotations[i]) * this.joints[i].transform.localRotation).eulerAngles.y);
            }
            return ret;
        }
    }

    public class Controller {
        public List<float> linkLenghes { get; private set; }
        public Controller(List<float> linkLenghes) {
            this.linkLenghes = linkLenghes;
        }

        public DenseVector getEEPosVec(List<float> angles) {
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

            double x = l(1) * s(1) + l(2) * s(1) * c(2) + l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) + l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - s(1) * s(2) * s(4));
            double y = -l(2) * s(2) - l(3) * s(2) * c(3) - l(4) * (s(2) * c(3) * c(4) + c(2) * s(4));
            double z = l(0) + l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * c(4) - c(1) * s(2) * s(4));
            print(x + ", " + y + ", " + z);
            return new DenseVector(new [] { x, y, z });
        }

        public DenseMatrix getInversedJacobi(List<float> angles) {
            DenseMatrix jacobi = this.getJacobi(angles);
            var svd = jacobi.Svd(true);
            var s = new DiagonalMatrix(jacobi.RowCount, jacobi.ColumnCount, (1 / svd.S).ToArray());
            // 疑似逆行列を計算する
            var m = svd.VT.Transpose() * s.Transpose() * svd.U.Transpose();
            return (DenseMatrix) m;
        }

        public DenseMatrix getJacobi(List<float> angles) {
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


            var m =  new DenseMatrix(3, 4);
            m[0, 0] = l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * s(4) - c(1) * s(2) * s(4));
            m[0, 1] = -l(2) * s(1) * s(2) - l(3) * s(1) * s(2) * c(3) - l(4) * (s(1) * s(2) * c(3) * c(4) + s(1) * c(2) * s(4));
            m[0, 2] = -l(3) * (s(1) * c(2) * s(3) - c(1) * c(3)) - l(4) * (s(1) * c(2) * s(3) * c(4) - c(1) * c(3) * c(4));
            m[0, 3] = -l(4) * (s(1) * c(2) * c(3) * s(4) + c(1) * s(3) * s(4) + s(1) * s(2) * c(4));

            m[1, 0] = 0.0f;
            m[1, 1] = -l(2) * c(2) - l(3) * c(2) * c(3) - l(4) * (c(2) * c(3) * c(4) - s(2) * s(4));
            m[1, 2] = l(3) * s(2) * s(3) + l(4) * s(2) * s(3) * c(4);
            m[1, 3] = l(4) * (s(2) * c(3) * s(4) - c(2) * c(4));

            m[2, 0] = -l(1) * s(1) - l(2) * s(1) * c(2) - l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) - l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - s(1) * s(2) * s(4));
            m[2, 1] = -l(2) * c(1) * s(2) - l(3) * c(1) * s(2) * c(3) - l(4) * (c(1) * c(2) * s(3) * c(4) + c(1) * c(2) * s(4));
            m[2, 2] = -l(3) * (c(1) * c(2) * s(3) + s(1) * c(3)) - l(4) * (c(1) * c(2) * s(3) * c(4) + s(1) * c(3) * c(4));
            m[2, 3] = -l(4) * (c(1) * c(2) * c(3) * s(4) - s(1) * s(3) * s(4) + c(1) * s(2) * s(4));

            return m;
        }

        public bool isClose(List<float> angles, Vector3 targetPosition , float ipsilon = 0.00001f) {
            var pos = this.getEEPosVec(angles);
            float diff = Vector3.Distance(targetPosition , new Vector3(pos[0], pos[1], pos[2]));
            float normalized = Mathf.Abs(diff / targetPosition.magnitude);
            return (normalized < ipsilon);
        }
    }

    public bool startButton = false;
    public GameObject target;
    public Robot robot;
    private Controller ctrler;

    // Use this for initialization
    void Start() {
        ctrler = new Controller(this.robot.linkLenghes);
    }

    // Update is called once per frame
    void Update() {
        if (!this.startButton) {
            return;
        }
        var targetPos = this.target.transform.localPosition;
        var angles = this.robot.getAngles();
        while (true) {
            angles = angles - this.ctrler.getInversedJacobi(angles) * (this.ctrler.getEEPosition(angles) - targetPos);
        }
    }
}












