using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using MathNet.Numerics.LinearAlgebra.Double;

//[System.Serializable]
public class Robot {

    public GameObject robotBase;
    public List<GameObject> joints;
    public GameObject robotTip;
    public List<float> linkLenghes { get; private set; }
    public List<Quaternion> initRotations { get; private set; }

    public Robot(GameObject robotBase, List<GameObject> joints, GameObject robotTip) {
        this.robotBase = robotBase;
        this.joints = joints;
        this.robotTip = robotTip;

        this.linkLenghes = new List<float>();
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
    public void setJointAnglesRadian(List<float> angles) {
        if (angles.Count < this.joints.Count) {
            return;
        }
        for (int i = 0; i < this.joints.Count; i++) {
            this.setJointAngle(i, angles[i]*Mathf.Rad2Deg);
        }
    }

    public List<float> getAngles() {
        var ret = new List<float>();
        float ang = 0.0f;
        for (int i = 0; i < this.joints.Count; i++) {
            ang = (Quaternion.Inverse(this.initRotations[i]) * this.joints[i].transform.localRotation).eulerAngles.y;
            ang = ang < 180.0f ? ang : ang - 360.0f;
            ret.Add(ang);
        }
        return ret;
    }

    public List<double> getAnglesd() {
        var ret = new List<double>();
        foreach (var ang in this.getAngles()) {
            ret.Add((double)ang);
        }
        return ret;
    }

    public List<double> getAnglesRadian() {
        return getAnglesd().Select(ang => ang * Mathf.Deg2Rad).ToList();
    }
}

public class Controller {
    public List<float> linkLenghes { get; private set; }

    public Controller(List<float> linkLenghes) {
        this.linkLenghes = linkLenghes;
    }

    public DenseVector getEEPosVec(DenseVector angles) {
        Func<int, float> l = (int i) => {
            return this.linkLenghes[i];
        };
        Func<int, float> th = (int i) => {
            return (float)angles[i - 1];
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
        return new DenseVector(new[] { x, y, z });
    }

    public DenseMatrix getInversedJacobiFromSVD(DenseVector angles) {
        DenseMatrix jacobi = this.getJacobi(angles);
        var svd = jacobi.Svd(true);
        var s = new DiagonalMatrix(jacobi.RowCount, jacobi.ColumnCount, (1 / svd.S).ToArray());
        // 疑似逆行列を計算する
        var m = svd.VT.Transpose() * s.Transpose() * svd.U.Transpose();
        return (DenseMatrix)m;
    }

    public DenseMatrix getInversedJacobi(DenseVector angles) {
        DenseMatrix jacobi = this.getJacobi(angles);
        return (DenseMatrix)(jacobi.Transpose() * (jacobi * jacobi.Transpose()).Inverse());
    }

    public DenseMatrix getJacobi(DenseVector angles) {
        Func<int, float> l = (int i) => {
            return this.linkLenghes[i];
        };
        Func<int, float> th = (int i) => {
            return (float)angles[i - 1];
        };
        Func<int, float> s = (int i) => {
            return Mathf.Sin(th(i));
        };
        Func<int, float> c = (int i) => {
            return Mathf.Cos(th(i));
        };

        var m = new DenseMatrix(3, 4);
        m[0, 0] = l(1) * c(1) + l(2) * c(1) * c(2) + l(3) * (c(1) * c(2) * c(3) - s(1) * s(3)) + l(4) * (c(1) * c(2) * c(3) * c(4) - s(1) * s(3) * c(4) - c(1) * s(2) * s(4));
        m[0, 1] = -l(2) * s(1) * s(2) - l(3) * s(1) * s(2) * c(3) - l(4) * (s(1) * s(2) * c(3) * c(4) + s(1) * c(2) * s(4));
        m[0, 2] = -l(3) * (s(1) * c(2) * s(3) - c(1) * c(3)) - l(4) * (s(1) * c(2) * s(3) * c(4) - c(1) * c(3) * c(4));
        m[0, 3] = -l(4) * (s(1) * c(2) * c(3) * s(4) + c(1) * s(3) * s(4) + s(1) * s(2) * c(4));

        m[1, 0] = 0.0f;
        m[1, 1] = -l(2) * c(2) - l(3) * c(2) * c(3) - l(4) * (c(2) * c(3) * c(4) - s(2) * s(4));
        m[1, 2] = l(3) * s(2) * s(3) + l(4) * s(2) * s(3) * c(4);
        m[1, 3] = l(4) * (s(2) * c(3) * s(4) - c(2) * c(4));

        m[2, 0] = -l(1) * s(1) - l(2) * s(1) * c(2) - l(3) * (s(1) * c(2) * c(3) + c(1) * s(3)) - l(4) * (s(1) * c(2) * c(3) * c(4) + c(1) * s(3) * c(4) - s(1) * s(2) * s(4));
        m[2, 1] = -l(2) * c(1) * s(2) - l(3) * c(1) * s(2) * c(3) - l(4) * (c(1) * s(2) * c(3) * c(4) + c(1) * c(2) * s(4));
        m[2, 2] = -l(3) * (c(1) * c(2) * s(3) + s(1) * c(3)) - l(4) * (c(1) * c(2) * s(3) * c(4) + s(1) * c(3) * c(4));
        m[2, 3] = -l(4) * (c(1) * c(2) * c(3) * s(4) - s(1) * s(3) * s(4) + c(1) * s(2) * c(4));

        return m;
    }

    public bool isClose(DenseVector angles, Vector3 targetPosition, float ipsilon = 0.0001f) {
        var pos = this.getEEPosVec(angles);
        float diff = Vector3.Distance(targetPosition, Controller.toVector3(pos));
        float normalized = Mathf.Abs(diff / targetPosition.magnitude);
        return (normalized < ipsilon);
    }

    static public Vector3 toVector3(DenseVector v) {
        return new Vector3((float)v[0], (float)v[1], (float)v[2]);
    }

    static public DenseVector toDenseVec(Vector3 v) {
        return DenseVector.OfArray(new double[] { (double)v.x, (double)v.y, (double)v.z });
    }
}



public class RobotController : MonoBehaviour {

    public bool startButton = false;
    public GameObject target;
    public GameObject robotBase;
    public List<GameObject> joints;
    public GameObject robotTip;

    private Robot robot;
    private Controller ctrler;

    // Use this for initialization
    void Awake() {
    }
    void Start() {
        robot = new Robot(robotBase, joints, robotTip);
        ctrler = new Controller(this.robot.linkLenghes);
        this.robot.setJointAngles(new List<float> { 45, -25, -25, 90 });
        var rightHandedEEpos = Controller.toVector3(this.ctrler.getEEPosVec(DenseVector.OfArray(this.robot.getAnglesRadian().ToArray())));
        this.target.transform.localPosition = right2leftHand(rightHandedEEpos);

        //testMatrix();
    }

    // Update is called once per frame
    void Update() {
        var targetPos = left2rightHand(this.target.transform.localPosition);
        var angles = DenseVector.OfArray(this.robot.getAnglesRadian().ToArray());
        print(Vector3.Distance(Controller.toVector3(this.ctrler.getEEPosVec(angles)), Controller.toVector3(Controller.toDenseVec(targetPos))));

        for (int i = 1; (i < 1000) && this.startButton; i++) {
            angles = angles - this.ctrler.getInversedJacobi(angles) * (this.ctrler.getEEPosVec(angles) - Controller.toDenseVec(targetPos));
            print(angles);
            if (this.ctrler.isClose(angles, targetPos)) {
                this.robot.setJointAnglesRadian(new List<float> { (float)angles[0], (float)angles[1], (float)angles[2], (float)angles[3] });
                this.startButton = false;
                break;
            }
        }
    }

    public static Vector3 left2rightHand(Vector3 left) {
        return new Vector3(left.x, -left.y, left.z);
    }
    public static Vector3 right2leftHand(Vector3 right) {
        return new Vector3(right.x, -right.y, right.z);
    }

    void testMatrix() {

        var M1 = DenseMatrix.OfArray(new double[,] { { 12, 23, 32, 2 }, { 43, 5, 61, 9 }, { 3, 25, 41, 39 } });
        print(M1);
        print(M1.Transpose());
        var M2 = (M1.Transpose()) * M1;
        print(M2.Rank());
        var M2inv = M2.Inverse();
        print(M2);
        print(M2inv);
        print(M2 * M2inv);
    }

}












