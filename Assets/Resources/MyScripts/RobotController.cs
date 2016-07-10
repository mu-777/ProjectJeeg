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
    public void setJointAnglesRadian(float angle1, float angle2, float angle3, float angle4) {
        this.setJointAngle(0, angle1 * Mathf.Rad2Deg);
        this.setJointAngle(1, angle2 * Mathf.Rad2Deg);
        this.setJointAngle(2, angle3 * Mathf.Rad2Deg);
        this.setJointAngle(3, angle4 * Mathf.Rad2Deg);
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

    public float getDistance(DenseVector angles, Vector3 targetPosition) {
        return Vector3.Distance(Controller.toVector3(this.getEEPosVec(angles)), Controller.toVector3(Controller.toDenseVec(targetPosition)));
    }


    static public Vector3 toVector3(DenseVector v) {
        return new Vector3((float)v[0], (float)v[1], (float)v[2]);
    }

    static public DenseVector toDenseVec(Vector3 v) {
        return DenseVector.OfArray(new double[] { (double)v.x, (double)v.y, (double)v.z });
    }
}



public class RobotController : MonoBehaviour {

    public GameObject target;
    public GameObject robotBase;
    public List<GameObject> joints;
    public GameObject robotTip;
    public List<float> initAngles = new List<float> { 25, 15, -35, 60 };
    public bool stopFlag = false;
    public bool initTargetPositionReset = false;

    private Robot robot;
    private Controller ctrler;
    private bool ctrlFlag = false;

    void Awake() {
    }
    // Use this for initialization
    void Start() {
        this.robot = new Robot(robotBase, joints, robotTip);
        this.ctrler = new Controller(this.robot.linkLenghes);
        this.robot.setJointAngles(initAngles);

        var rightHandedEEpos = Controller.toVector3(this.ctrler.getEEPosVec(DenseVector.OfArray(this.robot.getAnglesRadian().ToArray())));
        if (initTargetPositionReset) {
            this.target.transform.position = this.transform.TransformPoint(right2leftHand(rightHandedEEpos));
        }
    }

    // Update is called once per frame
    void Update() {

        var targetPos = left2rightHand(this.transform.InverseTransformPoint(this.target.transform.position));
        var angles = DenseVector.OfArray(this.robot.getAnglesRadian().ToArray());

        var dist = this.ctrler.getDistance(angles, targetPos);
        print(dist);
        this.ctrlFlag = dist > 0.1f;

        for (int i = 1; (i < 1000) && this.ctrlFlag && !this.stopFlag; i++) {
            angles = angles - this.ctrler.getInversedJacobi(angles) * (this.ctrler.getEEPosVec(angles) - Controller.toDenseVec(targetPos));
            if (this.ctrler.isClose(angles, targetPos)) {
                this.robot.setJointAnglesRadian((float)angles[0], (float)angles[1], (float)angles[2], (float)angles[3]);
                this.ctrlFlag = false;
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

}












