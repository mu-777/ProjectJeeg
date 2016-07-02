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

    public List<float> getAngles() {
        var ret = new List<float>();
        for (int i = 0; i < this.joints.Count; i++) {
            ret.Add((Quaternion.Inverse(this.initRotations[i]) * this.joints[i].transform.localRotation).eulerAngles.y);
        }
        return ret;
    }
    public List<double> getAnglesd() {
        var ret = new List<double>();
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

    public DenseVector getEEPosVec(DenseVector angles) {
        Func<int, float> l = (int i) => {
            return this.linkLenghes[i];
        };
        Func<int, float> th = (int i) => {
            return (float)angles[i - 1] * Mathf.Deg2Rad;
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
        return (DenseMatrix)((jacobi.Transpose() * jacobi).Inverse() * jacobi.Transpose());
    }

    public DenseMatrix getJacobi(DenseVector angles) {
        Func<int, float> l = (int i) => {
            return this.linkLenghes[i];
        };
        Func<int, float> th = (int i) => {
            return (float)angles[i - 1] * Mathf.Deg2Rad;
        };
        Func<int, float> s = (int i) => {
            return Mathf.Sin(th(i));
        };
        Func<int, float> c = (int i) => {
            return Mathf.Cos(th(i));
        };


        var m = new DenseMatrix(3, 4);
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

    public bool isClose(DenseVector angles, Vector3 targetPosition, float ipsilon = 0.00001f) {
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
        this.robot.setJointAngles(new List<float> { 5, 15, 25, 5 });
    }

    // Update is called once per frame
    void Update() {
        if (!this.startButton) {
            return;
        }
        var targetPos = this.target.transform.localPosition;
        var angles = DenseVector.OfArray(this.robot.getAnglesd().ToArray());

        print(this.ctrler.getInversedJacobi(angles) * (this.ctrler.getEEPosVec(angles) - Controller.toDenseVec(targetPos)));

        for (int i = 1; i < 100; i++) {
            angles = angles - this.ctrler.getInversedJacobi(angles) * (this.ctrler.getEEPosVec(angles) - Controller.toDenseVec(targetPos));
            if (this.ctrler.isClose(angles, targetPos)) {
                break;
            }
        }
        //foreach (var ang in angles) {
        //    if (ang == NaN {
        //        return;
        //    }
        //}
        //this.robot.setJointAngles(new List<float> { (float)angles[0], (float)angles[1], (float)angles[2], (float)angles[3] });
        //this.startButton = false;
    }
}












