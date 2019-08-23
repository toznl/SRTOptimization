using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

//Matrix Library [MathNet.Numerics]
//https://numerics.mathdotnet.com/
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using Microsoft.Kinect;

namespace SRTOptimization
{
    class SRT_Matrix
    {
        public Vector<double>[] Test1(int t1, int t2)
        {
            Vector<double>[] result;
            Matrix<double> A = DenseMatrix.OfArray(new double[,] {
                {1,1,1,1 },
                {1,2,3,4 },
                {4,3,2,1 }});

            Vector<double>[] nullspace = A.Kernel();


            Console.WriteLine(A.ToString());
            result = nullspace;

            return result;


        }


    }
    public class Get_Kinect_Point
    {
        public virtual void Get_Bodies(Body body, KinectSensor sensor)
        {
            Matrix<double> X_Points_Kinect;
            var joints = body.Joints;
            DepthSpacePoint head= sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Head].Position);
            DepthSpacePoint neck = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Neck].Position);
            DepthSpacePoint shoulder_Left= sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderLeft].Position);
            DepthSpacePoint elbow_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowLeft].Position);
            DepthSpacePoint wrist_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristLeft].Position);
            DepthSpacePoint shoulder_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderRight].Position);
            DepthSpacePoint elbow_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowRight].Position);
            DepthSpacePoint wrist_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristRight].Position);
            DepthSpacePoint spine_Mid = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineMid].Position);
            DepthSpacePoint spine_Base = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineBase].Position);

            X_Points_Kinect = DenseMatrix.OfArray(new double[,]
            {
                {head.X,          neck.X, 0, 0},
                {shoulder_Left.X, elbow_Left.X, wrist_Left.X,0 },
                {shoulder_Right.X, elbow_Right.X, wrist_Right.X,0 },
                {spine_Mid.X, spine_Base.X, 0,0 },
                {0,0,0,0 }
            });


        }

        public override Matrix<double> Y_Matrix(Body body, KinectSensor sensor) : X_Matrix{


    }

    public class Set_Kinect_Point : Get_Kinect_Point
    {
        public override void Get_Bodies(Body body, KinectSensor sensor)
        {
            base.Get_Bodies(body, sensor);
        }
    }


}
