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

namespace SRTOptimization.Kinect_Device
{

    public class Kinect_Point
    {
        public DepthSpacePoint head;
        public DepthSpacePoint neck;
        public DepthSpacePoint shoulder_Left;
        public DepthSpacePoint elbow_Left;
        public DepthSpacePoint wrist_Left;
        public DepthSpacePoint shoulder_Right;
        public DepthSpacePoint elbow_Right;
        public DepthSpacePoint wrist_Right;
        public DepthSpacePoint spine_Mid;
        public DepthSpacePoint spine_Base;

        public Matrix<double> body_X;
        public Matrix<double> body_Y;
        public Matrix<double> body_Z;
        public Kinect_Point()
        {
           
        }

    }
    public class Kinect_Mat_X : Kinect_Point
    {
        public Kinect_Mat_X()
        {
            Console.WriteLine("Kinect_Point");
        }
        public Matrix<double> Get_Bodies(Body body, KinectSensor sensor)
        {
            var joints = body.Joints;
            head = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Head].Position);
            neck = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Neck].Position);
            shoulder_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderLeft].Position);
            elbow_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowLeft].Position);
            wrist_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristLeft].Position);
            shoulder_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderRight].Position);
            elbow_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowRight].Position);
            wrist_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristRight].Position);
            spine_Mid = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineMid].Position);
            spine_Base = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineBase].Position);

            body_X = DenseMatrix.OfArray(new double[,]
            {
                {head.X, neck.X, spine_Mid.X, spine_Base.X},
                {shoulder_Left.X, elbow_Left.X, wrist_Left.X,1},
                {shoulder_Right.X, elbow_Right.X, wrist_Right.X,1},
                {1,2,3,4}
            });
            
            return body_X;
            
        }

    }

    public class Kinect_Mat_Y : Kinect_Point
    {
        public Matrix<double> Get_Bodies(Body body, KinectSensor sensor)
        {
            var joints = body.Joints;
            head = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Head].Position);
            neck = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Neck].Position);
            shoulder_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderLeft].Position);
            elbow_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowLeft].Position);
            wrist_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristLeft].Position);
            shoulder_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderRight].Position);
            elbow_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowRight].Position);
            wrist_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristRight].Position);
            spine_Mid = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineMid].Position);
            spine_Base = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineBase].Position);

            body_Y = DenseMatrix.OfArray(new double[,]
            {
                {head.Y, neck.Y, spine_Mid.Y, spine_Base.Y},
                {shoulder_Left.Y, elbow_Left.Y, wrist_Left.Y,1},
                {shoulder_Right.Y, elbow_Right.Y, wrist_Right.Y,1},
                {1,1,1,1}
            });

            return body_Y;
        }

    }

    public class Kinect_Mat_Z : Kinect_Point
    {
        public Matrix<double> Get_Bodies(Body body, KinectSensor sensor)
        {
            var joints = body.Joints;
            head = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Head].Position);
            neck = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.Neck].Position);
            shoulder_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderLeft].Position);
            elbow_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowLeft].Position);
            wrist_Left = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[ JointType.WristLeft].Position);
            shoulder_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ShoulderRight].Position);
            elbow_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.ElbowRight].Position);
            wrist_Right = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.WristRight].Position);
            spine_Mid = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineMid].Position);
            spine_Base = sensor.CoordinateMapper.MapCameraPointToDepthSpace(joints[JointType.SpineBase].Position);

            body_Z = DenseMatrix.OfArray(new double[,]
            {
                {body.Joints[JointType.Head].Position.Z, body.Joints[JointType.Neck].Position.Z, body.Joints[JointType.SpineMid].Position.Z, body.Joints[JointType.SpineBase].Position.Z},
                {body.Joints[JointType.ShoulderLeft].Position.Z, body.Joints[JointType.ElbowLeft].Position.Z,body.Joints[JointType.WristLeft].Position.Z, 1},
                {body.Joints[JointType.ShoulderRight].Position.Z, body.Joints[JointType.ElbowLeft].Position.Z,body.Joints[JointType.WristRight].Position.Z, 1},
                {1,1,1,1}
        });
            return body_Z;
        }

    }
}
