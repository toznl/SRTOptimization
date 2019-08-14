using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;


using Microsoft.Kinect;
    

namespace SRTOptimization
{
    class KinectCall
    {
        private KinectSensor kinectSensor = null;
        private ColorFrameReader colorFrameReader;
        private FrameDescription colorFrameDescription;
        private uint bytesPerPixel = 0;
        private WriteableBitmap colorBitmap = null;
        private byte[] colorPixels = null;
        struct Kinect_Value
        {
            public double head_X;
            public double head_Y;
            public double head_Z;

            public double neck_X;
            public double neck_Y;
            public double neck_Z;

            public double leftShoulder_X;
            public double leftShoulder_Y;
            public double leftShoulder_Z;

            public double leftElbow_X;
            public double leftElbow_Y;
            public double leftElbow_Z;

            public double leftWrist_X;
            public double leftWrist_Y;
            public double leftWrist_Z;

            public double rightShoulder_X;
            public double rightShoulder_Y;
            public double rightShoulder_Z;

            public double rightElbow_X;
            public double rightElbow_Y;
            public double rightElbow_Z;

            public double rightWrist_X;
            public double rightWrist_Y;
            public double rightWrist_Z;

            public double spineMid_X;
            public double spineMid_Y;
            public double spineMid_Z;

            public double spineBase_X;
            public double spineBase_Y;
            public double spineBase_Z;

            public Kinect_Value Kinect_Input(Body body)
            {
                Kinect_Value result;

                result.head_X = body.Joints[JointType.Head].Position.X;
                result.head_Y = body.Joints[JointType.Head].Position.Y;
                result.head_Z = body.Joints[JointType.Head].Position.Z;

                result.neck_X = body.Joints[JointType.Neck].Position.X;
                result.neck_Y = body.Joints[JointType.Neck].Position.Y;
                result.neck_Z = body.Joints[JointType.Neck].Position.Z;

                result.leftShoulder_X = body.Joints[JointType.ShoulderLeft].Position.X;
                result.leftShoulder_Y = body.Joints[JointType.ShoulderLeft].Position.Y;
                result.leftShoulder_Z = body.Joints[JointType.ShoulderLeft].Position.Z;

                result.leftElbow_X = body.Joints[JointType.ElbowLeft].Position.X;
                result.leftElbow_Y = body.Joints[JointType.ElbowLeft].Position.Y;
                result.leftElbow_Z = body.Joints[JointType.ElbowLeft].Position.Z;

                result.leftWrist_X = body.Joints[JointType.ElbowLeft].Position.X;
                result.leftWrist_Y = body.Joints[JointType.ElbowLeft].Position.Y;
                result.leftWrist_Z = body.Joints[JointType.ElbowLeft].Position.Z;

                result.rightShoulder_X = body.Joints[JointType.ShoulderRight].Position.X;
                result.rightShoulder_Y = body.Joints[JointType.ShoulderRight].Position.Y;
                result.rightShoulder_Z = body.Joints[JointType.ShoulderRight].Position.Z;

                result.rightElbow_X = body.Joints[JointType.ElbowRight].Position.X;
                result.rightElbow_Y = body.Joints[JointType.ElbowRight].Position.Y;
                result.rightElbow_Z = body.Joints[JointType.ElbowRight].Position.Z;

                result.rightWrist_X = body.Joints[JointType.WristRight].Position.X;
                result.rightWrist_Y = body.Joints[JointType.WristRight].Position.Y;
                result.rightWrist_Z = body.Joints[JointType.WristRight].Position.Z;

                result.spineMid_X = body.Joints[JointType.SpineMid].Position.X;
                result.spineMid_Y = body.Joints[JointType.SpineMid].Position.Y;
                result.spineMid_Z = body.Joints[JointType.SpineMid].Position.Z;

                result.spineBase_X = body.Joints[JointType.SpineBase].Position.X;
                result.spineBase_Y = body.Joints[JointType.SpineBase].Position.Y;
                result.spineBase_Z = body.Joints[JointType.SpineBase].Position.Z;

                return result;
            }
        }
        public KinectSensor kinectCall(KinectSensor kinectSensor01)
        {
            kinectSensor01 = kinectSensor;

            return kinectSensor01;
        }
        public SRT_Matrix(Kinect_Value kinect_Value)
        {

        }


    }
}
