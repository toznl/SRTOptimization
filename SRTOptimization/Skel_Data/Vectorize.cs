using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SRTOptimization.Skel_Data
{
    public class Vectorize
    {
        public Vectorize()
        {

        }

        public Matrix<double> AngleTransform_ArmFrontal(Matrix<double> mat_Skel) //Arm Angle 앞뒤 위아래
        {
            Matrix<double> result;

            Matrix<double> vector_SpineMid = DenseMatrix.OfArray(new double[,]{
                { mat_Skel[2,1]},
                { mat_Skel[2,2]}
            });
            Matrix<double> vector_SpineBase = DenseMatrix.OfArray(new double[,]{
                { mat_Skel[3,1]},
                { mat_Skel[3,2]}
            });

            Matrix<double> vector_Left_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Left
            {
                {mat_Skel[4,1]},
                {mat_Skel[4,2]}
            });

            Matrix<double> vector_Left_02 = DenseMatrix.OfArray(new double[,] //Elbow_Left
            {
                {mat_Skel[5,1] },
                {mat_Skel[5,2] }
            });

            Matrix<double> vector_Right_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Right
            {
                {mat_Skel[8,1] },
                {mat_Skel[8,2] }
            });

            Matrix<double> vector_Right_02 = DenseMatrix.OfArray(new double[,] //Elbow_Right
            {
                {mat_Skel[9,1] },
                {mat_Skel[9,2] }
            });

            Matrix<double> spine = vector_SpineMid - vector_SpineBase;

            Matrix<double> left_Arm = vector_Left_01 - vector_Left_02;
            left_Arm[0, 0] = left_Arm[0, 0] + (left_Arm[0, 0] - spine[0, 0]);
            left_Arm[1, 0] = left_Arm[1, 0] + (left_Arm[1, 0] - spine[1, 0]);

            Matrix<double> right_Arm = vector_Right_01 - vector_Right_02;
            right_Arm[0, 0] = right_Arm[0, 0] + (right_Arm[0, 0] - spine[0, 0]);
            right_Arm[1, 0] = right_Arm[1, 0] + (right_Arm[1, 0] - spine[1, 0]);

            return result = DenseMatrix.OfArray(new double[,]{
            {( Math.Acos(((spine[0,0] * left_Arm[0,0]) + (spine[1,0] * left_Arm[1,0]))/ (VectorSize2D(spine) * VectorSize2D(left_Arm))))*180/Math.PI},
            {(Math.Acos(((spine[0,0] * right_Arm[0,0]) + (spine[1,0]*right_Arm[1,0])) / (VectorSize2D(spine) * VectorSize2D(right_Arm))))*180/Math.PI}
            });
        }

        public Matrix<double> AngleTransform_ArmSide(Matrix<double> mat_Skel) //Arm Angle 양옆 위아래
        {
            Matrix<double> result;

            Matrix<double> vector_SpineMid = DenseMatrix.OfArray(new double[,]{
                { mat_Skel[2,0]},
                { mat_Skel[2,1]}
            });
            Matrix<double> vector_SpineBase = DenseMatrix.OfArray(new double[,]{
                { mat_Skel[3,0]},
                { mat_Skel[3,1]}
            });

            Matrix<double> vector_Left_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Left
            {
                {mat_Skel[4,0]},
                {mat_Skel[4,1]}
            });

            Matrix<double> vector_Left_02 = DenseMatrix.OfArray(new double[,] //Elbow_Left
            {
                {mat_Skel[5,0] },
                {mat_Skel[5,1] }
            });

            Matrix<double> vector_Right_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Right
            {
                {mat_Skel[8,0] },
                {mat_Skel[8,1] }
            });

            Matrix<double> vector_Right_02 = DenseMatrix.OfArray(new double[,] //Elbow_Right
            {
                {mat_Skel[9,0] },
                {mat_Skel[9,1] }
            });

            Matrix<double> spine = vector_SpineMid - vector_SpineBase;

            Matrix<double> left_Arm = vector_Left_01 - vector_Left_02;
            left_Arm[0, 0] = left_Arm[0, 0] + (left_Arm[0, 0] - spine[0, 0]);
            left_Arm[1, 0] = left_Arm[1, 0] + (left_Arm[1, 0] - spine[1, 0]);

            Matrix<double> right_Arm = vector_Right_01 - vector_Right_02;
            right_Arm[0, 0] = right_Arm[0, 0] + (right_Arm[0, 0] - spine[0, 0]);
            right_Arm[1, 0] = right_Arm[1, 0] + (right_Arm[1, 0] - spine[1, 0]);

            return result = DenseMatrix.OfArray(new double[,]{
            {( Math.Acos(((spine[0,0] * left_Arm[0,0]) + (spine[1,0] * left_Arm[1,0]))/ (VectorSize2D(spine) * VectorSize2D(left_Arm))))*180/Math.PI-30},
            {(Math.Acos(((spine[0,0] * right_Arm[0,0]) + (spine[1,0]*right_Arm[1,0])) / (VectorSize2D(spine) * VectorSize2D(right_Arm))))*180/Math.PI-30}
            });
        }
        public Matrix<double> AngleTransform_Elbow(Matrix<double> mat_Skel) //Elbow
        {
            Matrix<double> result;

            Matrix<double> vector_Left_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Left
            {
                {mat_Skel[4,0]},
                {mat_Skel[4,1]},
                {mat_Skel[4,2]}
            });

            Matrix<double> vector_Left_02 = DenseMatrix.OfArray(new double[,] //Elbow_Left
            {
                {mat_Skel[5,0] },
                {mat_Skel[5,1] },
                {mat_Skel[5,2] }
            });

            Matrix<double> vector_Left_03 = DenseMatrix.OfArray(new double[,] //Wrist_Left
            {
                {mat_Skel[6,0] },
                {mat_Skel[6,1] },
                {mat_Skel[6,2] }
            });

            Matrix<double> vector_Right_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Right
            {
                {mat_Skel[8,0] },
                {mat_Skel[8,1] },
                {mat_Skel[8,2] }
            });

            Matrix<double> vector_Right_02 = DenseMatrix.OfArray(new double[,] //Elbow_Right
            {
                {mat_Skel[9,0] },
                {mat_Skel[9,1] },
                {mat_Skel[9,2] }
            });

            Matrix<double> vector_Right_03 = DenseMatrix.OfArray(new double[,] //Wrist_Right
            {
                {mat_Skel[10,0] },
                {mat_Skel[10,1] },
                {mat_Skel[10,2] }
            });

            Matrix<double> vector_LeftArm_Up = vector_Left_02 - vector_Left_01;
            Matrix<double> vector_LeftArm_Below = vector_Left_02 - vector_Left_03;

            Matrix<double> vector_RightArm_Up = vector_Right_01 - vector_Right_02;
            Matrix<double> vector_RightArm_Below = vector_Right_02 - vector_Right_03;

            return result = DenseMatrix.OfArray(new double[,]{
            {(180-( Math.Acos(((vector_LeftArm_Up[0,0] * vector_LeftArm_Below[0,0]) + (vector_LeftArm_Up[1,0] * vector_LeftArm_Below[1,0]) + (vector_LeftArm_Up[2,0]*vector_LeftArm_Below[2,0])) / (VectorSize(vector_LeftArm_Up) * VectorSize(vector_LeftArm_Below))))*180/Math.PI)},
            {(180-( Math.Acos(((vector_RightArm_Up[0,0] * vector_RightArm_Below[0,0]) + (vector_RightArm_Up[1,0] * vector_RightArm_Below[1,0]) + (vector_RightArm_Up[2,0]*vector_RightArm_Below[2,0])) / (VectorSize(vector_RightArm_Up) * VectorSize(vector_RightArm_Below))))*180/Math.PI)}
            });
        }

        public Matrix<double> AngleTransform_ArmSpin(Matrix<double> mat_Skel)
        {
            Matrix<double> result;

            Matrix<double> vector_Left_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Left
             {
                {mat_Skel[4,0]},
                {mat_Skel[4,1]},
                {mat_Skel[4,2]}
             });

            Matrix<double> vector_Left_02 = DenseMatrix.OfArray(new double[,] //Elbow_Left
            {
                {mat_Skel[5,0] },
                {mat_Skel[5,1] },
                {mat_Skel[5,2] }
            });

            Matrix<double> vector_Left_03 = DenseMatrix.OfArray(new double[,] //Wrist_Left
            {
                {mat_Skel[6,0] },
                {mat_Skel[6,1] },
                {mat_Skel[6,2] }
            });

            Matrix<double> vector_Left_04 = DenseMatrix.OfArray(new double[,] //Thumb_Left
            {
                {mat_Skel[7,0] },
                {mat_Skel[7,1] },
                {mat_Skel[7,2] }
            });

            Matrix<double> vector_Right_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Right
            {
                {mat_Skel[8,0] },
                {mat_Skel[8,1] },
                {mat_Skel[8,2] }
            });

            Matrix<double> vector_Right_02 = DenseMatrix.OfArray(new double[,] //Elbow_Right
            {
                {mat_Skel[9,0] },
                {mat_Skel[9,1] },
                {mat_Skel[9,2] }
            });

            Matrix<double> vector_Right_03 = DenseMatrix.OfArray(new double[,] //Wrist_Right
            {
                {mat_Skel[10,0] },
                {mat_Skel[10,1] },
                {mat_Skel[10,2] }
            });

            Matrix<double> vector_Right_04 = DenseMatrix.OfArray(new double[,] //Thumb_Right
            {
                {mat_Skel[11,0] },
                {mat_Skel[11,1] },
                {mat_Skel[11,2] }
            });

            Matrix<double> arm_Upper_Left = vector_Left_02-vector_Left_01;
            Matrix<double> arm_Below_Left= vector_Left_03-vector_Left_02;
            
            Matrix<double> arm_Upper_Right = vector_Right_02-vector_Right_01;
            Matrix<double> arm_Below_Right = vector_Right_03-vector_Right_02;

            double Distance_Upper_Left = Math.Abs((arm_Upper_Left[0,0]*vector_Left_03[0,0])+ (arm_Upper_Left[1, 0] * vector_Left_03[1, 0])+ (arm_Upper_Left[2, 0] * vector_Left_03[2, 0])) /(Math.Sqrt(Math.Pow(arm_Upper_Left[0,0],2)+ Math.Pow(arm_Upper_Left[1, 0], 2) + Math.Pow(arm_Upper_Left[2, 0], 2)));
            double Distance_Upper_Right = Math.Abs((arm_Upper_Right[0, 0] * vector_Right_03[0, 0]) + (arm_Upper_Right[1, 0] * vector_Right_03[1, 0]) + (arm_Upper_Right[2, 0] * vector_Right_03[2, 0])) / (Math.Sqrt(Math.Pow(arm_Upper_Right[0, 0], 2) + Math.Pow(arm_Upper_Right[1, 0], 2) + Math.Pow(arm_Upper_Right[2, 0], 2)));
            
            //Upper Left Arm Angle
            Matrix<double> vector_Ortho_Arm_Upper_Left=(vector_Left_03-(Distance_Upper_Left*arm_Upper_Left))-vector_Left_02;
            //Matrix<double> vector_Ortho_Arm_Below_Left;

            //Upper Right Arm Angle
            Matrix<double> vector_Ortho_Arm_Upper_Right= (vector_Right_03 - (Distance_Upper_Right * arm_Upper_Right)) - vector_Right_02;
            //Matrix<double> vector_Ortho_Arm_Below_Right;
            

            return result = DenseMatrix.OfArray(new double[,]{
            {-(Math.Atan2(vector_Left_03[0,0]-vector_Left_01[0,0], vector_Left_03[2,0]-vector_Left_01[0,0])*180/Math.PI) },
            {Math.Atan2(vector_Right_03[0,0]-vector_Right_01[0,0], vector_Right_03[2,0]-vector_Right_01[0,0])*180/Math.PI }
            });
        }

        public double PlaneFunc(double x, double y, double z, Matrix<double> orthoVector, Matrix<double> initDot)
        {
            double result;

            return result = (orthoVector[0,0]*(x - initDot[0, 0]))+ (orthoVector[1, 0] * (y - initDot[1, 0]))+ (orthoVector[2, 0] * (z - initDot[2, 0]));
        }

        public double VectorSize(Matrix<double> Vector)
        {
            double VectorSize;

            VectorSize = Math.Sqrt((Vector[0, 0] * Vector[0, 0]) + (Vector[1, 0] * Vector[1, 0]) + (Vector[2, 0] * Vector[2, 0]));

            return VectorSize;
        }

        public double VectorSize2D(Matrix<double> Vector)
        {
            double VectorSize;

            VectorSize = Math.Sqrt((Vector[0, 0] * Vector[0, 0]) + (Vector[1, 0] * Vector[1, 0]));

            return VectorSize;
        }
    }
}