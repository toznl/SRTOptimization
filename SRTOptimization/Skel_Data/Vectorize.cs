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

            result = DenseMatrix.OfArray(new double[,]{
            {( Math.Acos(((spine[0,0] * left_Arm[0,0]) + (spine[1,0] * left_Arm[1,0]))/ (VectorSize2D(spine) * VectorSize2D(left_Arm))))*180/Math.PI},
            {(Math.Acos(((spine[0,0] * right_Arm[0,0]) + (spine[1,0]*right_Arm[1,0])) / (VectorSize2D(spine) * VectorSize2D(right_Arm))))*180/Math.PI}
            });

            return result;
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

            result = DenseMatrix.OfArray(new double[,]{
            {( Math.Acos(((spine[0,0] * left_Arm[0,0]) + (spine[1,0] * left_Arm[1,0]))/ (VectorSize2D(spine) * VectorSize2D(left_Arm))))*180/Math.PI},
            {(Math.Acos(((spine[0,0] * right_Arm[0,0]) + (spine[1,0]*right_Arm[1,0])) / (VectorSize2D(spine) * VectorSize2D(right_Arm))))*180/Math.PI}
            });

            return result;
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

            result = DenseMatrix.OfArray(new double[,]{
            {(180-( Math.Acos(((vector_LeftArm_Up[0,0] * vector_LeftArm_Below[0,0]) + (vector_LeftArm_Up[1,0] * vector_LeftArm_Below[1,0]) + (vector_LeftArm_Up[2,0]*vector_LeftArm_Below[2,0])) / (VectorSize(vector_LeftArm_Up) * VectorSize(vector_LeftArm_Below))))*180/Math.PI)},
            {(180-(Math.Acos(((vector_RightArm_Up[0,0] * vector_RightArm_Below[0,0]) + (vector_RightArm_Up[1,0] * vector_RightArm_Below[1,0]) + (vector_RightArm_Up[2,0]*vector_RightArm_Below[2,0])) / (VectorSize(vector_RightArm_Up) * VectorSize(vector_RightArm_Below))))*180/Math.PI)}
            });

            return result;
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