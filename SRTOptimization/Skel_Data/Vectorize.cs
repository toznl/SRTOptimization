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

            Matrix<double> Vector_Neck = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[1,0] },
                {mat_Skel[1,1] },
                {mat_Skel[1,2] },
            });

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

            return result = DenseMatrix.OfArray(new double[,]{
           {(-(Math.Atan2(vector_Left_02[0,0]-Vector_Neck[0,0], vector_Left_02[2,0]-Vector_Neck[2,0])*360/Math.PI)-270)},
            {(Math.Atan2(vector_Right_02[0,0]-Vector_Neck[0,0], vector_Right_02[2,0]-Vector_Neck[2,0])*360/Math.PI-250)}
            });
        }
        public Matrix<double> AngleTransform_ArmSide(Matrix<double> mat_Skel) //Arm Angle 양옆 위아래
        {
            Matrix<double> result;

            Matrix<double> vector_SpineShoulder = DenseMatrix.OfArray(new double[,]{
                { mat_Skel[1,0]},
                { mat_Skel[1,1]}
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

            Matrix<double> left_Arm_Mid = vector_SpineShoulder - vector_Left_01;
            Matrix<double> left_Arm_Below = vector_Left_02 - vector_Left_01;

            Matrix<double> right_Arm_Mid = vector_SpineShoulder - vector_Right_01;
            Matrix<double> right_Arm_Below = vector_Right_02 - vector_Right_01;

            return result = DenseMatrix.OfArray(new double[,]{
            {((Math.Acos(((left_Arm_Mid[0,0] * left_Arm_Below[0,0]) + (left_Arm_Mid[1,0] * left_Arm_Below[1,0])) / (VectorSize2D(left_Arm_Mid) * VectorSize2D(left_Arm_Below))))*360/Math.PI)-270},
            {((Math.Acos(((right_Arm_Mid[0,0] * right_Arm_Below[0,0]) + (right_Arm_Mid[1,0] * right_Arm_Below[1,0])) / (VectorSize2D(right_Arm_Mid) * VectorSize2D(right_Arm_Below))))*360/Math.PI)-270}
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

            Matrix<double> vector_LeftArm_Up = vector_Left_01 - vector_Left_02;
            Matrix<double> vector_LeftArm_Below = vector_Left_03 - vector_Left_02;

            Matrix<double> vector_RightArm_Up = vector_Right_01 - vector_Right_02;
            Matrix<double> vector_RightArm_Below = vector_Right_03 - vector_Right_02;

            return result = DenseMatrix.OfArray(new double[,]{
            {180-((Math.Acos(((vector_LeftArm_Up[0,0] * vector_LeftArm_Below[0,0]) + (vector_LeftArm_Up[1,0] * vector_LeftArm_Below[1,0]) + (vector_LeftArm_Up[2,0]*vector_LeftArm_Below[2,0])) / (VectorSize(vector_LeftArm_Up) * VectorSize(vector_LeftArm_Below))))*180/Math.PI)},
            {180-((Math.Acos(((vector_RightArm_Up[0,0] * vector_RightArm_Below[0,0]) + (vector_RightArm_Up[1,0] * vector_RightArm_Below[1,0]) + (vector_RightArm_Up[2,0]*vector_RightArm_Below[2,0])) / (VectorSize(vector_RightArm_Up) * VectorSize(vector_RightArm_Below))))*180/Math.PI)}
            });
        }
        public Matrix<double> AngleTransform_ArmSpin(Matrix<double> mat_Skel)//Arm Angle 회전
        {
            Matrix<double> result;

            Matrix<double> Vector_Neck = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[1,0] },
                {mat_Skel[1,1] },
                {mat_Skel[1,2] },
            });

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

            return result = DenseMatrix.OfArray(new double[,]{
            {(Math.Atan2(vector_Left_03[0,0]-vector_Left_01[0,0], vector_Left_03[2,0]-vector_Left_01[2,0]))*180/Math.PI},
            {145-(Math.Atan2(vector_Right_03[0,0]-vector_Right_01[0,0], vector_Right_03[2,0]-vector_Right_01[2,0])*180/Math.PI)}
            });
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