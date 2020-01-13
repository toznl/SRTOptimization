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

        public Matrix<double> AngleTransform_Arm(Matrix<double> mat_Skel) //Arm Angle 앞뒤 위아래
        {
            Matrix<double> result;

            Matrix<double> shoulder_Mid = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[1,0] },
                {mat_Skel[1,1] },
                {mat_Skel[1,2] }
            });

            Matrix<double> wrist_Left = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[6,0] },
                    {mat_Skel[6,1] },
                    {mat_Skel[6,2] }
                });

            Matrix<double> wrist_Right = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[10,0] },
                    {mat_Skel[10,1] },
                    {mat_Skel[10,2] }
                });

            Matrix<double> shoulder_Left = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[4,0] },
                    {mat_Skel[4,1] },
                    {mat_Skel[4,2] }
                });
            Matrix<double> shoulder_Right = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[8,0] },
                    {mat_Skel[8,1] },
                    {mat_Skel[8,2] }
                });

            Matrix<double> wrist_Left_c = wrist_Left - shoulder_Mid;
            Matrix<double> wrist_Right_c = wrist_Right - shoulder_Mid;
            Matrix<double> shoulder_Left_c = shoulder_Left - shoulder_Mid;
            Matrix<double> shoulder_Right_c = shoulder_Right - shoulder_Mid;

            double shoulder_Left_d = Math.Sqrt(shoulder_Left_c[0,0]*shoulder_Left_c[0,0]+ shoulder_Left_c[1, 0] * shoulder_Left_c[1,0]+ shoulder_Left_c[2, 0] * shoulder_Left_c[2,0]);
            double shoulder_Right_d = Math.Sqrt(shoulder_Right_c[0, 0] * shoulder_Right_c[0, 0] + shoulder_Right_c[1, 0] * shoulder_Right_c[1, 0] + shoulder_Right_c[2, 0] * shoulder_Right_c[2, 0]);


            return result = DenseMatrix.OfArray(new double[,]{
                { Math.Atan2(wrist_Left_c[0,0], wrist_Left_c[1,0])*180/Math.PI },
                { Math.Atan2(wrist_Right_c[0,0], wrist_Right_c[1,0])*180/Math.PI },
                {Math.Atan2(Math.Sqrt(wrist_Left_c[0,0]*wrist_Left_c[0,0]+wrist_Left_c[1,0]+wrist_Left_c[1,0]),wrist_Left_c[2,0]-shoulder_Left_d)*180/Math.PI+Math.PI/2 },
                {Math.Atan2(Math.Sqrt(wrist_Right_c[0,0]*wrist_Right_c[0,0]+wrist_Right_c[1,0]+wrist_Right_c[1,0]),wrist_Right_c[2,0]-shoulder_Right_d)*180/Math.PI+Math.PI/2 }
            });
        }

        public Matrix<double> Arm_Transform_Elbow(Matrix<double> mat_Skel)
        {
            Matrix<double> result;

            Matrix<double> shoulder_Mid = DenseMatrix.OfArray(new double[,]
          {
                {mat_Skel[1,0] },
                {mat_Skel[1,1] },
                {mat_Skel[1,2] }
          });

            Matrix<double> wrist_Left = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[6,0] },
                    {mat_Skel[6,1] },
                    {mat_Skel[6,2] }
                });

            Matrix<double> wrist_Right = DenseMatrix.OfArray(new double[,]
                {
                    {mat_Skel[10,0] },
                    {mat_Skel[10,1] },
                    {mat_Skel[10,2] }
                });

            Matrix<double> elbow_Left = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[5,0] },
                {mat_Skel[5,1] },
                {mat_Skel[5,2] }
            });
            Matrix<double> elbow_Right = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[9,0] },
                {mat_Skel[9,1] },
                {mat_Skel[9,2] }
            });

            Matrix<double> shoulder_Left = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[4,0] },
                {mat_Skel[4,1] },
                {mat_Skel[4,2] }
            });

            Matrix<double> shoulder_Right = DenseMatrix.OfArray(new double[,]
            {
                {mat_Skel[8,0] },
                {mat_Skel[8,1] },
                {mat_Skel[8,2] }
            });

            Matrix<double> wrist_Left_c = wrist_Left - shoulder_Mid;
            Matrix<double> wrist_Right_c = wrist_Right - shoulder_Mid;
            Matrix<double> shoulder_Left_c = shoulder_Left - shoulder_Mid;
            Matrix<double> shoulder_Right_c = shoulder_Right - shoulder_Mid;
            Matrix<double> elbow_Left_c = elbow_Left - shoulder_Mid;
            Matrix<double> elbow_Right_c = elbow_Right - shoulder_Mid;

            double shoulder_distance_Left = Math.Sqrt(shoulder_Left_c[0, 0] * shoulder_Left_c[0, 0] + shoulder_Left_c[1, 0] * shoulder_Left_c[1, 0] + shoulder_Left_c[2, 0] * shoulder_Left_c[2, 0]);
            double shoulder_distance_Right = Math.Sqrt(shoulder_Right_c[0, 0] * shoulder_Right_c[0, 0] + shoulder_Right_c[1, 0] * shoulder_Right_c[1, 0] + shoulder_Right_c[2, 0] * shoulder_Right_c[2, 0]);
            double a2_Left = Math.Sqrt((shoulder_Left_c[0, 0] - elbow_Left_c[0, 0]) * (shoulder_Left_c[0, 0] - elbow_Left_c[0, 0])+ (shoulder_Left_c[1, 0] - elbow_Left_c[1, 0]) * (shoulder_Left_c[1, 0] - elbow_Left_c[1, 0])+ (shoulder_Left_c[2, 0] - elbow_Left_c[2, 0]) * (shoulder_Left_c[2, 0] - elbow_Left_c[2, 0]));
            double a3_Left= Math.Sqrt((wrist_Left_c[0, 0] - elbow_Left_c[0, 0]) * (wrist_Left_c[0, 0] - elbow_Left_c[0, 0]) + (wrist_Left_c[1, 0] - elbow_Left_c[1, 0]) * (wrist_Left_c[1, 0] - elbow_Left_c[1, 0]) + (shoulder_Left_c[2, 0] - elbow_Left_c[2, 0]) * (shoulder_Left_c[2, 0] - elbow_Left_c[2, 0]));
            double a2_Right = Math.Sqrt((shoulder_Right_c[0, 0] - elbow_Right_c[0, 0]) * (shoulder_Right_c[0, 0] - elbow_Right_c[0, 0]) + (shoulder_Right_c[1, 0] - elbow_Right_c[1, 0]) * (shoulder_Right_c[1, 0] - elbow_Right_c[1, 0]) + (shoulder_Right_c[2, 0] - elbow_Right_c[2, 0]) * (shoulder_Right_c[2, 0] - elbow_Right_c[2, 0]));
            double a3_Right = Math.Sqrt((wrist_Right_c[0, 0] - elbow_Right_c[0, 0]) * (wrist_Right_c[0, 0] - elbow_Right_c[0, 0]) + (wrist_Right_c[1, 0] - elbow_Right_c[1, 0]) * (wrist_Right_c[1, 0] - elbow_Right_c[1, 0]) + (shoulder_Right_c[2, 0] - elbow_Right_c[2, 0]) * (shoulder_Right_c[2, 0] - elbow_Right_c[2, 0]));


            double big_D_Left = (wrist_Left_c[0, 0] * wrist_Left_c[0, 0] + wrist_Left_c[1, 0] * wrist_Left_c[1, 0] + (wrist_Left[2, 0] - shoulder_distance_Left) * (wrist_Left[2, 0]) - a2_Left * a2_Left - a3_Left * a3_Left)/2*a2_Left*a3_Left;
            double big_D_Right = (wrist_Right_c[0, 0] * wrist_Right_c[0, 0] + wrist_Right_c[1, 0] * wrist_Right_c[1, 0] + (wrist_Right[2, 0] - shoulder_distance_Right) * (wrist_Right[2, 0]) - a2_Right * a2_Right - a3_Right * a3_Right)/2*a2_Right*a3_Right;

            Console.WriteLine(big_D_Left);

            return result = DenseMatrix.OfArray(new double[,]
            {
                { Math.Atan2(big_D_Left,Math.Sqrt(1-(big_D_Left*big_D_Left))) },
                { Math.Atan2(big_D_Right,Math.Sqrt(1-(big_D_Right*big_D_Right))) }
            });
        }

        //public Matrix<double> AngleTransform_ArmSide(Matrix<double> mat_Skel) //Arm Angle 양옆 위아래
        //{
        //    Matrix<double> result;

        //    Matrix<double> vector_SpineMid = DenseMatrix.OfArray(new double[,]{
        //        { mat_Skel[2,0]},
        //        { mat_Skel[2,1]}
        //    });
        //    Matrix<double> vector_SpineBase = DenseMatrix.OfArray(new double[,]{
        //        { mat_Skel[3,0]},
        //        { mat_Skel[3,1]}
        //    });

        //    Matrix<double> vector_Left_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Left
        //    {
        //        {mat_Skel[4,0]},
        //        {mat_Skel[4,1]}
        //    });

        //    Matrix<double> vector_Left_02 = DenseMatrix.OfArray(new double[,] //Elbow_Left
        //    {
        //        {mat_Skel[5,0] },
        //        {mat_Skel[5,1] }
        //    });

        //    Matrix<double> vector_Right_01 = DenseMatrix.OfArray(new double[,] //Shoulder_Right
        //    {
        //        {mat_Skel[8,0] },
        //        {mat_Skel[8,1] }
        //    });

        //    Matrix<double> vector_Right_02 = DenseMatrix.OfArray(new double[,] //Elbow_Right
        //    {;
        //        {mat_Skel[9,0] },
        //        {mat_Skel[9,1] }
        //    });

        //    Matrix<double> spine = vector_SpineMid - vector_SpineBase;

        //    Matrix<double> left_Arm = vector_Left_01 - vector_Left_02;
        //    left_Arm[0, 0] = left_Arm[0, 0] + (left_Arm[0, 0] - spine[0, 0]);
        //    left_Arm[1, 0] = left_Arm[1, 0] + (left_Arm[1, 0] - spine[1, 0]);

        //    Matrix<double> right_Arm = vector_Right_01 - vector_Right_02;
        //    right_Arm[0, 0] = right_Arm[0, 0] + (right_Arm[0, 0] - spine[0, 0]);
        //    right_Arm[1, 0] = right_Arm[1, 0] + (right_Arm[1, 0] - spine[1, 0]);

        //    return result = DenseMatrix.OfArray(new double[,]{
        //    {( Math.Acos(((spine[0,0] * left_Arm[0,0]) + (spine[1,0] * left_Arm[1,0]))/ (VectorSize2D(spine) * VectorSize2D(left_Arm))))*180/Math.PI-30},
        //    {(Math.Acos(((spine[0,0] * right_Arm[0,0]) + (spine[1,0]*right_Arm[1,0])) / (VectorSize2D(spine) * VectorSize2D(right_Arm))))*180/Math.PI-30}
        //    });
        //}
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

            Matrix<double> vector_RightArm_Up = vector_Right_02 - vector_Right_01;
            Matrix<double> vector_RightArm_Below = vector_Right_02 - vector_Right_03;

            return result = DenseMatrix.OfArray(new double[,]{
            {((Math.Acos(((vector_LeftArm_Up[0,0] * vector_LeftArm_Below[0,0]) + (vector_LeftArm_Up[1,0] * vector_LeftArm_Below[1,0]) + (vector_LeftArm_Up[2,0]*vector_LeftArm_Below[2,0])) / (VectorSize(vector_LeftArm_Up) * VectorSize(vector_LeftArm_Below))))*180/Math.PI)},
            {((Math.Acos(((vector_RightArm_Up[0,0] * vector_RightArm_Below[0,0]) + (vector_RightArm_Up[1,0] * vector_RightArm_Below[1,0]) + (vector_RightArm_Up[2,0]*vector_RightArm_Below[2,0])) / (VectorSize(vector_RightArm_Up) * VectorSize(vector_RightArm_Below))))*180/Math.PI)}
            });
        }

        public Matrix<double> AngleTransform_ArmSpin(Matrix<double> mat_Skel)
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

            Matrix<double> arm_Upper_Left = vector_Left_02 - vector_Left_01;
            Matrix<double> arm_Below_Left = vector_Left_03 - vector_Left_02;

            Matrix<double> arm_Upper_Right = vector_Right_02 - vector_Right_01;
            Matrix<double> arm_Below_Right = vector_Right_03 - vector_Right_02;

            return result = DenseMatrix.OfArray(new double[,]{
            {(Math.Atan2(vector_Left_03[0,0]-vector_Left_01[0,0], vector_Left_03[2,0]-vector_Left_01[2,0]))*180/Math.PI},
            {(Math.Atan2(vector_Right_03[0,0]-vector_Right_01[0,0], vector_Right_03[2,0]-vector_Right_01[2,0])*180/Math.PI)},

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