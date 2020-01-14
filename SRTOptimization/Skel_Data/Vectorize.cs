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
        public Matrix<double> Arm_Transform_Upper(Matrix<double> mat_Skel)
        {
            Matrix<double> result;

            Matrix<double> shoulder_Mid = DenseMatrix.OfArray(new double[,]
          {
                {mat_Skel[1,0] },
                {mat_Skel[1,1] },
                {mat_Skel[1,2] }
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

            Matrix<double> shoulder_Left_c = shoulder_Left - shoulder_Mid;
            Matrix<double> shoulder_Right_c = shoulder_Right - shoulder_Mid;
            Matrix<double> elbow_Left_c = elbow_Left - shoulder_Mid;
            Matrix<double> elbow_Right_c = elbow_Right - shoulder_Mid;

            double r_Left = Math.Sqrt((elbow_Left_c[2,0]*elbow_Left_c[2,0])+ (elbow_Left_c[1, 0] * elbow_Left_c[1, 0]));
            double r_Right = Math.Sqrt((elbow_Right_c[2, 0] * elbow_Right_c[2, 0]) + (elbow_Right_c[1, 0] * elbow_Right_c[1, 0]));

            return result = DenseMatrix.OfArray(new double[,]
            {
                { Math.Atan2(elbow_Left_c[2,0], elbow_Left_c[1,0])*180/Math.PI },
                { Math.Atan2(elbow_Right_c[2,0], elbow_Right_c[1,0])*180/Math.PI },
                { Math.Atan2(r_Left,elbow_Left_c[0,0]-shoulder_Left_c[0,0])*180/Math.PI+Math.PI/2},
                { Math.Atan2(r_Right,elbow_Right_c[0,0]-shoulder_Right_c[0,0])*180/Math.PI+Math.PI/2}
            });
        }

        public Matrix<double> Arm_Transform_Below(Matrix<double> mat_Skel) //Arm Angle 양옆 위아래
        {
            Matrix<double> result;

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

            Matrix<double> elbow_Left_c = elbow_Left - shoulder_Left;
            Matrix<double> elbow_Right_c = elbow_Right - shoulder_Right;
            Matrix<double> wrist_Left_c = wrist_Left - shoulder_Left;
            Matrix<double> wrist_Right_c = wrist_Right - shoulder_Right;

            double r_Left = Math.Sqrt((wrist_Left_c[2, 0] * wrist_Left_c[2, 0]) + (wrist_Left_c[1, 0] * wrist_Left_c[1, 0]));
            double r_Right = Math.Sqrt((wrist_Right_c[2, 0] * wrist_Right_c[2, 0]) + (wrist_Right_c[1, 0] * wrist_Right_c[1, 0]));

            return result = DenseMatrix.OfArray(new double[,]
            {
                { Math.Atan2(wrist_Left_c[2,0], wrist_Left_c[1,0])*180/Math.PI},
                { Math.Atan2(wrist_Right_c[2,0], wrist_Right_c[1,0])*180/Math.PI},
                { Math.Atan2(r_Left,wrist_Left_c[0,0]-elbow_Left_c[0,0])*180/Math.PI+Math.PI/2},
                { Math.Atan2(r_Right,wrist_Right_c[0,0]-elbow_Right_c[0,0])*180/Math.PI+Math.PI/2}
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