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

            #region Shoulder_Elbow Matrix
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
            #endregion
            #region Shoulder_Elbow_c
            Matrix<double> shoulder_Left_c = shoulder_Left - shoulder_Mid;
            Matrix<double> shoulder_Right_c = shoulder_Right - shoulder_Mid;
            Matrix<double> elbow_Left_c = elbow_Left - shoulder_Mid;
            Matrix<double> elbow_Right_c = elbow_Right - shoulder_Mid;
            #endregion

            elbow_Left_c[2, 0] = -elbow_Left_c[2, 0];
            elbow_Right_c[2, 0] = -elbow_Right_c[2, 0];
            shoulder_Right_c[2, 0] = -shoulder_Right_c[2, 0];
            shoulder_Left_c[2, 0] = -shoulder_Left_c[2, 0];
            
            shoulder_Left_c[0, 0] = -shoulder_Left_c[0, 0];
            elbow_Left_c[0, 0] = -elbow_Left_c[0, 0];

            double r_Left = Math.Sqrt((elbow_Left_c[2, 0] * elbow_Left_c[2, 0]) + (elbow_Left_c[1, 0] * elbow_Left_c[1, 0]));
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
            #region Elbow&Shoulder Matrix
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

            #endregion
            #region Elbow_c
            Matrix<double> elbow_Left_c = elbow_Left - shoulder_Left;
            Matrix<double> wrist_Left_c = wrist_Left - shoulder_Left;

            Matrix<double> elbow_Right_c = elbow_Right - shoulder_Right;
            Matrix<double> wrist_Right_c = wrist_Right - shoulder_Right;
            #endregion

            Matrix<double> elbow_Left_u = elbow_Left_c / Math.Sqrt((elbow_Left_c[0, 0] * elbow_Left_c[0, 0])+ (elbow_Left_c[1, 0] * elbow_Left_c[1, 0]) + (elbow_Left_c[2, 0] * elbow_Left_c[2, 0]));
            Matrix<double> elbow_Right_u = elbow_Right_c / Math.Sqrt((elbow_Right_c[0, 0] * elbow_Right_c[0, 0]) + (elbow_Right_c[1, 0] * elbow_Right_c[1, 0]) + (elbow_Right_c[2, 0] * elbow_Right_c[2, 0]));
            elbow_Left_c[2, 0] = -elbow_Left_c[2, 0];
            elbow_Right_c[2, 0] = -elbow_Right_c[2, 0];
            wrist_Right_c[2, 0] = -wrist_Right_c[2, 0];
            wrist_Left_c[2, 0] = -wrist_Left_c[2, 0];

            wrist_Left_c[0, 0] = -wrist_Left_c[0, 0];
            elbow_Left_c[0, 0] = -elbow_Left_c[0, 0];
            Matrix<double> vec_Y = DenseMatrix.OfArray(new double[,] {
                {0 },
                {1 },
                {0 }
            });

            double Angle_X_Left = Math.Acos((elbow_Left_u[2, 0] * vec_Y[2, 0] + elbow_Left_u[1, 0] * vec_Y[1, 0])/ Math.Sqrt((elbow_Left_u[1, 0] * elbow_Left_u[1, 0]) + (elbow_Left_u[2, 0] * elbow_Left_u[2, 0])) * Math.Sqrt((vec_Y[1, 0] * vec_Y[1, 0]) + (vec_Y[2, 0] * vec_Y[2, 0])));
            double Angle_X_Right = Math.Acos((elbow_Right_u[2, 0] * vec_Y[2, 0] + elbow_Right_u[1, 0] * vec_Y[1, 0])/ Math.Sqrt((elbow_Right_u[1, 0] * elbow_Right_u[1, 0]) + (elbow_Right_u[2, 0] * elbow_Right_u[2, 0])) * Math.Sqrt((vec_Y[1, 0] * vec_Y[1, 0]) + (vec_Y[2, 0] * vec_Y[2, 0])));
            double Angle_Z_Left = Math.Acos((elbow_Left_u[0, 0] * vec_Y[0, 0] + elbow_Left_u[1, 0] * vec_Y[1, 0])/ Math.Sqrt((elbow_Left_u[0, 0] * elbow_Left_u[0, 0]) + (elbow_Left_u[1, 0] * elbow_Left_u[1, 0])) * Math.Sqrt((vec_Y[0, 0] * vec_Y[0, 0]) + (vec_Y[1, 0] * vec_Y[1, 0])));
            double Angle_Z_Right = Math.Acos((elbow_Right_u[0, 0] * vec_Y[0, 0] + elbow_Right_u[1, 0] * vec_Y[1, 0])/ Math.Sqrt((elbow_Right_u[0, 0] * elbow_Right_u[0, 0]) + (elbow_Right_u[1, 0] * elbow_Right_u[1, 0])) * Math.Sqrt((vec_Y[0, 0] * vec_Y[0, 0]) + (vec_Y[1, 0] * vec_Y[1, 0])));

            Matrix<double> Rotation_X_Left = DenseMatrix.OfArray(new double[,]
            {
                {1,0,0 },
                {0,Math.Cos(Angle_X_Left),-Math.Sin(Angle_X_Left) },
                {0,Math.Sin(Angle_X_Left),Math.Cos(Angle_X_Left) }
            });
            Matrix<double> Rotation_X_Right = DenseMatrix.OfArray(new double[,]
            {
                {1,0,0 },
                {0,Math.Cos(Angle_X_Right),-Math.Sin(Angle_X_Right) },
                {0,Math.Sin(Angle_X_Right),Math.Cos(Angle_X_Right) }
            });

            Matrix<double> Rotation_Z_Left = DenseMatrix.OfArray(new double[,]
            {
                {Math.Cos(Angle_Z_Left ),-Math.Sin(Angle_Z_Left),0 },
                {Math.Sin(Angle_Z_Left ),Math.Cos(Angle_Z_Left),0 },
                {0,0,1 },
            });
            Matrix<double> Rotation_Z_Right = DenseMatrix.OfArray(new double[,]
            {
                {Math.Cos(Angle_Z_Right ),-Math.Sin(Angle_Z_Right),0 },
                {Math.Sin(Angle_Z_Right ),Math.Cos(Angle_Z_Right),0 },
                {0,0,1 },
            });

            elbow_Left_c = Rotation_X_Left *( Rotation_Z_Left * elbow_Left_c);
            elbow_Right_c = Rotation_X_Right * (Rotation_Z_Right * elbow_Right_c);
            wrist_Left_c = Rotation_X_Left * (Rotation_Z_Left * wrist_Left_c);
            wrist_Right_c = Rotation_X_Right *( Rotation_Z_Right * wrist_Right_c);

            Matrix<double> shoulder_to_elbow_Left = shoulder_Left-elbow_Left;
            Matrix<double> shoulder_to_elbow_Right =shoulder_Right-elbow_Right;

            Matrix<double> el_to_wr_Left = wrist_Left - elbow_Left;
            Matrix<double> el_to_wr_Right = wrist_Right - elbow_Right;

            return result = DenseMatrix.OfArray(new double[,]
            {
                { Math.Atan2(wrist_Left_c[2,0], wrist_Left_c[0,0])*180/Math.PI },
                { Math.Atan2(wrist_Right_c[2,0], wrist_Right_c[0,0])*180/Math.PI},

                {180-((Math.Acos(((shoulder_to_elbow_Left[0,0]*el_to_wr_Left[0,0])+(shoulder_to_elbow_Left[1,0]*el_to_wr_Left[1,0])+(shoulder_to_elbow_Left[2,0]*el_to_wr_Left[2,0]))/(VectorSize(shoulder_to_elbow_Left)*VectorSize(el_to_wr_Left))))*180/Math.PI)},
                {180-((Math.Acos(((shoulder_to_elbow_Right[0,0]*el_to_wr_Right[0,0])+(shoulder_to_elbow_Right[1,0]*el_to_wr_Right[1,0])+(shoulder_to_elbow_Right[2,0]*el_to_wr_Right[2,0]))/(VectorSize(shoulder_to_elbow_Right)*VectorSize(el_to_wr_Right))))*180/Math.PI)},

            }); 
        }
        public double VectorSize(Matrix<double> Vector)
        {
            double Vectorsize;

            Vectorsize = Math.Sqrt((Vector[0, 0] * Vector[0, 0])+ (Vector[1, 0] * Vector[1, 0])+ (Vector[2, 0] * Vector[2, 0]));

            return Vectorsize;
        }
    }
}