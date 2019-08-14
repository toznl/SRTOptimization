using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

//Matrix Library [MathNet.Numerics]
//https://numerics.mathdotnet.com/
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

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

        public DenseMatrix Kinect(double head, double neck, double spine_mid, double spine_base, double shoulder_left, double elbow_left, double wrist_left, double inner_left, double outer_left, double shoulder_right, double elbow_right, double wrist_right, double inner_right, double outer_right)
        {
            DenseMatrix result;
            //5x3Matrix of Kinect_Device
            result = DenseMatrix.OfArray(new double[,]{
                {head, neck, spine_mid, spine_base,0},
                {shoulder_left, elbow_left, wrist_left, inner_left, outer_left},
                {shoulder_right, elbow_right, wrist_right, inner_right, outer_right}
            });

            return result;
        }
    }
}
