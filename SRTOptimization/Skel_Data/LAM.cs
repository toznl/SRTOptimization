using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SRTOptimization.Skel_Data
{
    public class LAM
    {
        public Matrix<double> Patter_01 = DenseMatrix.OfArray(new double[,]
        {
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {20 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 }
        });
        public Matrix<double> Patter_02 = DenseMatrix.OfArray(new double[,]
        {
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {10 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 },
            {0 }
        });

        public Matrix<double> W_Matrix_Pattern01(Matrix<double> Angle_Set)
        {
            Matrix<double> w_matrix;

            w_matrix = Angle_Set * Patter_01.Transpose();

            return w_matrix;
        }
        public Matrix<double> W_Matrix_Pattern02(Matrix<double> Angle_Set)
        {
            Matrix<double> w_matrix;

            w_matrix = Angle_Set * Patter_02.Transpose();

            return w_matrix;
        }
    }
}
