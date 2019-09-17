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

        public double AngleTransform_01(Matrix<double> mat_X, Matrix<double> mat_Y, Matrix<double> mat_Z)
        {
            double result;



            result = 0.0;

            return result;
        }

        public double AngleTransform_02(Matrix<double> mat_X, Matrix<double> mat_Y, Matrix<double> mat_Z)
        {
            double result;

            result = mat_X[0,0];

            return result;
        }


    }
}