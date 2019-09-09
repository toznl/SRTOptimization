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

        //    public double Vector_Left(Matrix<double> mat_X, Matrix<double> mat_Y, Matrix<double> mat_Z, string which_Joint)
        //    {
        //        Vector<double> result;
        //        if (which_Joint == "Elbow_Right")
        //        {
        //            Vector<double> shoulder_Left = new DenseVector(new[] { mat_X[1, 0], mat_Y[1, 0], mat_Z[1, 0] });
        //            Vector<double> elbow_Left = new DenseVector(new[] { mat_X[1, 1], mat_Y[1, 1], mat_Z[1, 1] });
        //            Vector<double> wrsit_Left = new DenseVector(new[] { mat_X[1, 2], mat_Y[1, 2], mat_Z[1, 2] });

        //            Vector<double> arm_part_01 = elbow_Left- shoulder_Left;
        //            Vector<double> arm_part_02 = wrsit_Left - elbow_Left;



        //            result = Vector.Acos(Vector.op_DotMultiply(arm_part_01, arm_part_02)/Vector.Sqrt(arm_part_01)*Vector.Sqrt(arm_part_02));
        //            return result;
        //        }
        //    }

        //    public double Vector_Right(Matrix<double> mat_X, Matrix<double> mat_Y, Matrix<double> mat_Z, string which_Joint)
        //    {
        //        double result;
        //        if (which_Joint == "Elbow")
        //        {
        //            result =
        //            return result;
        //        }
        //    }
    }
}