using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;

namespace SRTOptimization.Skel_Data
{
    public class Calibration
    {
    //    public Matrix<double> calibrateTwoPoints(Matrix<double> Device_01, Matrix<double> Device_02)
    //    {
    //        Matrix<double> combinedMatrix;

    //        return combinedMatrix;
    //    }

    //    public Matrix<double> gradientDescnet(Matrix<double> theta_Set)
    //    {
    //        Matrix<double> refinedThetaset;

    //        return refinedThetaset;
    //    }

        //public Matrix<double> Rotation(Matrix<double> SkeletonMatrix, Matrix<double> theta_Set)
        //{
        //    Matrix<double> rotatedMatrix;
        //    Matrix<double> rotateX = DenseMatrix.OfArray(new double[,]{
        //        {1,0,0 },
        //        {0,Math.Cos(theta_Set[0,0]), -Math.Sin(theta_Set[0,0]) },
        //        {0,Math.Sin(theta_Set[0,0]), Math.Cos(theta_Set[0,0])}
        //    });

        //    Matrix<double> rotateY= DenseMatrix.OfArray(new double[,]{
        //        {Math.Cos(theta_Set[0,1]),0,Math.Sin(theta_Set[0,1])},
        //        {0,1,0 },
        //        {-Math.Sin(theta_Set[0,1]),0,Math.Cos(theta_Set[0,1])}
        //    });

        //    Matrix<double> rotateZ= DenseMatrix.OfArray(new double[,]{
        //        {Math.Cos(theta_Set[0,2]),-Math.Sin(theta_Set[0,2]),0},
        //        {Math.Sin(theta_Set[0,2]),Math.Cos(theta_Set[0,2]),0},
        //        {0,0,1 }
        //    });

        //    Matrix<double> head = DenseMatrix.OfArray(new double[,] {
        //        {SkeletonMatrix[0,0], SkeletonMatrix[] }
        //    });

        //    return rotatedMatrix;
        
        //}

        //public Matrix<double> Translation(Matrix<double> SkeletonMatrix, Matrix<double> trans_Set)
        //{
        //    Matrix<double> translatedMatrix;
        //    Matrix<double> trans_Set_Expansion = DenseMatrix.OfArray(new double[,]{
        //        { trans_Set[0,0], trans_Set[0,1], trans_Set[0,2]},
        //        { trans_Set[0,0], trans_Set[0,1], trans_Set[0,2]},
        //        { trans_Set[0,0], trans_Set[0,1], trans_Set[0,2]},
        //    });

        //    translatedMatrix = SkeletonMatrix + trans_Set_Expansion;

        //    return translatedMatrix;
        //}

    }
}
