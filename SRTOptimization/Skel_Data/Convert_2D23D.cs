﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SRTOptimization.Skel_Data
{
    public class Convert_2D23D
    {
        static readonly double fx = 366.435;
        static readonly double fy = 366.435;
        static readonly double cx = 259.478;
        static readonly double cy = 203.774;

        public Convert_2D23D()
        {

        }

        public Matrix<double> ConvertX(Matrix<double> mat_X, Matrix<double> mat_Z)
        {
            Matrix<double> result;

            result = (mat_X * fx)+(cx*mat_Z);

            for(int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++){
                    mat_X[i,j]=mat_X[i, j] / mat_Z[i, j];
                }
            }

            result = mat_X;

            return result;
        }

        public Matrix<double> ConvertY(Matrix<double> mat_Y, Matrix<double> mat_Z)
        {
            Matrix<double> result;

            result = ((mat_Y * fy) + (cy * mat_Z));
            try
            {
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        mat_Y[i, j] = mat_Y[i, j]/ mat_Z[i, j];
                    }
                }

                result = mat_Y;

                return result;
            }

            catch(Exception e)
            {
                
                Console.WriteLine(e.ToString());
                return result;
            }
            
        }
    }
}
