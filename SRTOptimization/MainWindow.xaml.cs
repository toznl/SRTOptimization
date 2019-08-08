using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

//Matrix Library [MathNet.Numerics]
//https://numerics.mathdotnet.com/
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;



namespace SRTOptimization
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
                 
            System.Diagnostics.Debug.WriteLine("Debug WriteLine");
            SRT_Matrix mat = new SRT_Matrix();
            mat.Test1(1,1);
            SRT_Matrix_Kinect mat1 = new SRT_Matrix_Kinect();
            mat1.Test1(1, 1);
            System.Diagnostics.Trace.WriteLine("Trace WrtieLine");
        }


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
        }

        class SRT_Matrix_Kinect : SRT_Matrix
        {
            public void whatthe()
            {
                Console.WriteLine(Test1(1,2).ToString());
            }
        }

      
        
    }
}
