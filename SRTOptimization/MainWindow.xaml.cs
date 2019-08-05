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

            Vector<double>[] print = new Vector<double>[,];
                print = test(2, 2);

            System.Diagnostics.Debug.WriteLine("Debug WriteLine");
            Console.WriteLine(print);
            System.Diagnostics.Trace.WriteLine("Trace WrtieLine");
        }

        public Vector<double>[] test(double t1, double t2)
        {
            Vector<double>[] result;
            Matrix<double> A = DenseMatrix.OfArray(new double[,] {
                {1,1,1,1 },
                {1,2,3,4 },
                {4,3,2,1 }});

            Vector<double>[] nullspace = A.Kernel();

            result = nullspace;

            return result;
        }
    }
}
