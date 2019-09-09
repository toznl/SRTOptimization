using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Threading;
using System.Runtime.InteropServices;
using System.Windows.Controls;
using System.Windows.Shapes;

namespace SRTOptimization.Kinect_Device
{
    class DrawSkeleton
    {
        private string _name;
        public void DrawPoints(string PointName, String fill, double width, double height, String setLeft, String SetTop)
        {


            //Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
            //{
            //    try
            //    {
            //        _name = PointName;
            //        Ellipse PointName = new Ellipse
            //        {
            //            Fill = Brushes.Orange,
            //            Width = 20,
            //            Height = 20
            //        };

            //        Canvas.SetLeft(drawSpineBase, Convert.ToDouble(setLeft) - drawSpineBase.Width / 2);
            //        Canvas.SetTop(drawSpineBase, mat_Y_01[0, 3] - drawSpineBase.Width / 2);
            //        canvas.Children.Add(drawSpineBase);
            //    }
            //    catch (Exception e)
            //    {
            //        Console.WriteLine(e.ToString());
            //    }
            //}));
        }

    }

}

