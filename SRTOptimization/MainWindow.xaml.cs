using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Windows.Threading;

using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;


using Microsoft.Kinect;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


namespace SRTOptimization
{
    /// <summary>
    /// MainWindow.xaml에 대한 상호 작용 논리
    /// </summary>

    
    public partial class MainWindow : Window
    {
        KinectSensor sensor;
        DepthFrameReader depthReader;
        BodyFrameReader bodyReader;
        IList<Body> bodies;

        Matrix<double> mat_x_01;
        Matrix<double> mat_y_01;
        Matrix<double> mat_z_01;

        public MainWindow()
        {
            AllocConsole();
            InitializeComponent();
            this.Loaded += OnLoaded;
            this.Closing += delegate
            {
                Environment.Exit(1);
            };
        }

        private ImageSource ProcessFrame(DepthFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;
            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;
            ushort[] pixelData = new ushort[width * height];
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];
            frame.CopyFrameDataToArray(pixelData);
            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < pixelData.Length; ++depthIndex)
            {
                ushort depth = pixelData[depthIndex];
                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);
                pixels[colorIndex++] = intensity; // Blue
                pixels[colorIndex++] = intensity; // Green
                pixels[colorIndex++] = intensity; // Red
                ++colorIndex;
            }

            int stride = width * format.BitsPerPixel / 8;
            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        //Kinect sensor open & Depthreader, Bodyreader open
        void OnLoaded(object sender, RoutedEventArgs e)
        {
            this.sensor = KinectSensor.GetDefault();
            this.sensor.Open();

            this.depthReader = this.sensor.DepthFrameSource.OpenReader();
            this.depthReader.FrameArrived += OnDepthFrameArrived;

            this.bodyReader = this.sensor.BodyFrameSource.OpenReader();
            this.bodyReader.FrameArrived += OnBodyFrameArrived;

        }

        void CloseReader(object sender, RoutedEventArgs e)
        {
            this.depthReader.FrameArrived -= OnDepthFrameArrived;
            this.depthReader.Dispose();
            this.depthReader = null;

            this.bodyReader.FrameArrived -= OnBodyFrameArrived;
            this.bodyReader.Dispose();
            this.bodyReader = null;
        }
        //release Kinect v2 Sensor
        void ReleaseSensor()
        {
            this.sensor.Close();
            this.sensor = null;
        }

        #region ConsoleWindows
        //For using console windows

        [DllImport("kernel32.dll", SetLastError = true)]
        [return: MarshalAs(UnmanagedType.Bool)]
        static extern bool AllocConsole();
        #endregion

        void OnBodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            
            
            var frame = e.FrameReference.AcquireFrame();

            if (frame != null)
            {
                canvas.Children.Clear();
                bodies = new Body[frame.BodyFrameSource.BodyCount];
                frame.GetAndRefreshBodyData(bodies);

                foreach (var body in bodies)
                {
                    if(bodies!=null)
                    {
                        if (body.IsTracked)
                        {
                            try
                            {
                                Kinect_Device.Kinect_Mat_X _Mat_X_01 = new Kinect_Device.Kinect_Mat_X();
                                Kinect_Device.Kinect_Mat_Y _Mat_Y_01 = new Kinect_Device.Kinect_Mat_Y();
                                Kinect_Device.Kinect_Mat_Z _Mat_Z_01 = new Kinect_Device.Kinect_Mat_Z();

                                _Mat_X_01.Get_Bodies(body, sensor);
                                //_Mat_Y_01.Get_Bodies(body, sensor);
                                //_Mat_Z_01.Get_Bodies(body, sensor);

                                mat_x_01 = _Mat_X_01.body_X;
                                //mat_y_01 = _Mat_Y_01.body_y;
                                //mat_z_01 = _Mat_Z_01.body_z;

                                double x = mat_x_01[0, 1];

                                Console.WriteLine(mat_x_01);
                            }

                            catch(NullReferenceException ex)
                            {
                                Console.WriteLine(ex.ToString());
                            }
                            

                            //Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                            //{
                            //    System.Windows.Shapes.Ellipse drawHead = new System.Windows.Shapes.Ellipse
                            //    {
                            //        Fill = Brushes.Red,
                            //        Width = 20,
                            //        Height = 20
                            //    };

                            //Canvas.SetLeft(drawHead, mat_x_01[0, 0] * 300 - drawHead.Width / 2);
                            //Canvas.SetTop(drawHead, mat_y_01[0, 0] * 300 - drawHead.Width / 2);
                            //    canvas.Children.Add(drawHead);

                            //}));


                        }
                    }
                }
            }
            
           
        }
        void OnDepthFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    depthCamera.Source = ProcessFrame(frame);
                }
            }
        }
    }
}
