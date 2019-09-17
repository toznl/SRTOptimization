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

        public string angle01 = "0";
        public string angle02 = "0";
        public string angle03 = "0";
        public string angle04 = "0";
        public string angle05 = "0";
        public string angle06 = "0";
        public string angle07 = "0";
        public string angle08 = "0";
        public string angle09 = "0";
        public string angle10 = "0";
        public string angle11 = "0";
        public string angle12 = "0";
        public string angle13 = "0";
        public string angle14 = "0";
        public string angle15 = "0";
        public string angle16 = "0";
        public string angle17 = "0";

        Skel_Data.Convert_2D23D con3d = new Skel_Data.Convert_2D23D();
        Streaming.SendBuffer sendBuf = new Streaming.SendBuffer();
        
       
        #region Kinect_Matrix

        Matrix<double> mat_X_01;
        Matrix<double> mat_Y_01;
        Matrix<double> mat_Z_01;

        Matrix<double> mat_X_02;
        Matrix<double> mat_Y_02;
        Matrix<double> mat_Z_02;

        #endregion

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
                ulong TrackedOnly = 0;

                foreach (var body in bodies)
                {
                    if (bodies != null)
                    {
                        if (body.IsTracked)
                        {
                            try
                            {    
                                if (TrackedOnly == 0)
                                {
                                    TrackedOnly = body.TrackingId;
                                }

                                if (body.TrackingId == TrackedOnly)
                                {
                                    Kinect_Device.Kinect_Mat_X _Mat_X_01 = new Kinect_Device.Kinect_Mat_X();
                                    Kinect_Device.Kinect_Mat_Y _Mat_Y_01 = new Kinect_Device.Kinect_Mat_Y();
                                    Kinect_Device.Kinect_Mat_Z _Mat_Z_01 = new Kinect_Device.Kinect_Mat_Z();

                                    _Mat_X_01.Get_Bodies(body, sensor);
                                    _Mat_Y_01.Get_Bodies(body, sensor);
                                    _Mat_Z_01.Get_Bodies(body, sensor);

                                    mat_X_01 = _Mat_X_01.body_X;
                                    mat_Y_01 = _Mat_Y_01.body_Y;
                                    mat_Z_01 = _Mat_Z_01.body_Z;

                                    mat_X_01=con3d.ConvertX(mat_X_01, mat_Z_01);
                                    mat_Y_01 = con3d.ConvertY(mat_Y_01, mat_Z_01);
                                    mat_Z_01 = mat_Z_01 * 1000;

                                    Console.WriteLine(mat_X_01);
                                    Console.WriteLine(mat_Y_01);
                                    Console.WriteLine(mat_Z_01);

                                    #region DrawSkeleton_01
                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate { canvas.Children.Clear(); }));
                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawHead = new Ellipse
                                        {
                                            Fill = Brushes.LightPink,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawHead, mat_X_01[0, 0] - drawHead.Width / 2);
                                        Canvas.SetTop(drawHead, mat_Y_01[0, 0] - drawHead.Width / 2);
                                        canvas.Children.Add(drawHead);
                                        textCanvas.Text = "[Monitor Coordinates] \nHead : ("+mat_X_01[0,0].ToString("F3") + "  ,  " + mat_Y_01[0,0].ToString("F3") + "  ,  " + mat_Z_01[0, 0].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawNeck = new Ellipse
                                        {
                                            Fill = Brushes.LightPink,
                                            Width = 10,
                                            Height = 10
                                        };


                                        Canvas.SetLeft(drawNeck, mat_X_01[0, 1] - drawNeck.Width / 2);
                                        Canvas.SetTop(drawNeck, mat_Y_01[0, 1] - drawNeck.Width / 2);
                                        canvas.Children.Add(drawNeck);
                                        textCanvas.Text += "Neck : (" + mat_X_01[0, 1].ToString("F3") + "  ,  " + mat_Y_01[0, 1].ToString("F3") + "  ,  " + mat_Z_01[0, 1].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawSpineMid = new Ellipse
                                        {
                                            Fill = Brushes.LightPink,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawSpineMid, mat_X_01[0, 2] - drawSpineMid.Width / 2);
                                        Canvas.SetTop(drawSpineMid, mat_Y_01[0, 2] - drawSpineMid.Width / 2);
                                        canvas.Children.Add(drawSpineMid);
                                        textCanvas.Text += "SpineMid : (" + mat_X_01[0, 2].ToString("F3") + "  ,  " + mat_Y_01[0, 2].ToString("F3") + "  ,  " + mat_Z_01[0, 2].ToString("F3") + ")\n";
                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawSpineBase = new Ellipse
                                        {
                                            Fill = Brushes.LightPink,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawSpineBase, mat_X_01[0, 3] - drawSpineBase.Width / 2);
                                        Canvas.SetTop(drawSpineBase, mat_Y_01[0, 3] - drawSpineBase.Width / 2);
                                        canvas.Children.Add(drawSpineBase);
                                        textCanvas.Text += "SpineBase : (" + mat_X_01[0, 3].ToString("F3") + "  ,  " + mat_Y_01[0, 3].ToString("F3") + "  ,  " + mat_Z_01[0, 3].ToString("F3") + ")\n";
                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawShoulderLeft = new Ellipse
                                        {
                                            Fill = Brushes.LimeGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawShoulderLeft, mat_X_01[1, 0] - drawShoulderLeft.Width / 2);
                                        Canvas.SetTop(drawShoulderLeft, mat_Y_01[1, 0] - drawShoulderLeft.Width / 2);
                                        canvas.Children.Add(drawShoulderLeft);
                                        textCanvas.Text += "ShoulderLeft : (" + mat_X_01[1, 0].ToString("F3") + "  ,  " + mat_Y_01[1, 0].ToString("F3") + "  ,  " + mat_Z_01[1, 0].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawElbowLeft = new Ellipse
                                        {
                                            Fill = Brushes.LightSkyBlue,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawElbowLeft, mat_X_01[1, 1] - drawElbowLeft.Width / 2);
                                        Canvas.SetTop(drawElbowLeft, mat_Y_01[1, 1] - drawElbowLeft.Width / 2);
                                        canvas.Children.Add(drawElbowLeft);
                                        textCanvas.Text += "ElbowLeft : (" + mat_X_01[1, 1].ToString("F3") + "  ,  " + mat_Y_01[1, 1].ToString("F3") + "  ,  " + mat_Z_01[1, 1].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawWristLeft = new Ellipse
                                        {
                                            Fill = Brushes.LightSeaGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawWristLeft, mat_X_01[1, 2] - drawWristLeft.Width / 2);
                                        Canvas.SetTop(drawWristLeft, mat_Y_01[1, 2] - drawWristLeft.Width / 2);
                                        canvas.Children.Add(drawWristLeft);
                                        textCanvas.Text += "WristLeft : (" + mat_X_01[1, 2].ToString("F3") + "  ,  " + mat_Y_01[1, 2].ToString("F3") + "  ,  " + mat_Z_01[1, 2].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawThumbLeft = new Ellipse
                                        {
                                            Fill = Brushes.LightSeaGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawThumbLeft, mat_X_01[1, 2] - drawThumbLeft.Width / 2);
                                        Canvas.SetTop(drawThumbLeft, mat_Y_01[1, 2] - drawThumbLeft.Width / 2);
                                        canvas.Children.Add(drawThumbLeft);
                                        textCanvas.Text += "ThumbLeft : (" + mat_X_01[1 , 3].ToString("F3") + "  ,  " + mat_Y_01[1, 3].ToString("F3") + "  ,  " + mat_Z_01[1, 3].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawShoulderRight = new Ellipse
                                        {
                                            Fill = Brushes.LimeGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawShoulderRight, mat_X_01[2, 0] - drawShoulderRight.Width / 2);
                                        Canvas.SetTop(drawShoulderRight, mat_Y_01[2, 0] - drawShoulderRight.Width / 2);
                                        canvas.Children.Add(drawShoulderRight);
                                        textCanvas.Text += "ShoulderRight : (" + mat_X_01[2, 0].ToString("F3") + "  ,  " + mat_Y_01[2, 0].ToString("F3") + "  ,  " + mat_Z_01[2, 0].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawElbowRight = new Ellipse
                                        {
                                            Fill = Brushes.LightSkyBlue,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawElbowRight, mat_X_01[2, 1] - drawElbowRight.Width / 2);
                                        Canvas.SetTop(drawElbowRight, mat_Y_01[2, 1] - drawElbowRight.Width / 2);
                                        canvas.Children.Add(drawElbowRight);
                                        textCanvas.Text += "ElbowRight : (" + mat_X_01[2, 1].ToString("F3") + "  ,  " + mat_Y_01[2, 1].ToString("F3") + "  ,  " + mat_Z_01[2, 1].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawWristRight = new Ellipse
                                        {
                                            Fill = Brushes.LightSeaGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawWristRight, mat_X_01[2, 2] - drawWristRight.Width / 2);
                                        Canvas.SetTop(drawWristRight, mat_Y_01[2, 2] - drawWristRight.Width / 2);
                                        canvas.Children.Add(drawWristRight);
                                        textCanvas.Text += "WristRight : (" + mat_X_01[2, 2].ToString("F3") + "  ,  " + mat_Y_01[2, 2].ToString("F3") + "  ,  " + mat_Z_01[2, 2].ToString("F3") + ")\n";

                                    }));

                                    Dispatcher.Invoke(DispatcherPriority.Normal, new Action(delegate
                                    {
                                        Ellipse drawThumbRight = new Ellipse
                                        {
                                            Fill = Brushes.LightSeaGreen,
                                            Width = 10,
                                            Height = 10
                                        };

                                        Canvas.SetLeft(drawThumbRight, mat_X_01[2, 2] - drawThumbRight.Width / 2);
                                        Canvas.SetTop(drawThumbRight, mat_Y_01[2, 2] - drawThumbRight.Width / 2);
                                        canvas.Children.Add(drawThumbRight);
                                        textCanvas.Text += "ThumbRight : (" + mat_X_01[2, 3].ToString("F3") + "  ,  " + mat_Y_01[2, 3].ToString("F3") + "  ,  " + mat_Z_01[2, 3].ToString("F3") + ")\n";

                                    }));

                                    #endregion

                                    #region SendAngleto EveR
                                    //sendBuf.SendBuf(angle01+angle02+ angle03 + angle04 + angle05 + angle06 + angle07 + angle08 + angle09 + angle10 + angle11 + angle12 + angle13 + angle14 + angle15 + angle16 + angle17);
                                    #endregion
                                }

                            }

                            catch (NullReferenceException ex)
                            {
                                Console.WriteLine(ex.ToString());
                            }

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

