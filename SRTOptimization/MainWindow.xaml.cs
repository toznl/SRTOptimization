using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
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
using System.Threading;

using System.Net;
using System.Net.Sockets;


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
        #region Variables
        //Kinect Variables
        KinectSensor sensor;
        DepthFrameReader depthReader;
        BodyFrameReader bodyReader;
        IList<Body> bodies;

        //Angle Variables
        public double angle01 = 0;
        public double angle02 = 0;
        public double angle03 = 0;
        public double angle04 = 0;
        public double angle05 = 0;
        public double angle06 = 0;
        public double angle07 = 0;
        public double angle08 = 0;
        public double angle09 = 0;
        public double angle10 = 0;
        public double angle11 = 0;
        public double angle12 = 0;
        public double angle13 = 0;
        public double angle14 = 0;
        public double angle15 = 0;
        public double angle16 = 0;
        public double angle17 = 0;
        public string data;

        //Function Call
        Skel_Data.Convert_2D23D con3d = new Skel_Data.Convert_2D23D();
        #endregion
        #region Kinect_Matrix

        Matrix<double> mat_X_01;
        Matrix<double> mat_Y_01;
        Matrix<double> mat_Z_01;

        //Matrix<double> mat_X_02;
        //Matrix<double> mat_Y_02;
        //Matrix<double> mat_Z_02;

        Matrix<double> skel_Mat_01;
        //Matrix<double> skel_Mat_02;
        //byte[] skel_Mat_02_byte;

        #endregion

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += OnLoaded;
            this.Closing += delegate
            {
                Environment.Exit(1);
            };
        }

        #region Kinect V2
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
        #endregion
        void OnBodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            var frame = e.FrameReference.AcquireFrame();

            #region AngleMatrix
            Matrix<double> Angle_Set_ArmUpper = DenseMatrix.OfArray(new double[,]{
                {angle05 },
                {angle06 },
                {angle11 },
                {angle12 }
            });

            Matrix<double> Angle_Set_ArmBelow = DenseMatrix.OfArray(new double[,] {
                {angle07 },
                {angle08 },
                {angle13 },
                {angle14 }
            });
            Matrix<double> Angle_Set_Arm = DenseMatrix.OfArray(new double[,]
            {
                {angle05 },
                {angle06 },
                {angle07 },
                {angle08 },
                {angle09 },
                {angle11 },
                {angle12 },
                {angle13 },
                {angle14 },
                {angle15 }
            });
            #endregion

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
                                #region OnePerson Attach_Kinect
                                if (TrackedOnly == 0)
                                {
                                    TrackedOnly = body.TrackingId;
                                }
                                #endregion

                                if (body.TrackingId == TrackedOnly)
                                {
                                    #region Kinect Matrix Initiate
                                    //Kinect Matrix Init
                                    Kinect_Device.Kinect_Mat_X _Mat_X_01 = new Kinect_Device.Kinect_Mat_X();
                                    Kinect_Device.Kinect_Mat_Y _Mat_Y_01 = new Kinect_Device.Kinect_Mat_Y();
                                    Kinect_Device.Kinect_Mat_Z _Mat_Z_01 = new Kinect_Device.Kinect_Mat_Z();

                                    //Put BodySkeletons to Matrix
                                    _Mat_X_01.Get_Bodies(body, sensor);
                                    _Mat_Y_01.Get_Bodies(body, sensor);
                                    _Mat_Z_01.Get_Bodies(body, sensor);

                                    mat_X_01 = _Mat_X_01.body_X;
                                    mat_Y_01 = _Mat_Y_01.body_Y;
                                    mat_Z_01 = _Mat_Z_01.body_Z;

                                    //Convert X,Y Coordinations to real meters
                                    mat_X_01 = con3d.ConvertX(mat_X_01, mat_Z_01);
                                    mat_Y_01 = con3d.ConvertY(mat_Y_01, mat_Z_01);
                                    mat_X_01 *= 100;
                                    mat_Y_01 *= 100;
                                    mat_Z_01 *= 100;

                                    //Put Values to Skeleton Matrix
                                    skel_Mat_01 = DenseMatrix.OfArray(new double[,]{
                                        {mat_X_01[0,0], mat_Y_01[0,0], mat_Z_01[0,0] }, //Head          0
                                        {mat_X_01[0,1], mat_Y_01[0,1], mat_Z_01[0,1] }, //Shoulder_Mid  1
                                        {mat_X_01[0,2], mat_Y_01[0,2], mat_Z_01[0,2] }, //Spine_Mid     2
                                        {mat_X_01[0,3], mat_Y_01[0,3], mat_Z_01[0,3] }, //Spine_Base    3    
                                        {mat_X_01[1,0], mat_Y_01[1,0], mat_Z_01[1,0] }, //Shoulder_Left 4
                                        {mat_X_01[1,1], mat_Y_01[1,1], mat_Z_01[1,1] }, //Elbow_Left    5
                                        {mat_X_01[1,2], mat_Y_01[1,2], mat_Z_01[1,2] }, //Wrist_Left    6
                                        {mat_X_01[1,3], mat_Y_01[1,3], mat_Z_01[1,3] }, //Thumb_Left    7
                                        {mat_X_01[2,0], mat_Y_01[2,0], mat_Z_01[2,0] }, //Shoulder_Right8
                                        {mat_X_01[2,1], mat_Y_01[2,1], mat_Z_01[2,1] }, //Elbow_Right   9
                                        {mat_X_01[2,2], mat_Y_01[2,2], mat_Z_01[2,2] }, //Wrist_Right   10
                                        {mat_X_01[2,3], mat_Y_01[2,3], mat_Z_01[2,3] }, //Thumb_Right   11
                                         });
                                    #endregion

                                    //Function Call
                                    Skel_Data.Vectorize vector_Func = new Skel_Data.Vectorize();
                                    Angle_Set_ArmUpper = vector_Func.Arm_Transform_Upper(skel_Mat_01);
                                    Angle_Set_ArmBelow = vector_Func.Arm_Transform_Below(skel_Mat_01);
                                    
                                    #region Put data to Matrix
                                    for (int i = 0; i < 2; i++)
                                    {
                                        Angle_Set_ArmUpper[i, 0] = (double)((int)(Angle_Set_ArmUpper[i, 0]) / 5) * 5;
                                        if (Angle_Set_ArmUpper[i, 0] < 0)
                                        {
                                            Angle_Set_ArmUpper[i, 0] = 0;
                                        }

                                        if (Angle_Set_ArmUpper[i, 0] > 90)
                                        {
                                            Angle_Set_ArmUpper[i, 0] = 90;
                                        }

                                        Angle_Set_ArmUpper[i + 2, 0] = (double)(95 - ((int)(Angle_Set_ArmUpper[i + 2, 0]) / 5) * 5);
                                        if (Angle_Set_ArmUpper[i + 2, 0] < 0)
                                        {
                                            Angle_Set_ArmUpper[i + 2, 0] = 0;
                                        }

                                        if (Angle_Set_ArmUpper[i + 2, 0] > 90)
                                        {
                                            Angle_Set_ArmUpper[i + 2, 0] = 90;
                                        }

                                        Angle_Set_ArmBelow[i, 0] = 100-((double)((int)(Angle_Set_ArmBelow[i, 0]) / 5) * 5);

                                        if (Angle_Set_ArmBelow[i, 0] < -40)
                                        {
                                            Angle_Set_ArmBelow[i, 0] = -40;
                                        }

                                        if (Angle_Set_ArmBelow[i, 0] > 40)
                                        {
                                            Angle_Set_ArmBelow[i, 0] = 40;
                                        }

                                        Angle_Set_ArmBelow[i + 2, 0] = (double)((int)(Angle_Set_ArmBelow[i + 2, 0]) / 5) * 5;

                                        if (Angle_Set_ArmBelow[i + 2, 0] < 0)
                                        {
                                            Angle_Set_ArmBelow[i + 2, 0] = 0;
                                        }

                                        if (Angle_Set_ArmBelow[i + 2, 0] > 90)
                                        {
                                            Angle_Set_ArmBelow[i + 2, 0] = 90;
                                        }
                                    }

                                    Angle_Set_Arm[0, 0] = Angle_Set_ArmUpper[0, 0];
                                    Angle_Set_Arm[1, 0] = Angle_Set_ArmUpper[2, 0];
                                    Angle_Set_Arm[2, 0] = Angle_Set_ArmBelow[0, 0];
                                    Angle_Set_Arm[3, 0] = Angle_Set_ArmBelow[2, 0];
                                    Angle_Set_Arm[5, 0] = Angle_Set_ArmUpper[1, 0];
                                    Angle_Set_Arm[6, 0] = Angle_Set_ArmUpper[3, 0];
                                    Angle_Set_Arm[7, 0] = Angle_Set_ArmBelow[1, 0];
                                    Angle_Set_Arm[8, 0] = Angle_Set_ArmBelow[3, 0];
                                    #endregion

                                    //Send AngleData to Robot
                                    data = Angle_Set_Arm[0, 0]+","+Angle_Set_Arm[1, 0] + "," + Angle_Set_Arm[2, 0] + "," + Angle_Set_Arm[3, 0] + "," +Angle_Set_Arm[5, 0] + "," + Angle_Set_Arm[6, 0] + "," + Angle_Set_Arm[7, 0] + "," + Angle_Set_Arm[8, 0];
                                    sending(data);

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
                                        
                                        textCanvas.Text = "[Monitor Coordinates] \nHead : (" + mat_X_01[0, 0].ToString("F3") + "  ,  " + mat_Y_01[0, 0].ToString("F3") + "  ,  " + mat_Z_01[0, 0].ToString("F3") + ")\n";

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
                                        textCanvas.Text += "\n";
                                        textCanvas.Text += "----------------------Angle----------------------\n";
                                        textCanvas.Text += "[Left]\n";
                                        textCanvas.Text += "Frontal    : " + Angle_Set_Arm[0, 0] + "\n";
                                        textCanvas.Text += "Side       : " + Angle_Set_Arm[1, 0] + "\n";
                                        textCanvas.Text += "ElbowSpin      : " + Angle_Set_Arm[2, 0] + "\n";
                                        textCanvas.Text += "Elbow : " + Angle_Set_Arm[3, 0] + "\n";
                                        textCanvas.Text += "\n";
                                        textCanvas.Text += "[Right]\n";
                                        textCanvas.Text += "Frontal    : " + Angle_Set_Arm[5, 0] + "\n";
                                        textCanvas.Text += "Side       : " + Angle_Set_Arm[6, 0] + "\n";
                                        textCanvas.Text += "ElbowSpin      : " + Angle_Set_Arm[7, 0] + "\n";
                                        textCanvas.Text += "Elbow : " + Angle_Set_Arm[8, 0] + "\n";
                                    }));

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
        public void sending(string data)
        {
            TcpClient tc = new TcpClient("192.168.0.200", 5001);
            NetworkStream stream = tc.GetStream();

            byte[] check_sum = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            int packet_type = 0x02;
            int tr_no = 0;
            int data_type = 5000;
            string[] stringArray = new string[1492];
            byte[] buff;
            int data_len;
        
            buff = Encoding.ASCII.GetBytes(data);
            data_len = data.Length;

            check_sum[0] = 0x55;
            check_sum[1] = (byte)(packet_type & 0xff);
            check_sum[2] = (byte)(tr_no & 0xff);
            check_sum[3] = (byte)((tr_no >> 8) & 0xff);
            check_sum[4] = (byte)(data_len & 0xff);
            check_sum[5] = (byte)((data_len >> 8) & 0xff);
            check_sum[6] = (byte)(data_type & 0xff);
            check_sum[7] = (byte)((data_type >> 8) & 0xff);

            int sum = check_sum[0] + check_sum[1] + check_sum[2] + check_sum[3] + check_sum[4] + check_sum[5] + check_sum[6] + check_sum[7];

            int data_sum = 0;

            for (int i = 0; i < data_len; i++)
            {
                data_sum += buff[i];
            }

            check_sum[8] = (byte)(sum & 0xff);
            check_sum[9] = (byte)(data_sum & 0xff);

            byte[] msg = new byte[check_sum.Length + buff.Length];
            Buffer.BlockCopy(check_sum, 0, msg, 0, check_sum.Length);
            Buffer.BlockCopy(buff, 0, msg, check_sum.Length, buff.Length);

            stream.Write(msg, 0, msg.Length);
        }
        private void Button_Click(object sender, RoutedEventArgs e)
        {

        }
    }
}

