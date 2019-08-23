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

        public KinectSensor sensor;
        DepthFrameReader depthReader;
        BodyFrameReader bodyReader;
        IList<Body> bodies;

        DenseMatrix Kinect01_x;
        DenseMatrix Kinect01_y;
        DenseMatrix Kinect01_z;

        DenseMatrix Kinect02_x;
        DenseMatrix Kinect02_y;
        DenseMatrix Kinect02_z;



        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += OnLoaded;
            this.Closing += delegate
            {
                Environment.Exit(1);
            };
        }

        private ImageSource ProcessFrame(DepthFrame frame)
        {
           
            KinectCall kinect_01 = new KinectCall();
            
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
        DenseMatrix OnBodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            SRT_Matrix srtMat = new SRT_Matrix();
            
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
                            Kinect01_x(body);
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
