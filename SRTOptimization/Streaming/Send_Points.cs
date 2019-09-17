using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net;
using System.Net.Sockets;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SRTOptimization.Streaming
{
    public class Send_Points
    {
        static Socket sock;
        public void Send_Skel(Matrix<double> skel_set)
        {
            {
                sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                IPEndPoint localEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 1234);

                try
                {
                    sock.Connect(localEndPoint);
                }
                catch
                {
                    Console.Write("Unable to connect to remote end point!\r\n");
                }

                Console.Write("Enter Text: ");
                string text = Console.ReadLine();
                byte[] data = Encoding.UTF8.GetBytes(text);

                sock.Send(data);
                Console.Write("Data Sent!\r\n");
                Console.Write("Press any key To continue...");
                Console.Read();
                sock.Close();
            }
        }
    }

}
