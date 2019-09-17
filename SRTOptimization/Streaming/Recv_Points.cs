using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net.Sockets;
using System.Net;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

namespace SRTOptimization.Streaming
{
    public class Recv_Points
    {
        static byte[] Buffer { get; set; }
        static Socket sock;

        public void Recv_Skel(byte[] buf)
        {
            Matrix<double> receivedData;

            sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            sock.Bind(new IPEndPoint(IPAddress.Any, 7000));
            sock.Listen(100);

            Socket accepted = sock.Accept();

            Buffer = new byte[accepted.SendBufferSize];
            int bytesRead = accepted.Receive(Buffer);
            byte[] formatted = new byte[bytesRead];
            for (int i = 0; i < bytesRead; ++i)
            {
                formatted[i] = Buffer[i];
            }

            string strdata = Encoding.UTF8.GetString(formatted);
            Console.Write(strdata + "\r\n");
            Console.Read();

            accepted.Close();
            sock.Close();
        }
        

    }
}
