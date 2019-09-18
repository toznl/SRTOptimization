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
    public class Recv_Points { 
        static byte[] Buffer { get; set; }
        static Socket sock;

        public Recv_Points()
        {

        }

        public void Recv_Skel()
        {

        }

        public void Recv_Skel(byte[] getSkel)
        {
            sock = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            Console.WriteLine("waiting...");
            sock.Bind(new IPEndPoint(IPAddress.Any, 7000));

            while (true)
            {
            
                sock.Listen(1);
                Socket accepted = sock.Accept();
                Console.WriteLine("Socket Accepted");

                Buffer = new byte[accepted.SendBufferSize];
                int bytesRead = accepted.Receive(Buffer);
                getSkel = new byte[bytesRead];

                Console.WriteLine("Read byte");
                sock.Close();
            }
            
        }

    }
}
