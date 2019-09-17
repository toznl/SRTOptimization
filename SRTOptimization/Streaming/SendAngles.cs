using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using System.Net.Sockets;

namespace SRTOptimization.Streaming
{
    public class SendAngles
    {
        public SendAngles()
        {

        }

        public void SendBuf(string str)
        {
            TcpClient tc = new TcpClient("192.168.0.100", 5001);                   // 에버 제어 PC 아이피 확인 후 변경
            NetworkStream stream = tc.GetStream();

            byte[] check_sum = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            int packet_type = 0x02;
            int tr_no = 0;
            int data_type = 5000;
            string[] stringArray = new string[1492];

            string data = str;
            byte[] buff = Encoding.ASCII.GetBytes(data);
            int data_len = data.Length;

            check_sum[0] = 0x55;
            check_sum[1] = (byte)(packet_type & 0xff);
            check_sum[2] = (byte)(tr_no & 0xff);
            check_sum[3] = (byte)((tr_no >> 8) & 0xff);
            check_sum[4] = (byte)(data_len & 0xff);
            check_sum[5] = (byte)((data_len >> 8) & 0xff);
            check_sum[6] = (byte)(data_type & 0xff);
            check_sum[7] = (byte)((data_type >> 8) & 0xff);

            int sum = check_sum[0] + check_sum[1] + check_sum[2] + check_sum[3] + check_sum[4] + check_sum[5] +
                      check_sum[6] + check_sum[7];
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

            stream.Close();
            tc.Close();

        }
    }
}
