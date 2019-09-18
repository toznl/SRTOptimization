using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Runtime.Serialization.Formatters;
using System.Runtime.Serialization.Formatters.Binary;
using System.Runtime.Serialization;
using System.IO;
using System.Security.Cryptography;

namespace SRTOptimization.Streaming
{
    public class SerialFunc
    {
        public byte[] Serialize(object param)
        {
            byte[] encMsg = null;
            using (MemoryStream ms = new MemoryStream())
            {
                IFormatter br = new BinaryFormatter();
                br.Serialize(ms, param);
                encMsg = ms.ToArray();
            }

            return encMsg;
        }

        public T Deserialize<T>(byte[] param)
        {
            using (MemoryStream ms = new MemoryStream(param))
            {
                IFormatter br = new BinaryFormatter();
                return (T)br.Deserialize(ms);
            }
        }

    }
}
