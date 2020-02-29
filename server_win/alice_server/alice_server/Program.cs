using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO.Ports;
using System.Windows.Forms;
using System.Threading;

namespace alice_server
{
    class SerialPortProgram
    {
        const int
            LEFT_CLAW = 0
            , LEFT_ELBOW_PITCH = 1
            , LEFT_ELBOW_YAW = 2
            , LEFT_SHOULDER_PITCH = 3
            , LEFT_SHOULDER_YAW = 4
            , RIGHT_CLAW = 11
            , RIGHT_ELBOW_PITCH = 12
            , RIGHT_ELBOW_YAW = 13
            , RIGHT_SHOULDER_PITCH = 14
            , RIGHT_SHOULDER_YAW = 15
            , HEAD_YAW = 8
            , HEAD_PITCH = 7
            ;
        
        // Create the serial port with basic settings
        static SerialPort port = new SerialPort("COM16",
          115200, Parity.None, 8, StopBits.One);

        static byte[] command_bytes = new byte[21];

        [STAThread]
        static void Main(string[] args)
        {
            for (var i = 0; i<20; i++)
            {
                command_bytes[i] = 90;
            }
            command_bytes[20] = Convert.ToByte('\r');
            command_bytes[LEFT_CLAW] = 10;
            command_bytes[LEFT_ELBOW_PITCH] = 90;
            command_bytes[LEFT_ELBOW_YAW] = 90;
            command_bytes[LEFT_SHOULDER_PITCH] = 90;
            command_bytes[LEFT_SHOULDER_YAW] = 90;
            command_bytes[HEAD_PITCH] = 90;
            command_bytes[HEAD_YAW] = 90;
            command_bytes[RIGHT_CLAW] = 90;
            command_bytes[RIGHT_ELBOW_PITCH] = 90;
            command_bytes[RIGHT_ELBOW_YAW] = 90;
            command_bytes[RIGHT_SHOULDER_PITCH] = 90;
            command_bytes[RIGHT_SHOULDER_YAW] = 90;
            // Instatiate this class
            new SerialPortProgram();
            while (true) {
                try
                {
                    port.Write(command_bytes, 0, command_bytes.Length);
                    Thread.Sleep(1000);
                }
                catch
                {
                    //do nothing
                }
            }
        }

        private SerialPortProgram()
        {
            Console.WriteLine("Incoming Data:");
            port.Handshake = Handshake.XOnXOff;
            port.RtsEnable = true;
            port.DtrEnable = true;
            port.NewLine = "\r";
            port.Encoding = Encoding.UTF8;
            port.WriteTimeout = 100;

            // Attach a method to be called when there
            // is data waiting in the port's buffer
            port.DataReceived += new
              SerialDataReceivedEventHandler(port_DataReceived);

            // Begin communications
            port.Open();
            //Application.Run();
            
        }

        private void port_DataReceived(object sender,
          SerialDataReceivedEventArgs e)
        {
            // Show all the incoming data in the port's buffer
            Console.Write(port.ReadExisting());
        }
    }
}
