using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO.Ports;

namespace myHelperBot
{
  class mhbSerial
  {
    public mhbSerial()
    {
      mSerialPort = new SerialPort();
      mSerialPort.PortName = "COM7";
      mSerialPort.BaudRate = 57600;
      mSerialPort.Open();
    }

    public void loop()
    {
      while (true) {
        lock (mhbGlobal.gLock) {
          //mSerialPort.WriteLine(mhbGlobal.gLeftSpeed.ToString() + ", " + mhbGlobal.gRightSpeed.ToString());
          String data = "test\0";
          Console.WriteLine(data);
          mSerialPort.Write(data.ToCharArray(), 0, data.Length);
        }

        Thread.Sleep(50);
      }
    }

    private SerialPort mSerialPort;
  }
}
