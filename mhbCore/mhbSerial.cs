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
      try {
        mSerialPort = new SerialPort();
        mSerialPort.PortName = "COM8";
        mSerialPort.BaudRate = 57600;
        mSerialPort.Open();
      } catch {
        Console.WriteLine("Serial not connected");
      }
    }

    public void loop()
    {
      mhbCore.DebugThread("serial thread started");

      while (true) {
        mhbCore.DebugThread("serial spin");

        if (mSerialPort == null) {
          Console.WriteLine("Serial thread terminating");
          return;
        }

        lock (mhbState.Lock) {
          try {
            mSerialPort.WriteLine(mhbState.g.leftSpeed.ToString() + ", " +
                                  mhbState.g.rightSpeed.ToString() + "\0");
          } catch {
            Console.WriteLine("Serial not connected, thread terminating");
            return;
          }
        }

        Thread.Sleep(50);
      }
    }

    private SerialPort mSerialPort;
  }
}
