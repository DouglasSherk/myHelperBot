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
      Init();
    }

    public void loop()
    {
      mhbCore.DebugThread("serial thread started");

      while (true) {
        mhbCore.DebugThread("serial spin");

        if (mSerialPort != null && !mSerialPort.IsOpen) {
          mSerialPort.Close();
          mSerialPort.Dispose();
          mSerialPort = null;
        }

        if (mSerialPort == null) {
          Thread.Sleep(SERIAL_INTERVAL);
          Init();
        } else {
          lock (mhbState.Lock) {
            try {
              mSerialPort.WriteLine(mhbState.g.motors.leftSpeed.ToString() + ", " +
                                    mhbState.g.motors.rightSpeed.ToString() + "\0");
            }
            catch {
              Console.WriteLine("Serial not connected, retrying in " + SERIAL_INTERVAL/1000 + "s...");
              mSerialPort = null;
            }
          }

          Thread.Sleep(50);
        }
      }
    }

    private void Init()
    {
      try {
        mLastConnectAttempt = DateTime.Now;

        mSerialPort = new SerialPort();
        mSerialPort.PortName = "COM8";
        mSerialPort.BaudRate = 57600;
        mSerialPort.Open();

        if (!mSerialPort.IsOpen) {
          throw new Exception();
        }

        mhbCore.DebugPrint("Serial connection established");
      }
      catch {
        Console.WriteLine("Serial not connected, retrying in " + SERIAL_INTERVAL/1000 + "s...");
        mSerialPort = null;
      }
    }

    private const int SERIAL_INTERVAL = 3000;

    private SerialPort mSerialPort;

    private DateTime mLastConnectAttempt;
  }
}