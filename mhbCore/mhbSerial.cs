﻿using System;
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
        lock (mhbState.Lock) {
          mSerialPort.WriteLine(mhbState.g.leftSpeed.ToString() + ", " +
                                mhbState.g.rightSpeed.ToString() + "\0");
        }

        Thread.Sleep(50);
      }
    }

    private SerialPort mSerialPort;
  }
}
