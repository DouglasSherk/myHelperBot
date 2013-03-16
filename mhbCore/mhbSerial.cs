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

    /** This has to be above DataReceivedHandler for some reason. */
    private SerialPort mSerialPort;

    private void DataReceivedHandler(object obj,
                                     SerialDataReceivedEventArgs e)
    {
      lock (mSerialPort) {
        Thread.Sleep(50);
        string message = mSerialPort.ReadExisting();
        mhbCore.DebugSerial(mSerialPort, message);
      }
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
            SaveState saveState = SaveState.None;
            if (mhbState.g.startSavingVector) {
              saveState = SaveState.StartSavingVector;
            } else if (mhbState.g.moveToSavedVector) {
              saveState = SaveState.MoveToSavedVector;
            }

            mhbState.g.startSavingVector = false;
            mhbState.g.moveToSavedVector = false;

            string sendingMessage = "@" + (int)saveState + ", " +
                                   mhbState.g.motors.leftSpeed.ToString() + ", " +
                                   mhbState.g.motors.rightSpeed.ToString() + "*";

            //mhbCore.DebugSerial(mSerialPort, sendingMessage.Replace("*", string.Empty));

            try {
              lock (mSerialPort) {
                mSerialPort.WriteLine(sendingMessage);
              }
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
        mSerialPort.DataBits = 8;
        mSerialPort.BaudRate = 115200;
        mSerialPort.Open();

        mSerialPort.DtrEnable = true;

        mSerialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);

        Thread.Sleep(2000);

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

    private DateTime mLastConnectAttempt;

    private enum SaveState
    {
      None = 0,
      StartSavingVector = 1,
      MoveToSavedVector = 2,
    }
  }
}