using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace myHelperBot
{
  class mhbCore
  {
    public mhbCore()
    {
      mSerialThread = new Thread(new ThreadStart(mSerial.loop));
      mSerialThread.Start();
    }

    #region state
    public mhbKinect mKinect = new mhbKinect();
    private mhbSerial mSerial = new mhbSerial();

    private Thread mSerialThread;
    #endregion
  }
}
