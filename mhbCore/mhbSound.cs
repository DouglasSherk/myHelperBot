﻿using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Media;

namespace myHelperBot
{
  class mhbSound
  {
    public void loop()
    {
      mhbCore.DebugThread("sound thread started");

      while (true) {
        mhbCore.DebugThread("sound spin");

        mhbState state;
        lock (mhbState.Lock) {
          state = mhbState.g;
        }

        if (state.playFoundSound) {
          mFoundSound.Play();
        } else if (state.playLostSound) {
          mLostSound.Play();
        } else if (state.playGoSound) {
          mGoSound.Play();
        } else if (state.playStopSound) {
          mStopSound.Play();
        } else if (state.playSaveSound) {
          mSaveSound.Play();
        } else if (state.playRelocateSound) {
          mRelocateSound.Play();
        }

        state.playFoundSound = false;
        state.playLostSound = false;
        state.playGoSound = false;
        state.playStopSound = false;
        state.playSaveSound = false;
        state.playRelocateSound = false;

        lock (mhbState.Lock) {
          mhbState.g = state;
        }

        Thread.Sleep(200);
      }
    }

    private SoundPlayer mFoundSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/found.wav"));
    private SoundPlayer mLostSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/lost.wav"));
    private SoundPlayer mGoSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/go.wav"));
    private SoundPlayer mStopSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/stop.wav"));
    private SoundPlayer mSaveSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/save.wav"));
    private SoundPlayer mRelocateSound = new SoundPlayer(Path.Combine(Directory.GetCurrentDirectory(), "../../../sounds/relocate.wav"));
  }
}
