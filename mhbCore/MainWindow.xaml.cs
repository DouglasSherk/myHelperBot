/////////////////////////////////////////////////////////////////////////
//
// This module contains code to do Kinect NUI initialization and
// processing and also to display NUI streams on screen.
//
// Copyright © Microsoft Corporation.  All rights reserved.  
// This code is licensed under the terms of the 
// Microsoft Kinect for Windows SDK (Beta) 
// License Agreement: http://kinectforwindows.org/KinectSDK-ToU
//
/////////////////////////////////////////////////////////////////////////
using System;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Documents;
using System.Windows.Input;
using Microsoft.Research.Kinect.Nui;

namespace myHelperBot
{
  /// <summary>
  /// Interaction logic for MainWindow.xaml
  /// </summary>
  public partial class MainWindow : Window
  {
    #region ctor & Window events
    public MainWindow()
    {
      InitializeComponent();
      mCore = new mhbCore();
    }

    private void Window_Loaded(object sender, EventArgs e)
    {
      //Watch for Kinects connecting, disconnecting - and gracefully handle them.
      Runtime.Kinects.StatusChanged += new EventHandler<StatusChangedEventArgs>(Kinects_StatusChanged);

      //create a KinectViewer for each Kinect that is found.
      CreateAllKinectViewers();

      WindowState = WindowState.Minimized;
    }

    private void Window_Closed(object sender, EventArgs e)
    {
      CleanUpAllKinectViewers();
    }
    #endregion ctor & Window events

    #region UI
    private void updateUI()
    {
      if (mCore != null) {
        switchToAnotherKinectSensor.Visibility = System.Windows.Visibility.Visible;
        insertKinectSensor.Visibility = System.Windows.Visibility.Collapsed;
      } else {
        switchToAnotherKinectSensor.Visibility = System.Windows.Visibility.Collapsed;
        insertKinectSensor.Visibility = System.Windows.Visibility.Visible;
      }
    }
    #endregion UI

    #region Kinect discovery + setup
    private void Kinects_StatusChanged(object sender, StatusChangedEventArgs e)
    {
      switch (e.Status)
      {
        case KinectStatus.Connected:
          mCore.mKinect.Kinect = e.KinectRuntime;
          break;
        case KinectStatus.Disconnected:
          mCore.mKinect.Kinect = null;
          break;
        default:

          break;
      }
      updateUI();
    }
    #endregion Kinect discovery + setup

    #region KinectViewer Utilities
    private void CreateAllKinectViewers()
    {
      foreach (Runtime runtime in Runtime.Kinects)
      {
        mCore.mKinect.Kinect = runtime;
      }
      updateUI();
    }

    private void CleanUpAllKinectViewers()
    {
      foreach (object item in viewerHolder.Items)
      {
        mCore.mKinect.Kinect = null;
      }
      updateUI();
    }
    #endregion KinectViewer Utilities

    #region Private state
    mhbCore mCore;
    #endregion Private state
  }
}
