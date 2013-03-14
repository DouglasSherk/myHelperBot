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
using System.Windows.Media;
using Microsoft.Research.Kinect.Nui;

namespace myHelperBot
{
  /// <summary>
  /// Interaction logic for MainWindow.xaml
  /// </summary>
  public partial class MainWindow : Window
  {
    #region ctor & Window events
    public MainWindow() {
      InitializeComponent();
      mCore = new mhbCore();
    }

    private void Window_Loaded(object sender, RoutedEventArgs e) {
      Runtime.Kinects.StatusChanged += new EventHandler<StatusChangedEventArgs>(Kinects_StatusChanged);

      CreateKinect();
    }

    private void Window_Closed(object sender, EventArgs e) {
      DestroyKinect();
    }
    #endregion ctor & Window events

    #region UI
    private void updateUI() {
      if (mCore.HasKinect()) {
        this.Background = Brushes.Green;
      } else {
        this.Background = Brushes.Red;
      }
    }
    #endregion UI

    #region Kinect discovery + setup
    private void Kinects_StatusChanged(object sender, StatusChangedEventArgs e) {
      switch (e.Status) {
        case KinectStatus.Connected:
          mCore.SetKinect(e.KinectRuntime);
          break;
        case KinectStatus.Disconnected:
          mCore.SetKinect(null);
          break;
        default:

          break;
      }
      updateUI();
    }
    #endregion Kinect discovery + setup

    #region KinectViewer Utilities
    private void CreateKinect() {
      foreach (Runtime runtime in Runtime.Kinects) {
        mCore.SetKinect(runtime);
        break;
      }
      updateUI();
    }

    private void DestroyKinect() {
      mCore.SetKinect(null);
      updateUI();
    }
    #endregion KinectViewer Utilities

    #region Private state
    mhbCore mCore;
    #endregion Private state
  }
}
