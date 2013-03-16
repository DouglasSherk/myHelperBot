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
      mCore.InitKinect();
    }

    private void Window_Closed(object sender, EventArgs e) {

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

    #region Private state
    mhbCore mCore;
    #endregion Private state
  }
}
