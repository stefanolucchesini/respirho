/*
This software is subject to the license described in the license.txt file included with this software distribution.You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Reflection;
using System.Threading;
using System.IO;
using System.ComponentModel;
using ANT_Managed_Library;
using ANT_Managed_Library.ANTFS;

namespace OTAUpdater
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region Constants
        // Default channel parameters for OTA reference design
        private const byte DeviceType = 0x10;
        private const byte RadioFrequency = 50;
#error AntFsNetworkKey needs to be configured // Please comment out this line
// Then uncomment the following line and add the ANT-FS network key
//        readonly byte[] AntFsNetworkKey = { 0x__, 0x__, 0x__, 0x__, 0x__, 0x__, 0x__, 0x__ };
        private const ushort ChannelPeriod = 8192; // 4Hz

        private const byte ConnectRF = (byte) ANT_Managed_Library.ANTFS.RadioFrequency.Auto;

        // Maximum time to wait for an upload to complete
        private const int MinUploadTimeout = 30000; // ms

        // Time steps to update progress
        private const int UpdateSteps = 1000; // ms

        #endregion

        # region Architecture Specific Constants

        [Flags]
        enum ImageType : byte
        {
            Application = 1,
            Bootloader = 2,
            WirelessStack = 4
        }

        // Fixed directory entries used for the different combinations of images that can be uploaded
        readonly Dictionary<ImageType, ushort> DirectoryMapping = new Dictionary<ImageType, ushort>()
        {
            {ImageType.Application, 0xFB01},
            {ImageType.Bootloader, 0xFB02},
            {ImageType.WirelessStack, 0xFB03},
            {ImageType.Application | ImageType.Bootloader, 0xFB04},
            {ImageType.Application | ImageType.WirelessStack, 0xFB05},
            {ImageType.Bootloader | ImageType.WirelessStack, 0xFB06},
            {ImageType.Application | ImageType.Bootloader | ImageType.WirelessStack, 0xFB07}
        };

        #endregion

        #region Private Variables

        enum ApplicationState
        {
            Ready,
            Searching,
            Connected,
            Updating
        };

        // Object to use for synchronization
        readonly object key = new object();

        ANT_Device antDevice;
        ANT_Channel antChannel;
        ANTFS_HostChannel antfsHost;
        ANTFS_HostChannel.Response lastResponse;
        ApplicationState appState;

        // Connection and authentication parameters
        uint serialNumber = 0;
        ushort manufacturerId = 0;
        ushort productId = 0;
        ushort deviceNumber = 0;
        bool usePasskey = false;
        byte[] passKey;

        OtaUpdateInformationFile otaInfoFile;

        #endregion

        #region Main Application Logic

        public MainWindow()
        {
            InitializeComponent();
            Assembly assembly = Assembly.GetExecutingAssembly();
            Version ver = assembly.GetName().Version;

            String configuration = "";
            var attributes = assembly.GetCustomAttributes(typeof(AssemblyConfigurationAttribute),false).ToArray();
            if (attributes.Length > 0)
                configuration = ((AssemblyConfigurationAttribute) attributes[0]).Configuration;

            this.Title += " - v." + ver.Major + "." + ver.Minor + "." + ver.Build + " " + configuration;

            try
            {
                // Enable debug logging
                ANT_Common.enableDebugLogs();
                // Connect to ANT USB stick and configure ANT-FS host
                antDevice = new ANT_Device();
                antChannel = antDevice.getChannel(0);
                antfsHost = new ANTFS_HostChannel(antChannel);
                antfsHost.OnResponse += new Action<ANTFS_HostChannel.Response>(HandleAntFsResponses);
                appState = ApplicationState.Ready;

                UpdateStatusBar("Ready");
                SetUiState(appState);

            }
            catch (Exception)
            {
                UpdateStatusBar("Please connect an ANT USB stick...");
            }
        }

        /// <summary>
        /// Clean up resources when form closes
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void HandleClosing(object sender, CancelEventArgs e)
        {
            // Dispose connection to ANT USB stick and ANT-FS host
            if (antfsHost != null)
            {
                antfsHost.Dispose();
            }
            if (antDevice != null)
            {
                antDevice.Dispose();
            }
        }

        /// <summary>
        /// Handle "Connect" button click to initiate connection to a device matching the configured connection parameters.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bConnect_Click(object sender, RoutedEventArgs eventArgs)
        {
            // Set connection parameters from UI
            SetConnectionParameters();

            // Perform search in background thread to avoid blocking UI
            BackgroundWorker bw = new BackgroundWorker();
            bw.DoWork += (s, e) =>
            {
                try
                {
                    BeginSearch(true);
                }
                catch (Exception ex)
                {
                    UpdateStatusBar("Connection to device failed: " + ex.Message);
                    appState = ApplicationState.Ready;
                }
                finally
                {
                    SetUiState(appState);
                }
            };
            bw.RunWorkerAsync();
        }

        /// <summary>
        /// Handle "Start Update" button click by creating the file to upload and performing update process
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bStartUpdate_Click(object sender, RoutedEventArgs eventArgs)
        {
            // Set authentication parameters from UI
            SetAuthenticationParameters();

            // Create file to be updated
            bool includeApplication = cbUpdateApplication.IsChecked.Value;
            bool includeBootloader = cbUpdateBootloader.IsChecked.Value;
            bool includeWirelessStack = cbUpdateWirelessStack.IsChecked.Value;
            byte[] dataToUpload;
            string appPath = null;
            string bootPath = null;
            string stackPath = null;

            // Determine index to upload to base on selections
            ImageType whatToUpload = 0;
            if (includeApplication)
            {
                whatToUpload |= ImageType.Application;
                appPath = tbApplicationPath.Text;
            }
            if (includeBootloader)
            {
                whatToUpload |= ImageType.Bootloader;
                bootPath = tbBootloaderPath.Text;
            }
            if (includeWirelessStack)
            {
                whatToUpload |= ImageType.WirelessStack;
                stackPath = tbWirelessStackPath.Text;
            }
            try
            {
                dataToUpload = SystemUpdateFile.Create(appPath, bootPath, stackPath, cmbPlatform.SelectedItem as SystemUpdateFile.Platform);
            }
            catch (Exception ex)
            {
                UpdateStatusBar("Unable to create file for update: " + ex.Message);
                appState = ApplicationState.Connected;
                SetUiState(appState);
                return;
            }

            // Validate that image(s) can be uploaded to the device, if OTA info file was downloaded
            if (otaInfoFile != null && otaInfoFile.FreeUploadSpace != OtaUpdateInformationFile.FreeUploadSpaceUnknonw)
            {
                if (SystemUpdateFile.GetTotalImageSize(dataToUpload) > otaInfoFile.FreeUploadSpace)
                {
                    var result =
                        MessageBox.Show("Total image size exceeds the free upload space available in the device. " +
                                        "Unable to proceed with the update.", "Error",
                            MessageBoxButton.OK);
                    if (result == MessageBoxResult.OK)
                    {
                        UpdateStatusBar("Update failed: image too large");
                        appState = ApplicationState.Connected;
                        SetUiState(appState);
                        return;
                    }
                }
            }

            // Perform update in background thread to avoid blocking UI
            BackgroundWorker bw = new BackgroundWorker();
            bw.DoWork += (s, e) =>
            {

                try
                {
                    PerformUpdate(whatToUpload, dataToUpload);
                }
                catch (Exception ex)
                {
                    UpdateStatusBar("Update failed: " + ex.Message);
                }
                finally
                {
                    appState = ApplicationState.Ready;
                    SetUiState(appState);
                }
            };
            bw.RunWorkerAsync();
        }

        /// <summary>
        /// Handle click to "Cancel" button and cancel current operation to go back to previous state
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void bCancel_Click(object sender, RoutedEventArgs e)
        {
            switch (appState)
            {
                case ApplicationState.Updating:
                case ApplicationState.Searching:
                    try { antfsHost.Cancel(); }
                    catch (Exception) { }
                    // No need to wait for response here, search loop has the wait built in
                    break;
                case ApplicationState.Connected:
                    try { antfsHost.Disconnect(); }
                    catch (Exception) { }
                    UpdateStatusBar("Ready");
                    appState = ApplicationState.Ready;
                    SetUiState(appState);
                    break;
            }

        }

        #endregion

        #region ANT-FS Handling

        /// <summary>
        /// Search for a device matching configured connection parameters.
        /// Search times out after 30 seconds.
        /// Authenticate with the device
        /// * Optional *
        /// Download and decode OTA Update Information file
        /// Verify if version string matches with specified value
        /// * To skip downloading info file, set downloadInfoFile to false
        /// </summary>
        private void BeginSearch(bool downloadInfoFile)
        {
            otaInfoFile = null;

            // Configure ANT channel parameters
            antfsHost.SetNetworkKey(0, AntFsNetworkKey);
            antfsHost.SetChannelID(DeviceType, 0); // Wildcard transmission type
            antfsHost.SetChannelPeriod(ChannelPeriod);
            antfsHost.EnablePing(true);

            //Configure advanced bursts and advanced burst splitting
            antDevice.configureAdvancedBursting(true,
                                                0x03,
                                                (ANT_ReferenceLibrary.AdvancedBurstConfigFlags)0,
                                                (ANT_ReferenceLibrary.AdvancedBurstConfigFlags)0);
            antDevice.configureAdvancedBurstSplitting(true);

            // Configure search parameters and begin search
            antfsHost.AddSearchDevice(serialNumber, manufacturerId, productId);

            // Begin search
            antfsHost.SearchForDevice(RadioFrequency, ConnectRF, deviceNumber);
            UpdateStatusBar("Searching for device...");
            appState = ApplicationState.Searching;
            SetUiState(appState);

            lastResponse = ANTFS_HostChannel.Response.None;
            lock (key) Monitor.Wait(key, 30000); //  Search for up to 30 seconds
            if (lastResponse == ANTFS_HostChannel.Response.ConnectPass)
            {
                appState = ApplicationState.Connected;
                UpdateStatusBar("Connected to device");
                var discoveredDevice = antfsHost.GetFoundDeviceParameters();
                if (discoveredDevice != null)
                {
                    ShowDeviceInfo(String.Format("{0}\r\nManufacturer ID: {1}\r\nProduct ID: {2}\r\nSerial Number: {3}",
                        discoveredDevice.FriendlyName, discoveredDevice.DeviceParameters.ManufacturerID,
                        discoveredDevice.DeviceParameters.DeviceType, discoveredDevice.DeviceParameters.DeviceID));
                }
            }
            else
            {
                CancelAntFs(); // cancel search if we timeout
                throw new TimeoutException("Unable to find specified device. Check Connection Settings and try again.");
            }

            // Request authentication
            UpdateStatusBar("Authenticating...");
            lastResponse = ANTFS_HostChannel.Response.None;
            if (usePasskey)
            {
                antfsHost.Authenticate(AuthenticationType.PassKey, passKey, 10000);
            }
            else
            {
                antfsHost.Authenticate(AuthenticationType.None, 10000);
            }

            lock (key) Monitor.Wait(key, 5100);
            if (lastResponse == ANTFS_HostChannel.Response.AuthenticatePass)
            {
                UpdateStatusBar("Authentication Successful");
            }
            else if (lastResponse == ANTFS_HostChannel.Response.AuthenticateReject)
            {
                throw new ArgumentException("Authentication Rejected.  Check Authentication Settings and try again.");
            }
            else
            {
                throw new ArgumentException("Authentication failed, possible RF connectivity issues.");
            }

            if (!downloadInfoFile)
                return;

            // Download directory and find OTA Update Information File
            int retries = 5;
            bool deviceNotReady;
            ushort otaInfoFileIndex = 0;
            do
            {
                deviceNotReady = false;
                antfsHost.DownloadDirectory();
                UpdateStatusBar("Requesting directory");
                lastResponse = ANTFS_HostChannel.Response.None;
                lock (key) Monitor.Wait(key, 15000);
                if (lastResponse == ANTFS_HostChannel.Response.DownloadPass)
                {
                    byte[] downloadData = antfsHost.GetTransferData();
                    if (downloadData.Length > 0)
                    {
                        UpdateStatusBar("Directory received");
                        ANTFS_Directory directory = new ANTFS_Directory(downloadData);
                        var infoFileEntries =
                            directory.GetIndexes(x => x.FileDataType == OtaUpdateInformationFile.FileType);
                    if (infoFileEntries != null && infoFileEntries.Count != 0)
                    {
                        otaInfoFileIndex = infoFileEntries[0];
                    }
                }
            }
            else if (lastResponse == ANTFS_HostChannel.Response.DownloadFail ||
                lastResponse == ANTFS_HostChannel.Response.ConnectionLost)
            {
                DisconnectAntFs();
                throw new ArgumentException("Connection to device lost.");
            }
            else if (lastResponse == ANTFS_HostChannel.Response.None)
                {
                    CancelAntFs();
                    DisconnectAntFs();
                    throw new Exception("Timed out waiting for download response.");
                }
                else if (lastResponse == ANTFS_HostChannel.Response.DownloadNotReady)
                {
                    // Retry download
                    retries--;
                    deviceNotReady = true;
                }
            } while (retries > 0 && deviceNotReady);

            // Download OTA Update Information File, if available
            if (otaInfoFileIndex == 0)
            {
                AppendDeviceInfo("No OTA Update Information Available");
            }
            else
            {
                retries = 5;
                do
                {
                    deviceNotReady = false;
                    antfsHost.Download(otaInfoFileIndex, 0, 0);
                    UpdateStatusBar("Requesting OTA Update Information File");

                    lastResponse = ANTFS_HostChannel.Response.None;
                        lock (key) Monitor.Wait(key, 15000);

                    if (lastResponse == ANTFS_HostChannel.Response.DownloadPass)
                    {
                        byte[] downloadData = antfsHost.GetTransferData();
                        if (downloadData.Length > 0)
                        {
                            otaInfoFile = new OtaUpdateInformationFile(downloadData);
                            AppendDeviceInfo(otaInfoFile.ToString());
                        }
                    }
                    else if (lastResponse == ANTFS_HostChannel.Response.DownloadFail ||
                        lastResponse == ANTFS_HostChannel.Response.ConnectionLost)
                    {
                        DisconnectAntFs();
                        throw new ArgumentException("Connection to device lost.");
                    }
                    else if (lastResponse == ANTFS_HostChannel.Response.None)
                    {
                        CancelAntFs();
                        DisconnectAntFs();
                        throw new Exception("Timed out waiting for download response, possible RF connectivity issues");
                    }
                    else if (lastResponse == ANTFS_HostChannel.Response.DownloadNotReady)
                    {
                        deviceNotReady = true;
                        retries--;
                    }
                } while (deviceNotReady && retries > 0);
            }

            UpdateStatusBar("Connected, click 'Start Update' to begin update, or 'Cancel' to disconnect from device.");
        }


        /// <summary>
        /// Perform OTA Update.  This includes:
        /// * Determining file index to upload to based on images to send
        /// * Upload file to device
        /// * Disconnect from device (image will be activated on device on Disconnect)
        /// </summary>
        private void PerformUpdate(ImageType whatToUpload, byte[] dataToUpload)
        {
            if (!DirectoryMapping.ContainsKey(whatToUpload))
            {
                DisconnectAntFs();
                throw new ArgumentException("Please select at least one image type to update");
            }

            appState = ApplicationState.Updating;
            SetUiState(appState);

            ushort index = DirectoryMapping[whatToUpload];

            int maxUploadTimeout = MinUploadTimeout + dataToUpload.Length*2;
            int retries = 5;
            bool deviceNotReady;
            do
            {
                deviceNotReady = false;
                // Request upload
                antfsHost.Upload(index, dataToUpload);
                UpdateStatusBar("Uploading image(s)... 0%");

                lastResponse = ANTFS_HostChannel.Response.None;
                int totalTimeout = 0;
                while (totalTimeout < maxUploadTimeout && lastResponse == ANTFS_HostChannel.Response.None)
                {
                    lock (key) Monitor.Wait(key, UpdateSteps);
                    totalTimeout += UpdateSteps;
                    UpdateStatusBar(String.Format("Uploading image(s)... {0}%", antfsHost.GetUploadStatus().Percentage));
                }

                if (lastResponse == ANTFS_HostChannel.Response.UploadPass)
                {
                    UpdateStatusBar("Uploading image(s)... 100%");
                    UpdateStatusBar("Image upload complete. Image will be activated, do not power down device!");
                    DisconnectAntFs();
                }
                else if (lastResponse == ANTFS_HostChannel.Response.None)
                {
                    CancelAntFs();
                    DisconnectAntFs();
                    throw new Exception("Timed out during upload.");
                }
                else if (lastResponse == ANTFS_HostChannel.Response.UploadReject) // Not Ready
                {
                    deviceNotReady = true;
                    retries--;
                }
                else
                {
                    DisconnectAntFs();
                    throw new ArgumentException("Image upload failed.");
                }
            } while (retries > 0 && deviceNotReady);
        }

        /// <summary>
        /// Cancel ANT-FS operations
        /// </summary>
        private void CancelAntFs()
        {
            try
            {
                antfsHost.Cancel();

                lastResponse = ANTFS_HostChannel.Response.None;
                lock (key) Monitor.Wait(key, 2000); // Wait for cancel done response (if any) before proceeding
            }
            catch (Exception)
            {
                // Safe to ignore if we are in wrong state
            }
        }

        /// <summary>
        /// Disconnect ANT-FS session
        /// </summary>
        private void DisconnectAntFs()
        {
            try
            {
                antfsHost.Disconnect();

                lastResponse = ANTFS_HostChannel.Response.None;
                lock (key) Monitor.Wait(key, 8000); // Wait for disconnect pass response (if any) before proceeding
            }
            catch (Exception)
            {
                // Safe to ignore if we are in wrong state
            }

        }

        /// <summary>
        /// Handle ANT-FS Responses.
        /// </summary>
        /// <param name="response"></param>
        private void HandleAntFsResponses(ANTFS_HostChannel.Response response)
        {
            // Store the last received response and pulse to indicate other threads that a response
            // was received.
            lastResponse = response;
            lock (key) Monitor.Pulse(key);
        }

        #endregion



        #region UI Code

        /// <summary>
        /// Updates text in status bar in thread safe manner
        /// </summary>
        /// <param name="msg">Text to place in status bar</param>
        private void UpdateStatusBar(string msg)
        {
            if(!Dispatcher.CheckAccess())
            {
                Dispatcher.BeginInvoke(new Action<string>(UpdateStatusBar), msg);
                return;
            }
            lbStatus.Text = msg;
        }

        /// <summary>
        /// Updates text in device information panel
        /// </summary>
        /// <param name="msg">Text to place in device information panel</param>
        private void ShowDeviceInfo(string msg)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.BeginInvoke(new Action<string>(ShowDeviceInfo), msg);
                return;
            }
            tbDeviceInfo.Text = msg;
        }

        /// <summary>
        /// Appends text to device information panel
        /// </summary>
        /// <param name="msg">Text to append</param>
        private void AppendDeviceInfo(string msg)
        {
            if (!Dispatcher.CheckAccess())
            {
                Dispatcher.BeginInvoke(new Action<string>(AppendDeviceInfo), msg);
                return;
            }
            tbDeviceInfo.Text += Environment.NewLine + msg;

        }

        /// <summary>
        /// Update status of the buttons depending on state of application
        /// </summary>
        /// <param name="state">Application state</param>
        private void SetUiState(ApplicationState state)
        {
            if(!Dispatcher.CheckAccess())
            {
                Dispatcher.BeginInvoke(new Action<ApplicationState>(SetUiState), state);
                return;
            }

            switch(state)
            {
                case ApplicationState.Ready:
                    bConnect.IsEnabled = true;
                    bStartUpdate.IsEnabled = false;
                    bCancel.IsEnabled = false;
                    tbDeviceInfo.Text = "";
                    break;
                case ApplicationState.Searching:
                    bConnect.IsEnabled = false;
                    bStartUpdate.IsEnabled = false;
                    bCancel.IsEnabled = true;
                    break;
                case ApplicationState.Connected:
                    bConnect.IsEnabled = false;
                    bStartUpdate.IsEnabled = true;
                    bCancel.IsEnabled = true;
                    break;
                case ApplicationState.Updating:
                    bConnect.IsEnabled = false;
                    bStartUpdate.IsEnabled = false;
                    bCancel.IsEnabled = true;
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// Set connection parameters from UI settings
        /// </summary>
        private void SetConnectionParameters()
        {
            try
            {
                serialNumber = UInt32.Parse(tbSerialNumber.Text);
            }
            catch (Exception)
            {
                UpdateStatusBar("Connection to device failed: Serial number must be a 4-byte unsigned integer");
            }
            try
            {
                manufacturerId = UInt16.Parse(tbManufacturerId.Text);
            }
            catch (Exception)
            {
                UpdateStatusBar("Connection to device failed: Manufacturer ID must be a 2-byte unsigned integer");
            }
            try
            {
                productId = UInt16.Parse(tbProductId.Text);
            }
            catch (Exception)
            {
                throw new Exception("Connection to device failed: Product ID must be a 2-byte unsigned integer");
            }

            if (serialNumber == 0)
            {
                deviceNumber = 0;
            }
            else
            {
                deviceNumber = (ushort)(0xFFFF & serialNumber);
            }
        }

        /// <summary>
        /// Set authentication parameters from UI settings
        /// </summary>
        private void SetAuthenticationParameters()
        {
            if (rbPassthru.IsChecked.Value)
            {
                usePasskey = false;
            }
            else if (rbPasskey.IsChecked.Value)
            {
                usePasskey = true;
                try
                {
                    passKey = ParsePasskey(tbPasskey.Text, 1, 255);
                }
                catch (Exception ex)
                {
                    UpdateStatusBar("Invalid passkey: " + ex.Message);
                }
            }
        }

        /// <summary>
        /// Configure Manufacturer ID based on checkbox input
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void cbFilterManufacturerId_Click(object sender, RoutedEventArgs eventArgs)
        {
            if (cbFilterManufacturerId.IsChecked.Value)
            {
                tbManufacturerId.IsEnabled = true;
            }
            else
            {
                tbManufacturerId.Text = "0";
                tbManufacturerId.IsEnabled = false;
            }
        }

        /// <summary>
        /// Configure Product ID based on checkbox input
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void cbFilterProductId_Click(object sender, RoutedEventArgs eventArgs)
        {
            if(cbFilterProductId.IsChecked.Value)
            {
                tbProductId.IsEnabled = true;
            }
            else
            {
                tbProductId.Text = "0";
                tbProductId.IsEnabled = false;
            }
        }

        /// <summary>
        /// Configure Serial Number based on checkbox input
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void cbFilterSerialNumber_Click(object sender, RoutedEventArgs eventArgs)
        {
            if(cbFilterSerialNumber.IsChecked.Value)
            {
                tbSerialNumber.IsEnabled = true;
            }
            else
            {
                tbSerialNumber.Text = "0";
                tbSerialNumber.IsEnabled = false;
            }
        }

        /// <summary>
        /// Configure Authentication type based on radio buttons
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void rbAuthentication_Click(object sender, RoutedEventArgs eventArgs)
        {
            if(rbPasskey.IsChecked.Value)
            {
                tbPasskey.IsEnabled = true;
            }
            else
            {
                tbPasskey.IsEnabled = false;
            }
        }

        /// <summary>
        /// Parse hex string for passkey
        /// </summary>
        /// <param name="stringHex">Hex string</param>
        /// <param name="minLength">Minimum expected length</param>
        /// <param name="maxLength">Maximum allowed length</param>
        /// <returns></returns>
        private byte[] ParsePasskey(string stringHex, int minLength, int maxLength)
        {
            //First we split the string by the delimiters
            stringHex = stringHex.Replace(",", "-");
            string[] byteStrings = stringHex.Split(new char[] { '-' });

            //Handle parsing if not using delimiters
            if (byteStrings.Length == 1 && byteStrings[0].Length > 2)
            {
                int numBytes = (int)Math.Ceiling(((double)byteStrings[0].Length) / 2);
                string[] hexParse = new string[numBytes];
                for (int j = 0; j < numBytes; ++j)
                {
                    hexParse[j] = byteStrings[0].Substring(j * 2, Math.Min(2, byteStrings[0].Length - j * 2));
                }
                byteStrings = hexParse;
            }

            //Make sure it isn't too long
            if (byteStrings.Length > maxLength)
                throw new Exception("Too many bytes");

            //Parse the given values as hex
            byte[] toReturn = new byte[Math.Max(byteStrings.Length, minLength)];
            int i;
            for (i = 0; i < byteStrings.Length; ++i)
            {
                if (byteStrings[i].Equals(""))
                    throw new Exception("Empty byte in string, please ensure commas/dashes are placed appropriately");
                toReturn[i] = Convert.ToByte(byteStrings[i], 16);
            }

            //Fill with zeroes if it is not minLength bytes
            while (i++ < minLength - 1)
                toReturn[i] = 0;

            return toReturn;
        }

        /// <summary>
        /// Browse for application image file
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bBrowseApplication_Click(object sender, RoutedEventArgs eventArgs)
        {
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.Filter = "Binary Files (*.bin)|*.bin|Intel Hex Files (*.hex)|*.hex";
            dlg.FilterIndex = 2; // hex files by default

            var result = dlg.ShowDialog();
            if (result.HasValue && result.Value)
            {
                string filename = dlg.FileName;
                tbApplicationPath.Text = filename;
                cbUpdateApplication.IsChecked = true;
            }
        }

        /// <summary>
        /// Browse for bootloader image file
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bBrowseBootloader_Click(object sender, RoutedEventArgs eventArgs)
        {
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.DefaultExt = ".hex";
            dlg.Filter = "Binary Files (*.bin)|*.bin|Intel Hex Files (*.hex)|*.hex";
            dlg.FilterIndex = 2; // hex files by default
            dlg.RestoreDirectory = true;

            var result = dlg.ShowDialog();
            if (result.HasValue && result.Value)
            {
                string filename = dlg.FileName;
                tbBootloaderPath.Text = filename;
                cbUpdateBootloader.IsChecked = true;
            }

        }

        /// <summary>
        /// Browse for wireless stack image file
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bBrowseWirelessStack_Click(object sender, RoutedEventArgs eventArgs)
        {
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.DefaultExt = ".hex";
            dlg.Filter = "Intel Hex Files (*.hex)|*.hex";
            dlg.RestoreDirectory = true;

            var result = dlg.ShowDialog();
            if (result.HasValue && result.Value)
            {
                string filename = dlg.FileName;
                tbWirelessStackPath.Text = filename;
                cbUpdateWirelessStack.IsChecked = true;
            }
        }

        /// <summary>
        /// Handle "Create Image" button by creating an update file based on provided image(s).
        /// Output file is saved as .SUF in selected directory.
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="eventArgs"></param>
        private void bCreateImage_Click(object sender, RoutedEventArgs eventArgs)
        {
            byte[] dataToSave;
            try
            {
                string appPath = null;
                string bootPath = null;
                string stackPath = null;
                if (cbUpdateApplication.IsChecked.Value)
                    appPath = tbApplicationPath.Text;
                if (cbUpdateBootloader.IsChecked.Value)
                    bootPath = tbBootloaderPath.Text;
                if (cbUpdateWirelessStack.IsChecked.Value)
                    stackPath = tbWirelessStackPath.Text;
                dataToSave  = SystemUpdateFile.Create(appPath, bootPath, stackPath, cmbPlatform.SelectedItem as SystemUpdateFile.Platform);
            }
            catch (Exception ex)
            {
                UpdateStatusBar("Unable to create file: " + ex.Message);
                return;
            }

            Microsoft.Win32.SaveFileDialog dlg = new Microsoft.Win32.SaveFileDialog();
            dlg.Filter = "SoC Update File (*.suf)|*.suf";
            dlg.RestoreDirectory = true;

            var result = dlg.ShowDialog();
            if(result.HasValue && result.Value)
            {
                Stream imageFile = dlg.OpenFile();
                imageFile.Write(dataToSave, 0, dataToSave.Length);
                imageFile.Close();
                UpdateStatusBar("Created .SUF file successfully");
            }
        }

        /// <summary>
        /// Populate combo box of platforms
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void cmbPlatform_Loaded(object sender, RoutedEventArgs e)
        {
            cmbPlatform.ItemsSource = SystemUpdateFile.SupportedPlatforms;
            cmbPlatform.DisplayMemberPath = "Name";
            cmbPlatform.SelectedIndex = 0;
        }

        #endregion

     }
}
