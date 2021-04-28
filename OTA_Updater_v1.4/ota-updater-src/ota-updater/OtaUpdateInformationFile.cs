/*
This software is subject to the license described in the license.txt file included with this software distribution.You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/
using System;
using System.Text;

namespace OTAUpdater
{
    public class OtaUpdateInformationFile
    {
        public static readonly byte FileType = 0x0E;
        public static readonly uint FreeUploadSpaceUnknonw = UInt32.MaxValue;

        private const byte FileStructureVersion = 0x10;

        #region Properties

        public byte HardwareVersion { get; set; }
        public uint FreeUploadSpace { get; set; }
        public byte ProductIdentifier { get; set; }
        public uint WirelessStackVersionNumber { get; set; }
        public string WirelessStackVersionString { get; set; }
        public uint BootloaderVersionNumber { get; set; }
        public string BootloaderVersionString { get; set; }
        public uint ApplicationVersionNumber { get; set; }
        public string ApplicationVersionString { get; set; }

        #endregion

        /// <summary>
        /// Create an OTA Update Information file from binary content
        /// </summary>
        /// <param name="data">Byte array with raw file</param>
        public OtaUpdateInformationFile(byte[] data)
        {
            byte BootloaderVersion = data[0];

            // Remove the minor version number (least signifigant nibble)
            BootloaderVersion &= 0xF0;

            if ((BootloaderVersion != FileStructureVersion))
                throw new ArgumentException("OTA update information file version not supported");

            HardwareVersion = data[1];
            ProductIdentifier = data[2];
            FreeUploadSpace = BitConverter.ToUInt32(data, 3);
            WirelessStackVersionNumber = BitConverter.ToUInt32(data, 7);
            byte wirelessStackVersionStringLength = data[11];
            WirelessStackVersionString = "";
            if (wirelessStackVersionStringLength != 0)
            {
                WirelessStackVersionString = Encoding.UTF8.GetString(data, 12, wirelessStackVersionStringLength);
                // Trim NULL character and any possible garbage after, for appropriate string comparisons
                int nullIndex = WirelessStackVersionString.IndexOf('\0');
                if (nullIndex > 0)
                    WirelessStackVersionString = WirelessStackVersionString.Remove(nullIndex);

            }
            BootloaderVersionNumber = BitConverter.ToUInt32(data, 12 + wirelessStackVersionStringLength);
            byte bootloaderVersionStringLength = data[16 + wirelessStackVersionStringLength];
            BootloaderVersionString = "";
            if (bootloaderVersionStringLength != 0)
            {
                BootloaderVersionString = Encoding.UTF8.GetString(data, 17 + wirelessStackVersionStringLength, bootloaderVersionStringLength);
                // Trim NULL character and any possible garbage after, for appropriate string comparisons
                int nullIndex = BootloaderVersionString.IndexOf('\0');
                if (nullIndex > 0)
                    BootloaderVersionString = BootloaderVersionString.Remove(nullIndex);
            }
            ApplicationVersionNumber = BitConverter.ToUInt32(data, 17 + wirelessStackVersionStringLength + bootloaderVersionStringLength);
            byte applicationVersionStringLength = data[21 + wirelessStackVersionStringLength + bootloaderVersionStringLength];
            ApplicationVersionString = "";
            if (applicationVersionStringLength != 0)
            {
                ApplicationVersionString = Encoding.UTF8.GetString(data, 22 + wirelessStackVersionStringLength + bootloaderVersionStringLength, applicationVersionStringLength);
                // Trim NULL character and any possible garbage after, for appropriate string comparisons
                int nullIndex = ApplicationVersionString.IndexOf('\0');
                if (nullIndex > 0)
                    ApplicationVersionString = ApplicationVersionString.Remove(nullIndex);
            }
        }

        /// <summary>
        /// Returns printable string with the information in the file
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            var str = new StringBuilder();
            str.AppendLine("Hardware version: " + HardwareVersion);
            str.AppendLine("Product identifier: " + ProductIdentifier);
            if (ApplicationVersionNumber != 0)
            {
                str.AppendLine(String.Format("Application Version: {0} ({1})", ApplicationVersionString, ApplicationVersionNumber));
            }
            else if (!String.IsNullOrEmpty(ApplicationVersionString))
            {
                str.AppendLine(String.Format("Application Version: {0}", ApplicationVersionString));
            }
            else
            {
                str.AppendLine("Application Version: Unknown");
            }
            if (WirelessStackVersionNumber != 0)
            {
                str.AppendLine(String.Format("Wireless Stack Version: {0} ({1})", WirelessStackVersionString, WirelessStackVersionNumber));
            }
            else if (!String.IsNullOrEmpty(WirelessStackVersionString))
            {
                str.AppendLine(String.Format("Wireless Stack Version: {0}", WirelessStackVersionString));
            }
            else
            {
                str.AppendLine("Wireless Stack Version: Unknown");
            }
            if (BootloaderVersionNumber != 0)
            {
                str.AppendLine(String.Format("Bootloader Version: {0} ({1})", BootloaderVersionString, BootloaderVersionNumber));
            }
            else if (!String.IsNullOrEmpty(BootloaderVersionString))
            {
                str.AppendLine(String.Format("Bootloader Version: {0}", BootloaderVersionString));
            }
            else
            {
               str.AppendLine(String.Format("Bootloader Version: Unknown"));
            }
            str.AppendLine("Free Upload Space: " + FreeUploadSpace);

            return str.ToString();
        }
    }
}
