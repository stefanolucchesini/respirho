/*
This software is subject to the license described in the license.txt file included with this software distribution.You may not use this file except in compliance with this license.
Copyright © Dynastream Innovations Inc. 2014
All rights reserved.
*/
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;


namespace OTAUpdater
{
    public class SystemUpdateFile
    {
        // Hardware platforms, with specific parsing requirements for hex files
        public class Platform
        {
            public ushort SoftDeviceStartAddress { get; private set; }
            public uint ValidCodeSpaceUpperLimit { get; private set; }
            public string Name { get; private set; }

            public Platform(string name, ushort startAddress, uint upperLimit)
            {
                Name = name;
                SoftDeviceStartAddress = startAddress;
                ValidCodeSpaceUpperLimit = upperLimit;
            }
        }

        public static List<Platform> SupportedPlatforms = new List<Platform>
        {
            new Platform("nRF51", 0x1000, 0x40000),
            new Platform("nRF52", 0x1000, 0x80000)
        };

        // Generic SUF constants
        private const String IdentifierString = ".SUF";
        private const byte HeaderSize = 32;
        private const byte CrcSize = 4;
        private const byte FileStructureVersion = 0x11; // 1.1
        private const byte Architecture = 0x01;

        /// <summary>
        /// This function creates files formatted for OTA Updates
        /// for SoC Architectures with potential updateable application,
        /// bootloader and wireless software stack
        /// </summary>
        /// <param name="applicationPath">Path to file including application image (hex or bin). Set to null if not including an application.</param>
        /// <param name="bootloaderPath">Path to file including bootloader image (hex or bin). Set to null if not including a bootloader.</param>
        /// <param name="wirelessStackPath">Path to file including wireless protocol stack (hex or bin). Set to null if not including a wireless protocol stack.</param>
        /// <param name="platform">Hardware platform the hex file is intended for</param>
        /// <returns>Byte array with encoded file</returns>
        public static byte[] Create(string applicationPath, string bootloaderPath, string wirelessStackPath, Platform platform)
        {
            if(String.IsNullOrEmpty(applicationPath) && String.IsNullOrEmpty(bootloaderPath) && String.IsNullOrEmpty(wirelessStackPath))
                throw new ArgumentException("Please specify at least one image type to update");

            // Figure out size of the application.
            // Exclude anything present outside of the valid code space for the specific platform,
            // as this could include things like UICR values that need to be programmed over JTAG
            // but cannot be included over the air.
            uint applicationSize = 0;
            byte[] appBuffer = null;
            if (!String.IsNullOrEmpty(applicationPath))
            {
                appBuffer = ReadAsBinary(applicationPath, 0, platform.ValidCodeSpaceUpperLimit);
                applicationSize = (uint)appBuffer.Length;
            }

            // Figure out size of the bootloader.
            // Exclude anything present outside of the valid code space for the specific platform,
            // as this could include things like UICR values that need to be programmed over JTAG
            // but cannot be included over the air.
            uint bootloaderSize = 0;
            byte[] bootloaderBuffer = null;
            if (!String.IsNullOrEmpty(bootloaderPath))
            {
                bootloaderBuffer = ReadAsBinary(bootloaderPath, 0, platform.ValidCodeSpaceUpperLimit);
                bootloaderSize = (uint)bootloaderBuffer.Length;
            }

            // Figure out the size of the wireless stack
            // For the specific case of a Nordic SoftDevice, it is distributed as a hex/bin file including
            // both the MBR and SoftDevice; MBR section needs to be removed from the file.
            // Also exclude anything present outside of the valid code space for the specific platform,
            // as this could include things like UICR values that need to be programmed over JTAG
            // but cannot be included over the air.
            uint wirelessStackSize = 0;
            byte[] wirelessStackBuffer = null;
            if (!String.IsNullOrEmpty(wirelessStackPath))
            {
                wirelessStackBuffer = ReadAsBinary(wirelessStackPath, platform.SoftDeviceStartAddress, platform.ValidCodeSpaceUpperLimit);
                wirelessStackSize = (uint)wirelessStackBuffer.Length;
            }

            // Create file with all the images to be uploaded
            // This code does not implement the optional version descriptor block

            // Format header
            byte[] fileBuffer = new byte[HeaderSize + wirelessStackSize + bootloaderSize + applicationSize + CrcSize];
            fileBuffer[0] = HeaderSize; // Header size
            fileBuffer[1] = FileStructureVersion; // File structure version
            fileBuffer[2] = Architecture; // Architecture
            fileBuffer[3] = 0; // Architecture
            Array.Copy(Encoding.UTF8.GetBytes(IdentifierString), 0, fileBuffer, 4, 4); // Encode identifier string
            Array.Clear(fileBuffer, 8, 10); // Reserved, set to zero
            Array.Copy(BitConverter.GetBytes(wirelessStackSize), 0, fileBuffer, 18, 4);  // Wireless stack size,  Little-Endian
            Array.Copy(BitConverter.GetBytes(bootloaderSize), 0, fileBuffer, 22, 4); // Bootloader size, Little-Endian
            Array.Copy(BitConverter.GetBytes(applicationSize), 0, fileBuffer, 26, 4); // Application size, Little-Endian
            fileBuffer[30] = 0; // Version descriptor size
            fileBuffer[31] = 0; // Version descriptor size

            // Copy images
            if (wirelessStackBuffer != null)
            {
                Array.Copy(wirelessStackBuffer, 0, fileBuffer, HeaderSize, wirelessStackBuffer.Length);
            }
            if (bootloaderBuffer != null)
            {
                Array.Copy(bootloaderBuffer, 0, fileBuffer, HeaderSize + wirelessStackSize, bootloaderBuffer.Length);
            }
            if (appBuffer != null)
            {
                Array.Copy(appBuffer, 0, fileBuffer, HeaderSize + wirelessStackSize + bootloaderSize, appBuffer.Length);
            }

            Array.Clear(fileBuffer, fileBuffer.Length-CrcSize, CrcSize); // Zero padding

            // Calculate  CRC of the entire file, including header and binary images present, as well as zero padding
            ushort fileCrc = CRC.Calc16(fileBuffer, fileBuffer.Length - 2);
            Array.Copy(BitConverter.GetBytes(fileCrc), 0, fileBuffer, fileBuffer.Length - 2, 2);

            return fileBuffer;
        }

        /// <summary>
        /// Obtain the total size of all images included in the file, including CRC
        /// (i.e, the data that is written into flash)
        /// </summary>
        /// <param name="fileBytes">Byte array with the conents of an SUF file</param>
        /// <returns>Total image size (bytes)</returns>
        public static uint GetTotalImageSize(byte[] fileBytes)
        {
            uint wirelessStackSize = BitConverter.ToUInt32(fileBytes, 18); // Wireless stack size,  Little-Endian
            uint bootloaderSize = BitConverter.ToUInt32(fileBytes, 22); // Bootloader size, Little-Endian
            uint applicationSize = BitConverter.ToUInt32(fileBytes, 26); // Application size, Little-Endian

            return wirelessStackSize + bootloaderSize + applicationSize + CrcSize;
        }

        /// <summary>
        /// Reads input from a binary or hex file, and converts to binary.
        /// </summary>
        /// <param name="path">Path to file</param>
        /// <param name="offset">Start offset of data to include in file.  Any data prior to this offset is excluded from binary output.</param>
        /// <param name="upperLimit">Upper limit of data to include in file. Any data after this limit is excluded from binary output.</param>
        /// <returns>Byte array with binary contents of file.</returns>
        private static byte[] ReadAsBinary(string path, ushort offset, uint upperLimit)
        {
            FileInfo fi = new FileInfo(path);
            if (!fi.Exists)
                throw new ArgumentException("File at specified path does not exist");

            if (fi.Extension.Equals(".bin"))
            {
                return File.ReadAllBytes(path).Skip(offset).ToArray();
            }

            if (fi.Extension.Equals(".hex"))
            {
                using (MemoryStream binBuffer = new MemoryStream())
                {
                    using (StreamReader hexFile = new StreamReader(path))
                    {
                        uint upperAddress = 0;
                        while (!hexFile.EndOfStream)
                        {
                            string line = hexFile.ReadLine();
                            if (!line.Substring(0, 1).Equals(":"))
                                throw new ArgumentException("Incorrectly formatted hex file");

                            byte recordLength = Byte.Parse(line.Substring(1, 2), NumberStyles.HexNumber);
                            ushort address = UInt16.Parse(line.Substring(3, 4), NumberStyles.HexNumber);
                            byte recordType = Byte.Parse(line.Substring(7, 2), NumberStyles.HexNumber);

                            uint fullAddress = upperAddress + address;

                            if (recordType == 0 &&  // Data Record
                                (fullAddress >= offset) &&  (fullAddress <= upperLimit))  // Address is between the specified lower and upper limit
                            {
                                string data = line.Substring(9, recordLength*2);
                                for (int i = 0; i < data.Length; i = i + 2)
                                {
                                    binBuffer.WriteByte(Byte.Parse(data.Substring(i, 2), NumberStyles.HexNumber));
                                }
                            }
                            else if (recordType == 4) // Extended Linear Address Record
                            {
                                upperAddress = UInt32.Parse(line.Substring(9, 4), NumberStyles.HexNumber) << 16;
                            }
                        }
                    }
                    return binBuffer.ToArray();
                }
            }

            throw new ArgumentException("Unsupported file format provided.");
        }

    }
}
