using mcp2210_dll_m;
using System;
using System.Text;

namespace HID_Command_Line
{
    public class Mcp2210Device
    {
        public IntPtr Handle    { get; set; }
        public string sPassword { get; set; }
        public byte pwr;
        public byte wake;
        public ushort current;
        public uint uinTxferSize;
        public uint uinIndex;
        public byte bFgSelector;
        public byte[] baGpSetting { get; set; }
        public uint uinGpioVal { get; set; }
        public uint uinGpioDir;
        public byte bRmtWkupEn; 
        public byte bPinMode;
        public byte bSpiBusRelEn;
        public uint uinBaudRate;
        public uint uinIdleCsVal;
        public uint uinActiveCsVal;
        public uint uinCsToDataDly;
        public uint uinDataToCsDly;
        public uint uinDataToDataDly; //Data Byte to Data Byte delay for MCP2210
        public byte bSpiMd;
        public uint pgpioPinVal;
        public byte[] eepromData;
        public static ushort VID;
        public static ushort PID;


        public Mcp2210Device(IntPtr handle, string password, uint index, uint TxSize)
        {
            VID = 0x04D8;
            PID = 0x00DE;
            current = 200;
            wake = 1;
            pwr = 0b11000000;
            Handle = handle;
            sPassword = password;
            uinIndex = index;
            uinTxferSize = TxSize;
            bFgSelector = (byte)MCP2210.M_MCP2210_VM_CONFIG;
            baGpSetting = new byte[MCP2210.M_MCP2210_GPIO_NR];
            uinGpioVal = 0;
            uinGpioDir = 0;
            bRmtWkupEn = (byte)MCP2210.M_MCP2210_REMOTE_WAKEUP_DISABLED;
            bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_NONE;
            bSpiBusRelEn = (byte)MCP2210.M_MCP2210_SPI_BUS_RELEASE_DISABLED;
            uinBaudRate = 0;
            uinIdleCsVal = 0;
            uinActiveCsVal = 0;
            uinCsToDataDly = 0;
            uinDataToCsDly = 0;
            uinDataToDataDly = 0;
            bSpiMd = 0;
            pgpioPinVal = 0;
            eepromData = new byte[256];
        }
       
    }
    public class PullbackException : Exception
    {
        public int Mcp2210Error { get; set; }
        public PullbackException()
        {
            Mcp2210Error = 0;
        }

        public PullbackException(string message)
            : base(message)
        {
        }

        public PullbackException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }

    class Program
    {
        

        static Mcp2210Device d1 = new Mcp2210Device((IntPtr)null, "00000000", 0, 2);
        static void Main(string[] args)
        {
            string scommand;
            Console.WriteLine("Continue Connecting To Your MCP2210 Device? [y] or [n]\r\n");
            scommand = Console.ReadLine();
            Console.Write("\r\n");
            if (scommand == "y")
            {
                if (ConnectUSB() == false)
                {
                    Console.WriteLine("System does not register the device. Please connect the device");
                }
                //else if (i > 1)
                //{
                //  Console.WriteLine("To Many Devices Connected. Only one is to be run at a time \r\n");
                //Console.Read();

                //}
                if (SetUpDeviceSettings() != 0)
                {

                }
                try
                {
                    Console.WriteLine("Please Enter the Number for the Command that you wish to complete \r\n 1 : Set GPIO Settings \r\n 2 : Get GPIO Settings \r\n " +
                    "3 : Set SPI Settings \r\n 4 : Get SPI Settings \r\n 5 : Set GPIO Values \r\n " +
                    "6 : Get GPIO Values \r\n 7 : Set Pin Direction \r\n 8 : Get Pin Directon \r\n " +
                    "9 : Send Access Password \r\n 10 : Write EEPROM \r\n 11 : Read EEPROM \r\n 0 : Exit \r\n");
                    scommand = Console.ReadLine();
                    switch (scommand)
                    {
                        case "0": //exit
                            {
                                break;
                            }
                        case "1": 
                            {
                                SetGpioConfig();
                                break;
                            }
                        case "2": 
                            {
                                GetGpioConfig();
                                break;
                            }
                        case "3":
                            {
                                SetSpiConfig();
                                break;
                            }
                        case "4":
                            {
                                GetSpiConfig();
                                break;
                            }
                        case "5":
                            {
                                SetGpioVal();
                                break;
                            }
                        case "6":
                            {
                                GetGpioVal();
                                break;
                            }
                        case "7":
                            {
                                SetGpioDir();
                                break;
                            }
                        case "8":
                            {
                                GetGpioDir();
                                break;
                            }
                        case "9":
                            {
                                Mcp2210Password();
                                break;
                            }
                        case "10"://Write EEPROM
                            {
                                WriteEeprom();
                                break;
                            }
                        case "11"://Read EEPROM
                            {
                                ReadEeprom();
                                break;
                            }
                        case "12": // SPI Data Transfer
                            {
                                break;
                            }
                        case "13": //Cancel SPI Data Transfer
                            {
                                break;
                            }
                        default:
                            {
                                break;
                            }

                            
                    }

                }
                catch (PullbackException error)
                {
                    switch (error.Mcp2210Error)
                    {
                        case -101:
                            {
                                Console.WriteLine("Error, No Such Index Exists");
                                break;
                            }
                        case -103:
                            {
                                Console.WriteLine("Error, Device Not Found");
                                break;
                            }
                        case -104:
                            {
                                Console.WriteLine("Error, Internal Buffer too small");
                                break;
                            }
                        case -105:
                            {
                                Console.WriteLine("Error, Open Device Error");
                                break;
                            }
                        case -106:
                            {
                                Console.WriteLine("Error, Device Already Connected");
                                
                                break;
                            }
                        case -20:
                            {
                                Console.WriteLine("Error, MALLOC Unsucessful");
                                break;
                            }
                        case -10:
                            {
                                Console.WriteLine("Error, NULL Returned");
                                break;
                            }
                        case -1:
                            {
                                Console.WriteLine("Error, Unknown Error");
                                break;
                            }
                        case 0:
                            {
                                Console.WriteLine("Success");
                                break;
                            }

                    };
                }
                finally
                {
                    ;
                }
                Console.Read();
            }

        }

        public static void SetGpioConfig()      //7h
        {
            ConsoleKeyInfo memType;
            ConsoleKeyInfo gpioTemp;
            string temp_gp;
            int err;
            Console.WriteLine("Would you like to set the GPIO settings for [1] NVRAM or [2] Volitile Memory \r\n");
            memType = Console.ReadKey();
            if (memType.Key == ConsoleKey.D1)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_NVRAM_CONFIG;
            }
            else if (memType.Key == ConsoleKey.D2)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_VM_CONFIG;
            }
            Console.WriteLine("Please Indicate the GP Pin Designation in Hex Format, GP0 is First & GP8 is Last \r\n 00 : GPIO \r\n 01 : Chip Select \r\n 02 : Dedicated Pin Function \r\n ");
            temp_gp = Console.ReadLine();
            d1.baGpSetting = Converter.ToByteArray(temp_gp);
            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Output Value of Pin GPIO{0} \r\n", i);
                gpioTemp = Console.ReadKey();
                if (gpioTemp.Key == ConsoleKey.D1)
                {
                    d1.uinGpioVal += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            Console.WriteLine("The chosen indication is {0} \r\n", d1.uinGpioVal);
            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Direction of Pin GPIO{0}\r\n 1 is In, 0 is Out \r\n", i);
                gpioTemp = Console.ReadKey();
                if (gpioTemp.Key == ConsoleKey.D1)
                {
                    d1.uinGpioDir += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            Console.WriteLine("The chosen direction is {0}", d1.uinGpioDir);
            Console.WriteLine("\r\n Would You Like the Device to Be Remote Wake Enababled [Y] or [N] /r/n");
            gpioTemp = Console.ReadKey();
            if (gpioTemp.Key == ConsoleKey.Y)
            {
                d1.bRmtWkupEn = (byte)MCP2210.M_MCP2210_REMOTE_WAKEUP_ENABLED;
            }
            if (gpioTemp.Key == ConsoleKey.N)
            {
                d1.bRmtWkupEn = (byte)MCP2210.M_MCP2210_REMOTE_WAKEUP_DISABLED;
            }
            Console.WriteLine("\r\n How would you like the interupt pin to count interupts, GP6 \r\n 0 : NONE \r\n 1 : High Pulses \r\n 2 : Low Pulses \r\n 3 : Rising Edges \r\n 4 : Falling Edges \r\n");
            gpioTemp = Console.ReadKey();
            if (gpioTemp.Key == ConsoleKey.D0)
            {
                d1.bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_NONE;
            }
            else if (gpioTemp.Key == ConsoleKey.D1)
            {
                d1.bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_HIGH_PULSES;
            }
            else if (gpioTemp.Key == ConsoleKey.D2)
            {
                d1.bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_LOW_PULSES;
            }
            else if (gpioTemp.Key == ConsoleKey.D3)
            {
               d1.bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_RISING_EDGES;
            }
            else if (gpioTemp.Key == ConsoleKey.D4)
            {
                d1.bPinMode = (byte)MCP2210.M_MCP2210_INT_MD_CNT_FALLING_EDGES;
            }
            Console.WriteLine("\r\n Would you like your device to release the SPI Bus to another master when not broadcasting [Y] or [N] \r\n");
            gpioTemp = Console.ReadKey();
            if (gpioTemp.Key == ConsoleKey.Y)
            {
                d1.bSpiBusRelEn = (byte)MCP2210.M_MCP2210_SPI_BUS_RELEASE_ENABLED;
            }
            if (gpioTemp.Key == ConsoleKey.N)
            {
                d1.bSpiBusRelEn = (byte)MCP2210.M_MCP2210_SPI_BUS_RELEASE_DISABLED;
            }
            err = MCP2210.M_Mcp2210_SetGpioConfig(d1.Handle, d1.bFgSelector, d1.baGpSetting, d1.uinGpioVal, d1.uinGpioDir, d1.bRmtWkupEn, d1.bPinMode, d1.bSpiBusRelEn);
            if (err != 0)
            {
                PullbackException oops = new PullbackException();
                oops.Mcp2210Error = err;
                
                throw oops;
            }
            Console.WriteLine("\r\n Error is {0}", err);
        }  
        public static void GetGpioConfig()      //8h
        {
            int err;
            uint tempuinGpioVal = 0;
            uint tempuinGpioDir = 0;
            byte tempbRmtWkupEn = 0;
            byte tempbPinMode = 0;
            byte tempbSpiBusRelEn = 0;



            ConsoleKeyInfo memType;
            Console.WriteLine("Would you like to see GPIO settings for [1] NVRAM or [2] Volitile Memory \r\n");
            memType = Console.ReadKey();
            if (memType.Key == ConsoleKey.D1)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_NVRAM_CONFIG;
            }
            else if (memType.Key == ConsoleKey.D2)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_VM_CONFIG;
            }
            err = MCP2210.M_Mcp2210_GetGpioConfig(d1.Handle, d1.bFgSelector, d1.baGpSetting,  ref tempuinGpioVal,  ref tempuinGpioDir, ref tempbRmtWkupEn, ref tempbPinMode, ref tempbSpiBusRelEn);
            Converter.CheckByteArray(d1.baGpSetting);
            d1.uinGpioVal = tempuinGpioVal;
            d1.uinGpioDir = tempuinGpioDir;
            d1.bRmtWkupEn = tempbRmtWkupEn;
            d1.bPinMode = tempbPinMode;
            d1.bSpiBusRelEn = tempbSpiBusRelEn;

            Console.WriteLine("The chosen indication is {0} \r\n", d1.uinGpioVal);
            Console.WriteLine("The chosen direction is {0} \r\n", d1.uinGpioDir);
            if (d1.bRmtWkupEn == (byte)MCP2210.M_MCP2210_REMOTE_WAKEUP_ENABLED)
            {
                Console.WriteLine("Remote Wake Up is Enabled");
            }
            else if (d1.bRmtWkupEn == (byte)MCP2210.M_MCP2210_REMOTE_WAKEUP_DISABLED)
            {
                Console.WriteLine("Remote Wake Up is Disabled");
            }
            if (d1.bPinMode == (byte)MCP2210.M_MCP2210_INT_MD_CNT_NONE)
            {
                Console.WriteLine("Interupt pin is Disabled");
            }
            else if (d1.bPinMode == (byte)MCP2210.M_MCP2210_INT_MD_CNT_HIGH_PULSES)
            {
                Console.WriteLine("Interrupt is High Pulse Triggered");
            }
            else if (d1.bPinMode == (byte)MCP2210.M_MCP2210_INT_MD_CNT_LOW_PULSES)
            {
                Console.WriteLine("Interrupt is Low Pulse Triggered");
            }
            else if (d1.bPinMode == (byte)MCP2210.M_MCP2210_INT_MD_CNT_RISING_EDGES)
            {
                Console.WriteLine("Interrupt is Rising Edge Triggered");
            }
            else if (d1.bPinMode == (byte)MCP2210.M_MCP2210_INT_MD_CNT_FALLING_EDGES)
            {
                Console.WriteLine("Interrupt is Falling Edge Triggered");
            }
            if (d1.bSpiBusRelEn == (byte)MCP2210.M_MCP2210_SPI_BUS_RELEASE_ENABLED)
            {
                Console.WriteLine("SPI Bus Release is Enabled");
            }
            if (d1.bSpiBusRelEn == (byte)MCP2210.M_MCP2210_SPI_BUS_RELEASE_DISABLED)
            {
                Console.WriteLine("SPI Bus Release is Disabled");
            }
            if (err != 0)
            {
                PullbackException oops = new PullbackException();
                oops.Mcp2210Error = err;
                throw oops;
            }
            Console.WriteLine("Error is {0}", err);
        }   
        public static void SetSpiConfig()       //5h
        {
            ConsoleKeyInfo memType;
            ConsoleKeyInfo spiTemp;
            string sSpi;
            int err;

            Console.WriteLine("Would you like to edit [1] NVRAM or [2] Volitile Memory \r\n");
            memType = Console.ReadKey();
            if (memType.Key == ConsoleKey.D1)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_NVRAM_CONFIG;
            }
            else if (memType.Key == ConsoleKey.D2)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_VM_CONFIG;
            }
            Console.WriteLine(" What is your requested baud rate \r\n 12 Mb is Maximum");
            sSpi = Console.ReadLine();
            if (Int32.TryParse(sSpi, out int baud))
            {
                d1.uinBaudRate = (uint)baud;
            }
            else
            {
                //Do something
            }
            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Idle Chip Select Values of CS{0}\r\n 1 or 0 \r\n", i);
                spiTemp = Console.ReadKey();
                if (spiTemp.Key == ConsoleKey.D1)
                {
                    d1.uinIdleCsVal += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Active Chip Select Values of CS{0}\r\n 1 or 0 \r\n", i);
                spiTemp = Console.ReadKey();
                if (spiTemp.Key == ConsoleKey.D1)
                {
                    d1.uinActiveCsVal += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            Console.WriteLine("Select the Delay between Chip Select and Data in Decimal Value , quanta of 100 uS \r\n");
            sSpi = Console.ReadLine();
            if (Int32.TryParse(sSpi, out int cs_data_delay))
            {
                d1.uinCsToDataDly = (uint)cs_data_delay;
            }
            else
            {
                //Do something
            }
            Console.WriteLine("Select the Delay between Last Data Byte and Chip Select in Decimal Value , quanta of 100 uS \r\n");
            sSpi = Console.ReadLine();
            if (Int32.TryParse(sSpi, out int data_cs_delay))
            {
                d1.uinDataToCsDly = (uint)data_cs_delay;
            }
            else
            {
                //Do something
            }
            Console.WriteLine("Select the Delay between Subsequent Data Bytes in Decimal Value , quanta of 100 uS \r\n");
            sSpi = Console.ReadLine();
            if (Int32.TryParse(sSpi, out int data_data_delay))
            {
                d1.uinDataToDataDly = (uint)data_data_delay;
            }
            else
            {
                //Do something
            }
            Console.WriteLine("Select the Total Number of Bytes to Transfer in Decimal Value \r\n");
            sSpi = Console.ReadLine();
            if (Int32.TryParse(sSpi, out int bytes_to_tx))
            {
                d1.uinTxferSize = (uint)bytes_to_tx;
            }
            else
            {
                //Do something
            }
            Console.WriteLine("Please Select the Desired Spi Mode [0] [1] [2] [3] \r\n");
            spiTemp = Console.ReadKey();
            if (spiTemp.Key == ConsoleKey.D0)
            {
                d1.bSpiMd = (byte)MCP2210.M_MCP2210_SPI_MODE0;
            }
            else if (spiTemp.Key == ConsoleKey.D1)
            {
                d1.bSpiMd = (byte)MCP2210.M_MCP2210_SPI_MODE1;
            }
            else if (spiTemp.Key == ConsoleKey.D2)
            {
                d1.bSpiMd = (byte)MCP2210.M_MCP2210_SPI_MODE2;
            }
            else if (spiTemp.Key == ConsoleKey.D3)
            {
                d1.bSpiMd = (byte)MCP2210.M_MCP2210_SPI_MODE3;
            }
            err = MCP2210.M_Mcp2210_SetSpiConfig(d1.Handle, d1.bFgSelector, ref d1.uinBaudRate, ref d1.uinIdleCsVal, ref d1.uinActiveCsVal, ref d1.uinCsToDataDly, ref d1.uinDataToCsDly, ref d1.uinDataToDataDly, ref d1.uinTxferSize, ref d1.bSpiMd);
            Console.WriteLine(" \r\n Error is {0}", err);
            //Handle Error
        }    
        public static void GetSpiConfig()       //6h
        {
            ConsoleKeyInfo memType;
            int err;

            Console.WriteLine("Would you like to see SPI settings for [1] NVRAM or [2] Volitile Memory \r\n");
            memType = Console.ReadKey();
            if (memType.Key == ConsoleKey.D1)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_NVRAM_CONFIG;
            }
            else if (memType.Key == ConsoleKey.D2)
            {
                d1.bFgSelector = (byte)MCP2210.M_MCP2210_VM_CONFIG;
            }
            err = MCP2210.M_Mcp2210_GetSpiConfig(d1.Handle, d1.bFgSelector, ref d1.uinBaudRate, ref d1.uinIdleCsVal, ref d1.uinActiveCsVal, ref d1.uinCsToDataDly, ref d1.uinDataToCsDly, ref d1.uinDataToDataDly, ref d1.uinTxferSize, ref d1.bSpiMd);
            Console.WriteLine("The baud rate is {0}.\r\n The Idle CS Values are {1}.\r\n The Active CS Values are {2}.\r\n The CS to First Data Byte Delay is {3} * 100 uS.\r\n The Last Data Byte to CS Delay is {4} * 100 uS.\r\n The Data to Data Byte Delay is {5} * 100uS.\r\n The Transfer Size is {6}. SPI Mode{7}\r\n ", d1.uinBaudRate, d1.uinIdleCsVal, d1.uinActiveCsVal, d1.uinCsToDataDly, d1.uinDataToCsDly, d1.uinDataToDataDly, d1.uinTxferSize, d1.bSpiMd);
            Console.WriteLine(" \r\n Error is {0}", err);
            //handle error
        }    
        public static void SetGpioVal()         //9h
        {
            int err;
            ConsoleKeyInfo gpioTemp;
            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Output Value of Pin GPIO{0} \r\n", i);
                gpioTemp = Console.ReadKey();
                if (gpioTemp.Key == ConsoleKey.D1)
                {
                    d1.uinGpioVal += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            Console.WriteLine("The chosen indication is {0} \r\n", d1.uinGpioVal);
            uint pgpioPinVal = d1.uinGpioVal;
            err = MCP2210.M_Mcp2210_SetGpioPinVal(d1.Handle, d1.uinGpioVal, ref d1.pgpioPinVal);
            if (err != 0)
            {
                //handle
            }
        }      
        public static void GetGpioVal()         //Ah
        {
            int err;
            err = MCP2210.M_Mcp2210_GetGpioPinVal(d1.Handle, ref d1.pgpioPinVal);
            Console.WriteLine("The Pin Values are {0} \r\n", d1.pgpioPinVal);
            
        }       
        public static void SetGpioDir()         //Bh
        {
            ConsoleKeyInfo gpioTemp;
            int err;

            for (int i = 0; i < 9; i++)
            {
                Console.WriteLine("Please Indicate the Direction of Pin GPIO{0}\r\n 1 is In, 0 is Out \r\n", i);
                gpioTemp = Console.ReadKey();
                if (gpioTemp.Key == ConsoleKey.D1)
                {
                    d1.uinGpioDir += (uint)1 << i;
                }
                Console.WriteLine("\r\n");
            }
            Console.WriteLine("The chosen direction is {0}", d1.uinGpioDir);
            err = MCP2210.M_Mcp2210_SetGpioPinDir(d1.Handle, d1.uinGpioDir);
            if (err != 0)
            {
                //Handle it
            }
        }
        public static void GetGpioDir()         //Ch
        {
            int err;
            err = MCP2210.M_Mcp2210_GetGpioPinDir(d1.Handle, ref d1.uinGpioDir);
            Console.WriteLine("The Pin Directions are {0} \r\n", d1.uinGpioDir);
        }
        public static void SetPassword()
        {

        }
        public static void Mcp2210Password()    //Eh
        {
            int err;
            Console.WriteLine("Please input the Device Password, 8 Character Maximum \r\n");
            d1.sPassword = Console.ReadLine();
            err = MCP2210.M_Mcp2210_EnterPassword(d1.Handle, d1.sPassword);
            Console.WriteLine("Error is {0}", err);
        }
        public static void WriteEeprom()
        {
            byte[] baContentTemp;
            string sEeprom;
            byte[] baAddress;
            byte bContent;
            int err;

            Console.WriteLine("What Adrress would you like to write to? Writen in Hex");
            sEeprom = Console.ReadLine();
            baAddress = Converter.ToByteArray(sEeprom);
            Console.WriteLine("What would you like to write? Writen in Hex");
            sEeprom = Console.ReadLine();
            baContentTemp = Converter.ToByteArray(sEeprom);
            bContent = baContentTemp[0];
            d1.eepromData[baAddress[0]] = baContentTemp[0]; // fills in a byte array that correlates to the memory of the EEPROM. Utility for large reads
            err = MCP2210.M_Mcp2210_WriteEEProm(d1.Handle, baAddress[0], bContent);
            if (err != 0)
            {
                //Handle it
            }
            Console.WriteLine(" your Error response is {0} \r\n Your content is {1] for address {2}", err, baAddress[0], bContent);
        }
        public static void ReadEeprom()
        {
            string sEeprom;
            byte[] baAddress;
            byte bContent = 0;
            int err;
            Console.WriteLine("What Address would you like to read from? Writen in Hex");
            sEeprom = Console.ReadLine();
            baAddress = Converter.ToByteArray(sEeprom);
            err = MCP2210.M_Mcp2210_ReadEEProm(d1.Handle, baAddress[0], ref bContent);
            if (err != 0)
            {
                //Handle it
            }
            d1.eepromData[baAddress[0]] = bContent;
            Console.WriteLine(" your Error response is {0} \r\n Your content is {1] for address {2}", err, baAddress[0], bContent);
        }


        public static bool ConnectUSB() // 0h and 1h

        {
            bool confirmation = false;
            
            
            uint i = (uint) MCP2210.M_Mcp2210_GetConnectedDevCount(Mcp2210Device.VID, Mcp2210Device.PID);
            if (i == 1)
            {
                Console.WriteLine("yeah Baby, Press any Button to Connect to Single Devices in your Area");  //assuming that the index for 1 device is always 0
               // Console.Read();
                d1.Handle = MCP2210.M_Mcp2210_OpenByIndex(Mcp2210Device.VID, Mcp2210Device.PID, i-1, null);
                Console.WriteLine("Handle is {0}", d1.Handle);
                int Error = MCP2210.M_Mcp2210_GetLastError();
                Console.WriteLine("Response is {0} \r\n", Error);
                
                

            }
           




            return confirmation;
        }

       public static int SetUpDeviceSettings( ) //2h
        {
           
            int temp = MCP2210.M_Mcp2210_SetUsbKeyParams(d1.Handle, Mcp2210Device.VID, Mcp2210Device.PID, d1.pwr, d1.wake, d1.current);
            return temp;
       }

    }

    static class Converter
    {
        public static void CheckByteArray(byte[] bytes)
        {
            var convert = new StringBuilder("Is this command correct[] { ");
            foreach (var b in bytes)
            {
                convert.Append(b + ", ");
            }
            convert.Append("} \r\n [y]es, [n]o or Escape to Exit \r\n");
            Console.WriteLine(convert.ToString());
        }

        public static byte[] ToByteArray(String HexString)
        {
            int NumberChars = HexString.Length;
            byte[] bytes = new byte[NumberChars / 2];
            for (int i = 0; i < NumberChars; i += 2)
            {
                bytes[i / 2] = Convert.ToByte(HexString.Substring(i, 2), 16);
            }
            return bytes;
        }
    }
}
