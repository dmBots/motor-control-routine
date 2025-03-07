using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices.ComTypes;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;

using DM_USB2CAN;

namespace DM_USB2CAN
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();

            //检测可用串口
            string[] ports = SerialPort.GetPortNames();  //获取可用串口
            string[] unique = ports.Distinct().ToArray(); //删除重复项
            unique = unique.OrderBy(s => int.Parse(Regex.Match(s, @"\d+").Value)).ToArray(); //按数字大小排序 com口
            if (unique.Length > 0)//ports.Length > 0说明有串口可用
            {
                Console.WriteLine(unique.Length);
                foreach (var item in unique)
                    Console.WriteLine(item.ToString());
            }
            //阻塞等待串口选择
            //Console.WriteLine("请输入要选择的串口索引（0起始）：");
            int selectedIndex = 0;//-1;
            //string input = Console.ReadLine();
            //if (int.TryParse(input, out selectedIndex))
            //{
            //    Console.WriteLine($"你选择的索引是：{selectedIndex}");
            //}
            //else
            //{
            //    Console.WriteLine("输入无效，请输入一个有效的整数索引。");
            //    return;
            //}
            //连接
            if (selectedIndex <= unique.Length - 1)
                serialPort.PortName = unique[selectedIndex];
            serialPort.BaudRate = 921600;//波特率
            serialPort.DataBits = 8; //数据位
            serialPort.ReadTimeout = 500;
            serialPort.WriteTimeout = 500;
            serialPort.StopBits = StopBits.One;
            serialPort.Parity = Parity.None;
            serialPort.DataReceived += (sender, e) =>
            {
                if (serialPort.IsOpen)
                {
                    int Length = serialPort.BytesToRead * 2;//处理为HEX时每Byte被分为了两位char
                    serialPort.Read(dataTemp, 0, serialPort.BytesToRead);//dataTemp必须非局部变量（多线程访问）

                    string hexString = BitConverter.ToString(dataTemp).Replace("-", "");

                    // 限制长度
                    if (hexString.Length > Length)
                    {
                        hexString = hexString.Substring(0, Length);
                    }

                    Console.WriteLine("recive:" + hexString);
                }
            };

            serialPort.Open();

            //发送
            // com 串口号  newMsg CAN数据（8个字节） frameId_t CANID  sendtime发送次数 Mode模式（是否返回发送状态）len can数据有效长度
            //public void SendToCanData(SerialPort com, byte[] newMsg, string frameId_t, UInt16 sendtime, uint sendInterval, bool Mode, byte len)

            int canid = 0x00;

            //读id
            byte[] readID_msg = { 0xAA, 0x00, 0x00, 0x55 };
            dm_can.SendToCanData(serialPort, readID_msg, "0x7ff", 1, 1, true, 4);//0x7ff标准帧

            //RID寄存器读
            byte[] ReadRID_msg = { 0x00, 0x00, 0x33, 0x00 };
            ReadRID_msg[0] = (byte)(canid & 0xff);//low Byte
            ReadRID_msg[1] = (byte)(canid >> 8);
            //RID枚举中存储了前32个寄存器对应地址
            ReadRID_msg[3] = (byte)RID.cmode;//控制模式
            dm_can.SendToCanData(serialPort, ReadRID_msg, "0x7ff", 1, 1, true, 8);//0x7ff标准帧

            //电机使能
            byte[] Motor_msg = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
            dm_can.SendToCanData(serialPort, Motor_msg, String.Format("{0:X4}", canid), 1, 1, true, 8);//发送电机控制id需要改为canid

            //传入参数在frameId_t后多加一个 ctrMode控制模式的索引（注意需要与寄存器中的控制模式一致）,移除了长度传参（会根据模式自动截取长度）
            //public void ControlSendtoCAN(SerialPort com, byte[] newMsg, string frameId_t, int ctrMode, UInt16 sendtime, uint sendInterval, bool Mode)

            //电机调试
            byte[] MotorMIT_msg = new byte[8];
            //根据协议填充发送数据
            //can 给定
            float POS = 10, VEL = 10, KP = 100, KD = 1, TOR = 1;
            UInt16 pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
            pos_tmp = dm_can.float_to_uint(POS, P_MIN, P_MAX, 16);//根据范围线性转换为uint
            vel_tmp = dm_can.float_to_uint(VEL, V_MIN, V_MAX, 12);
            kp_tmp = dm_can.float_to_uint(KP, KP_MIN, KP_MAX, 12);
            kd_tmp = dm_can.float_to_uint(KD, KD_MIN, KD_MAX, 12);
            tor_tmp = dm_can.float_to_uint(TOR, T_MIN, T_MAX, 12);
            MotorMIT_msg[0] = (byte)(pos_tmp >> 8);
            MotorMIT_msg[1] = (byte)(pos_tmp & 0xFF);
            MotorMIT_msg[2] = (byte)((vel_tmp >> 4) & 0xFF);
            MotorMIT_msg[3] = (byte)((byte)(((vel_tmp & 0xF) << 4) & 0xFF) | (byte)((kp_tmp >> 8) & 0xFF));
            MotorMIT_msg[4] = (byte)(kp_tmp & 0xFF);
            MotorMIT_msg[5] = (byte)((kd_tmp >> 4) & 0xFF);
            MotorMIT_msg[6] = (byte)(((byte)((kd_tmp & 0xF) << 4) & 0xFF) | (byte)((tor_tmp >> 8) & 0xFF));
            MotorMIT_msg[7] = (byte)(tor_tmp & 0xFF);
            dm_can.ControlSendtoCAN(serialPort, MotorMIT_msg, String.Format("{0:X4}", canid), 0, 1, 1, true);//发送电机控制id需要改为canid
        }
        CanProcess dm_can = new CanProcess();
        SerialPort serialPort = new SerialPort();
        byte[] dataTemp = new byte[4 * 1024];//4KB

        //依据实际改动
        public float P_MIN = -12.5f;
        public float P_MAX = 12.5f;
        public float V_MIN = -45.0f;
        public float V_MAX = 45.0f;
        public float KP_MIN = 0.0f;
        public float KP_MAX = 500.0f;
        public float KD_MIN = 0.0f;
        public float KD_MAX = 5.0f;
        public float T_MIN = -18.0f;
        public float T_MAX = 18.0f;
    }
}
