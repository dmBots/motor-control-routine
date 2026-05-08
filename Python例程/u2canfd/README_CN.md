本仓库提供达妙科技电机python控制例程，目前该控制例程是旧SDK，后续会更新一版新的SDK

## 准备环境和硬件
注意！！使用的usb转canfd模块固件版本必须是1.0.0.3（适配旧SDK）
注意电机 canID以及 masterID，电机波特率为5M，如果没接入多个电机就不要发送多条控制报文，否则会报错
```bash
#打开终端，先安装python版本的libusb库，输入：
pip install pyusb
```

## 读取SN码
```bash
python3 dev_sn.py
```

## 运行控制
```bash
python3 damiao.py
```