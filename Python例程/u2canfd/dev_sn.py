import usb.core
import usb.util

def list_u2canfd_devices():
    target_vid = 0x34B7
    target_pid = 0x6877
    dual_target_pid = 0x6632
    target_pids = {target_pid, dual_target_pid}

    # 查找所有匹配设备
    devices = usb.core.find(
        find_all=True,
        idVendor=target_vid,
        custom_match=lambda dev: dev.idProduct in target_pids,
    )

    for i, dev in enumerate(devices):
        try:
            # 获取字符串描述符：Serial Number
            serial_number = "[No serial number]"
            if dev.iSerialNumber:
                try:
                    serial_number = usb.util.get_string(dev, dev.iSerialNumber)
                except usb.core.USBError:
                    pass  # 保持默认值

            print(f"U2CANFD_DEV {i}:")
            print(f"  VID: 0x{dev.idVendor:04x}")
            print(f"  PID: 0x{dev.idProduct:04x}")
            print(f"  SN: {serial_number}")
            print()

        except usb.core.USBError as e:
            print(f"Failed to access device {i}: {e}")

if __name__ == "__main__":
    list_u2canfd_devices()
