import serial, time
import sys
import array
import tkinter as tk
import threading

MODBUS_READ_REG = 0x3
MODBUS_WRITE_REG_SINGLE = 0x6
MODBUS_WRITE_REG_MULTIPLE = 0x10


#Spec default BaudRate 115200, Device Address 1
SK120X_DEFAULT_BAUD_RATE    = 115200
SK120X_DEFAULT_DEVICE_ADDR  = 1

SK120X_REG_V_SET       = 0x0000
SK120X_REG_I_SET       = 0x0001
SK120X_REG_VOUT        = 0x0002
SK120X_REG_IOUT        = 0x0003
SK120X_REG_POWER       = 0x0004
SK120X_REG_UIN         = 0x0005
SK120X_REG_AH_LOW      = 0x0006
SK120X_REG_AH_HIGH     = 0x0007
SK120X_REG_WH_LOW      = 0x0008
SK120X_REG_WH_HIGH     = 0x0009
SK120X_REG_OUT_H       = 0x000A
SK120X_REG_OUT_M       = 0x000B
SK120X_REG_OUT_S       = 0x000C
SK120X_REG_T_IN        = 0x000D
SK120X_REG_T_EX        = 0x000E
SK120X_REG_LOCK        = 0x000F
SK120X_REG_PROTECT     = 0x0010
SK120X_REG_CVCC        = 0x0011
SK120X_REG_ONOFF       = 0x0012
SK120X_REG_F_C         = 0x0013
SK120X_REG_B_LED       = 0x0014
SK120X_REG_SLEEP       = 0x0015
SK120X_REG_MODEL       = 0x0016
SK120X_REG_VERSION     = 0x0017
SK120X_REG_SLAVE_ADD   = 0x0018
SK120X_REG_BAUDRATE_L  = 0x0019
SK120X_REG_T_IN_OFFSET = 0x001A
SK120X_REG_T_EX_OFFSET = 0x001B
SK120X_REG_BUZZER      = 0x001C
SK120X_REG_EXTRACT_M   = 0x001D
SK120X_REG_DEVICE      = 0x001E

#SK120X_Reg_Dict = OrderedDict()
SK120X_Reg_Dict = {
      0 : { "reg_name": "V-SET"       ,  "reg_offset":  0x0000,  "curr_value":  0},
      1 : { "reg_name": "I-SET"       ,  "reg_offset":  0x0001,  "curr_value":  0},
      2 : { "reg_name": "VOUT"        ,  "reg_offset":  0x0002,  "curr_value":  0},
      3 : { "reg_name": "IOUT"        ,  "reg_offset":  0x0003,  "curr_value":  0},
      4 : { "reg_name": "POWER"       ,  "reg_offset":  0x0004,  "curr_value":  0},
      5 : { "reg_name": "UIN"         ,  "reg_offset":  0x0005,  "curr_value":  0},
      6 : { "reg_name": "AH-LOW"      ,  "reg_offset":  0x0006,  "curr_value":  0},
      7 : { "reg_name": "AH-HIGH"     ,  "reg_offset":  0x0007,  "curr_value":  0},
      8 : { "reg_name": "WH-LOW"      ,  "reg_offset":  0x0008,  "curr_value":  0},
      9 : { "reg_name": "WH-HIGH"     ,  "reg_offset":  0x0009,  "curr_value":  0},
      10: { "reg_name": "OUT_H"       ,  "reg_offset":  0x000A,  "curr_value":  0},
      11: { "reg_name": "OUT_M"       ,  "reg_offset":  0x000B,  "curr_value":  0},
      12: { "reg_name": "OUT_S"       ,  "reg_offset":  0x000C,  "curr_value":  0},
      13: { "reg_name": "T_IN"        ,  "reg_offset":  0x000D,  "curr_value":  0},
      14: { "reg_name": "T_EX"        ,  "reg_offset":  0x000E,  "curr_value":  0},
      15: { "reg_name": "LOCK"        ,  "reg_offset":  0x000F,  "curr_value":  0},
      16: { "reg_name": "PROTECT"     ,  "reg_offset":  0x0010,  "curr_value":  0},
      17: { "reg_name": "CVCC"        ,  "reg_offset":  0x0011,  "curr_value":  0},
      18: { "reg_name": "ONOFF"       ,  "reg_offset":  0x0012,  "curr_value":  0},
      19: { "reg_name": "F-C"         ,  "reg_offset":  0x0013,  "curr_value":  0},
      20: { "reg_name": "B-LED"       ,  "reg_offset":  0x0014,  "curr_value":  0},
      21: { "reg_name": "SLEEP"       ,  "reg_offset":  0x0015,  "curr_value":  0},
      22: { "reg_name": "MODEL"       ,  "reg_offset":  0x0016,  "curr_value":  0},
      23: { "reg_name": "VERSION"     ,  "reg_offset":  0x0017,  "curr_value":  0},
      24: { "reg_name": "SLAVE-ADD"   ,  "reg_offset":  0x0018,  "curr_value":  0},
      25: { "reg_name": "BAUDRATE_L"  ,  "reg_offset":  0x0019,  "curr_value":  0},
      26: { "reg_name": "T-IN-OFFSET" ,  "reg_offset":  0x001A,  "curr_value":  0},
      27: { "reg_name": "T-EX-OFFSET" ,  "reg_offset":  0x001B,  "curr_value":  0},
      28: { "reg_name": "BUZZER"      ,  "reg_offset":  0x001C,  "curr_value":  0},
      29: { "reg_name": "EXTRACT-M"   ,  "reg_offset":  0x001D,  "curr_value":  0},
      30: { "reg_name": "DEVICE"      ,  "reg_offset":  0x001E,  "curr_value":  0},
}

for idx in range(0, 31):
    print(SK120X_Reg_Dict[idx])


# CRC-16-MODBUS
def calculate_crc16(data: bytes) -> int:
    # 初始化crc为0xFFFF
    crc = 0xFFFF

    # 循环处理每个数据字节
    for byte in data:
        # 将每个数据字节与crc进行异或操作
        crc ^= byte

        # 对crc的每一位进行处理
        for _ in range(8):
            # 如果最低位为1，则右移一位并执行异或0xA001操作(即0x8005按位颠倒后的结果)
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            # 如果最低位为0，则仅将crc右移一位
            else:
                crc = crc >> 1

    # 返回最终的crc值
    return crc


def get_uart_cmd_crc(str_data):

    # 测试数据
    #print("str_data=", str_data)
    uart_cmd_crc = str_data
    test_data = bytes.fromhex(str_data)

    # test_data = bytes("Hello, world!", encoding='utf-8')
    #print("test_data=", test_data)

    # 计算CRC-16校验码
    crc16 = calculate_crc16(test_data)
    #print("crc16=%x"%crc16)

    # 输出校验码值
    #print(f'CRC-16校验码值为: 0x{crc16:04X}')

    byte_swap_crc16_lsb = (crc16 & 0xff00)>>8
    byte_swap_crc16_msb = (crc16 & 0x00ff)

    #print("byte_swap_crc16_lsb=%x, byte_swap_crc16_msb=%x"%(byte_swap_crc16_lsb, byte_swap_crc16_msb))
    uart_cmd_crc = "%s%02x%02x\n"%(str_data, byte_swap_crc16_msb, byte_swap_crc16_lsb)
    #print("uart_cmd_crc=", uart_cmd_crc)

    return byte_swap_crc16_msb, byte_swap_crc16_lsb

def MODBUS_RTU_READ_0x3( addr, mode, offset_start, read_reg_num):
    #print("--------------------------------")
    #print("------MODBUS_RTU_READ_0x3-------")
    #print("--------------------------------")
    uart_cmd = "%02x%02x%02x%02x%02x%02x"%(addr, mode, (offset_start&0xff00)>>8,(offset_start&0x00ff),  (read_reg_num&0xff00)>>8,(read_reg_num&0x00ff))
    crc16_msb, crc16_lsb=get_uart_cmd_crc(uart_cmd)
    #print("crc16_msb=%x, crc16_lsb=%x"%(crc16_msb, crc16_lsb))

    mylist = [addr,mode,(offset_start&0xff00)>>8,(offset_start&0x00ff),  (read_reg_num&0xff00)>>8,(read_reg_num&0x00ff), crc16_msb, crc16_lsb]

    try:
        ser.open()
    except Exception as ex:
        print ("open serial port error " + str(ex))
        exit()

    if ser.isOpen():

        try:

            ser.flushInput() #flush input buffer
            ser.flushOutput() #flush output buffer


            #print("serial.to_bytes(mylist)=", serial.to_bytes(mylist))
            ser.write(serial.to_bytes(mylist))
            data = wait_read()  #wait 0.2s

            #print("UART Response len=", len(data))
            #for i in range(0, len(data)):
            #    print("UART Response data[%d]=0x%x"%(i, data[i]))
            uart_resp = ""
            for i in range(0, len(data)-2):
                uart_resp = uart_resp + "%02x"%data[i]
            #print("rx uart_resp=", uart_resp)
            crc16_msb, crc16_lsb=get_uart_cmd_crc(uart_resp)
            #print("response crc16_msb=%2x, crc16_lsb=%2x"%(crc16_msb, crc16_lsb))
            #print("    rx resp crc16_msb %2x, cal. crc16_msb %2x ==> %s"%(crc16_msb, data[len(data)-1-1], "V" if(crc16_msb == data[len(data)-1-1]) else "XXXX"))
            #print("    rx resp crc16_lsb %2x, cal. crc16_lsb %2x ==> %s"%(crc16_lsb, data[len(data)-1-0], "V" if(crc16_lsb == data[len(data)-1-0]) else "XXXX"))
            if( (crc16_msb == data[len(data)-1-1]) and (crc16_lsb == data[len(data)-1-0])):
                #print("response CRC check OK!")
                #print("---REG Status---")

                REG_VALUE = array.array('i')
                for i in range(0, read_reg_num):
                    REG_VALUE.append((data[i*2+3]<<8) + data[i*2+4])
                    #print("REG {:<16} (offset {:#06x}) = {:#06x} (Dec: {:#6d})".format( SK120X_Reg_Dict[offset_start+i]["reg_name"], offset_start+i, REG_VALUE[i], REG_VALUE[i]))
                    SK120X_Reg_Dict[offset_start+i]["REG_VALUE"] = REG_VALUE[i]
            else:
                print("response CRC check NG!")
            time.sleep(0.3)

            ser.close()
        except Exception as e1:
            print ("communicating error " + str(e1))

    else:
        print ("open serial port error")

    return REG_VALUE

def MODBUS_RTU_WRITE_0x6( addr, mode, offset_start, write_reg_vale):
    #print("--------------------------------")
    #print("------MODBUS_RTU_WRITE_0x6------")
    #print("--------------------------------")

    uart_cmd = "%02x%02x%02x%02x%02x%02x"%(addr, mode, (offset_start&0xff00)>>8,(offset_start&0x00ff),  (write_reg_vale&0xff00)>>8,(write_reg_vale&0x00ff))
    crc16_msb, crc16_lsb=get_uart_cmd_crc(uart_cmd)
    #print("crc16_msb=%x, crc16_lsb=%x"%(crc16_msb, crc16_lsb))

    mylist = [addr,mode,(offset_start&0xff00)>>8,(offset_start&0x00ff),  (write_reg_vale&0xff00)>>8,(write_reg_vale&0x00ff), crc16_msb, crc16_lsb]


    try:
        ser.open()
    except Exception as ex:
        print ("open serial port error " + str(ex))
        exit()

    if ser.isOpen():

        try:

            ser.flushInput() #flush input buffer
            ser.flushOutput() #flush output buffer


            #print("serial.to_bytes(mylist)=", serial.to_bytes(mylist))
            ser.write(serial.to_bytes(mylist))
            data = wait_read()  #wait 0.2s

            #print("UART Response len=", len(data))
            #for i in range(0, len(data)):
            #    print("UART Response data[%d]=0x%x"%(i, data[i]))
            uart_resp = ""
            for i in range(0, len(data)-2):
                uart_resp = uart_resp + "%02x"%data[i]
            #print("rx uart_resp=", uart_resp)
            crc16_msb, crc16_lsb=get_uart_cmd_crc(uart_resp)
            #print("response crc16_msb=%2x, crc16_lsb=%2x"%(crc16_msb, crc16_lsb))
            #print("    rx resp crc16_msb %2x, cal. crc16_msb %2x ==> %s"%(crc16_msb, data[len(data)-1-1], "V" if(crc16_msb == data[len(data)-1-1]) else "XXXX"))
            #print("    rx resp crc16_lsb %x2, cal. crc16_lsb %2x ==> %s"%(crc16_lsb, data[len(data)-1-0], "V" if(crc16_lsb == data[len(data)-1-0]) else "XXXX"))
            if( (crc16_msb == data[len(data)-1-1]) and (crc16_lsb == data[len(data)-1-0])):
                #print("response CRC check OK!")
                #print("---REG Status Update---")

                REG_VALUE = array.array('i')
                for i in range(0, 1):    #write single reg
                    REG_VALUE.append((data[i*2+4]<<8) + data[i*2+5])
                    #print("Set REG {:<16} (offset {:#06x}) = {:#06x} (Dec: {:#6d})".format( SK120X_Reg_Dict[offset_start+i]["reg_name"], offset_start+i, REG_VALUE[i], REG_VALUE[i]))
            else:
                print("response CRC check NG!")
            time.sleep(0.3)

            ser.close()
        except Exception as e1:
            print ("communicating error " + str(e1))

    else:
        print ("open serial port error")
    return REG_VALUE

def wait_read():
    data = ""
    while 1:
        if sys.version_info.major == 3:
            if ser.inWaiting() > 0:
                data = ser.read(10240)
                #print("get----")
                #print("data=",data)
                #print("len data=", len(data))
                #for i in range(0, len(data)):
                #    print("data[%d]=0x%x"%(i, data[i]))

                #lines = data.split(b'\r\n')
                #for line in lines:
                #    print("line=", line)
                #    if line:
                #        try:
                #            print(line.decode('utf-8'))
                #        except:
                #            print("cannot perform line.decode('utf-8')")
                #print("get----done")
                break
            #else:
                #print("wait----")
                #data = ser.read(16)
                #print(data.decode())
                #print("wait----done")

        else: #sys.version_info.major == 2:
            if ser.inWaiting() > 0:
                print (ser.read(10240))
                break
    return data


def sk120x_ctrl_scan_status():
    addr = SK120X_DEFAULT_DEVICE_ADDR
    mode = MODBUS_READ_REG
    offset_start = SK120X_Reg_Dict[0]["reg_offset"]
    read_reg_num = SK120X_REG_DEVICE+1
    MODBUS_RTU_READ_0x3( addr, mode, offset_start, read_reg_num)

def sk120x_ctrl_get_IVWSetOut_status():
    addr = SK120X_DEFAULT_DEVICE_ADDR
    mode = MODBUS_READ_REG
    offset_start = SK120X_Reg_Dict[SK120X_REG_V_SET]["reg_offset"]
    read_reg_num = 5
    RET_REG_VALUE = MODBUS_RTU_READ_0x3( addr, mode, offset_start, read_reg_num)
    #for i in range(0, read_reg_num):
    #    print("return REG {:<16} (offset {:#06x}) = {:#06x} (Dec: {:#6d})".format( SK120X_Reg_Dict[offset_start+i]["reg_name"], offset_start+i, RET_REG_VALUE[i], RET_REG_VALUE[i]))

def sk120x_ctrl_set_V():
    addr = SK120X_DEFAULT_DEVICE_ADDR
    mode = MODBUS_WRITE_REG_SINGLE
    offset_start = SK120X_Reg_Dict[SK120X_REG_V_SET]["reg_offset"]
    write_reg_vale = 2000       #xx.xxV

    MODBUS_RTU_WRITE_0x6( addr, mode, offset_start, write_reg_vale)

def sk120x_ctrl_set_A():
    addr = SK120X_DEFAULT_DEVICE_ADDR
    mode = MODBUS_WRITE_REG_SINGLE
    offset_start = SK120X_Reg_Dict[SK120X_REG_I_SET]["reg_offset"]
    write_reg_vale = 500        #x.xxxA

    MODBUS_RTU_WRITE_0x6( addr, mode, offset_start, write_reg_vale)

def sk120x_ctrl_set_ONOFF():
    addr = SK120X_DEFAULT_DEVICE_ADDR
    mode = MODBUS_WRITE_REG_SINGLE
    offset_start = SK120X_Reg_Dict[SK120X_REG_ONOFF]["reg_offset"]
    write_reg_vale = 1

    MODBUS_RTU_WRITE_0x6( addr, mode, offset_start, write_reg_vale)




# 子執行緒的工作函數
def update_sk120x_status_IVSetOut_job(stop_flag):
    while 1:
        sk120x_ctrl_get_IVWSetOut_status()
        VSET.set("%d.%02dV"%( SK120X_Reg_Dict[SK120X_REG_V_SET]["REG_VALUE"]/100,  SK120X_Reg_Dict[SK120X_REG_V_SET]["REG_VALUE"]%100 ))
        ISET.set("%d.%03dA"%( SK120X_Reg_Dict[SK120X_REG_I_SET]["REG_VALUE"]/1000, SK120X_Reg_Dict[SK120X_REG_I_SET]["REG_VALUE"]%1000))
        VOUT.set("%d.%02dV"%( SK120X_Reg_Dict[SK120X_REG_VOUT ]["REG_VALUE"]/100,  SK120X_Reg_Dict[SK120X_REG_VOUT ]["REG_VALUE"]%100 ))
        IOUT.set("%d.%03dA"%( SK120X_Reg_Dict[SK120X_REG_IOUT ]["REG_VALUE"]/1000, SK120X_Reg_Dict[SK120X_REG_IOUT ]["REG_VALUE"]%1000))
        WOUT.set("%d.%02d0W"%(SK120X_Reg_Dict[SK120X_REG_POWER]["REG_VALUE"]/100,  SK120X_Reg_Dict[SK120X_REG_POWER]["REG_VALUE"]%100 ))
        time.sleep(0.5)
        #if (stop_flag):
        #   break;



if __name__ == '__main__':

    com_port = "COM18"
    argnum=len(sys.argv)
    #print ('%d arguments found' % (argnum)                                          )
    #print ('They are:'                                                              )
    #print (sys.argv                                                                 )
    #print ('=============================\n'                                        )
    print ('==================================================================='    )
    print ('{:^68}'.format('SK120X UART Control')      )
    print ('{:^68}'.format('usage: SK120X_UART_Control.exe COM18')      )
    print ('==================================================================='    )


    print ("sys.argv", sys.argv                 )
    print ("len(sys.argv)", len(sys.argv)       )

    if sys.version_info.major == 3:
        print("Python 3.x")
    elif sys.version_info.major == 2:
        print("Python 2.x")
    else:
        print("Unknow Python version!")

    if (argnum >1 ):
        com_port = sys.argv[1]

    ser = serial.Serial()
    ser.port = com_port

    #115200,N,8,1
    ser.baudrate = SK120X_DEFAULT_BAUD_RATE
    ser.bytesize = serial.EIGHTBITS #number of bits per bytes
    ser.parity = serial.PARITY_NONE #set parity check
    ser.stopbits = serial.STOPBITS_ONE #number of stop bits

    ser.timeout = 0.2          #non-block read 0.2s
    ser.writeTimeout = 0.2     #timeout for write 0.2s
    ser.xonxoff = False    #disable software flow control
    ser.rtscts = False     #disable hardware (RTS/CTS) flow control
    ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control

    sk120x_ctrl_scan_status()
    sk120x_ctrl_set_V()
    sk120x_ctrl_set_A()
    sk120x_ctrl_set_ONOFF()


    # 建立一個子執行緒
    stop_flag = False
    t = threading.Thread(target = update_sk120x_status_IVSetOut_job, args = (lambda : stop_flag, ))

    # 執行該子執行緒
    t.start()

    win = tk.Tk()
    win.geometry('320x100')
    win.title("SK120X")
    VSET = tk.IntVar()
    ISET = tk.IntVar()
    VOUT = tk.IntVar()
    IOUT = tk.IntVar()
    WOUT = tk.IntVar()

    #rdioOne = tk.Radiobutton(win, text='January',variable=radioValue, value=0)
    #rdioTwo = tk.Radiobutton(win, text='Febuary',variable=radioValue, value=1)
    #rdioThree = tk.Radiobutton(win, text='March',variable=radioValue, value=2)
    #rdioOne.grid(column=0, row=0, sticky="W")
    #rdioTwo.grid(column=0, row=1, sticky="W")
    #rdioThree.grid(column=0, row=2, sticky="W")
    lableVSetTitle = tk.Label(win,text='V_SET',background = "#8f8",font=("Arial", 12, "bold"))
    lableVSetTitle.grid(column=1, row=0, columnspan=2, sticky="W")
    lableISetTitle = tk.Label(win,text='I_SET',background = "#fc8",font=("Arial", 12, "bold"))
    lableISetTitle.grid(column=1, row=1, columnspan=2, sticky="W")
    
    labelVSetValue = tk.Label(win, textvariable=VSET,background = "#8f8",font=("Arial", 12, "bold"))
    labelVSetValue.grid(column=3, row=0, sticky="E", padx=20)
    labelISetValue = tk.Label(win, textvariable=ISET,background = "#fc8",font=("Arial", 12, "bold"))
    labelISetValue.grid(column=3, row=1, sticky="E", padx=20)


    lableVOutTitle = tk.Label(win,text='V_OUT',background = "#8f0",font=("Arial", 12, "bold"))
    lableVOutTitle.grid(column=4, row=0, columnspan=2, sticky="W")
    lableIOutTitle = tk.Label(win,text='I_OUT',background = "#fc0",font=("Arial", 12, "bold"))
    lableIOutTitle.grid(column=4, row=1, columnspan=2, sticky="W")
    lableWOutTitle = tk.Label(win,text='W_OUT',background = "#f0f",font=("Arial", 12, "bold"))
    lableWOutTitle.grid(column=4, row=2, columnspan=2, sticky="W")
    
    labelVOutValue = tk.Label(win, textvariable=VOUT,background = "#8f0",font=("Arial", 12, "bold"))
    labelVOutValue.grid(column=6, row=0, sticky="E", padx=20)
    labelIOutValue = tk.Label(win, textvariable=IOUT,background = "#fc0",font=("Arial", 12, "bold"))
    labelIOutValue.grid(column=6, row=1, sticky="E", padx=20)
    labelWOutValue = tk.Label(win, textvariable=WOUT,background = "#f0f",font=("Arial", 12, "bold"))
    labelWOutValue.grid(column=6, row=2, sticky="E", padx=20)

    win.mainloop()



    # 等待 t 這個子執行緒結束
    #stop_flag = True
    t.join()

    print("Done.")

