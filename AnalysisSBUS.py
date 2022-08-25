#!/usr/bin/python
import serial, sys, time, string, pygame

class SBUSReceiver():
    def __init__(self, port):
        #初始化树莓派串口参数
        self.ser=serial.Serial(port,115200)
        # 常数
        self.START_BYTE = b'\x0f'  #起始字节为0x0f
        self.END_BYTE = b'\x00'		#结束字节为0x00
        self.SBUS_FRAME_LEN = 25	#SBUS帧有25个字节
        self.SBUS_NUM_CHAN = 18		#18个通道
        self.OUT_OF_SYNC_THD = 10
        self.SBUS_NUM_CHANNELS = 18	#18个通道
        self.SBUS_SIGNAL_OK = 0		#信号正常为0
        self.SBUS_SIGNAL_LOST = 1		#信号丢失为1
        self.SBUS_SIGNAL_FAILSAFE = 2	#输出failsafe信号时为2

        # 堆栈变量初始化
        self.isReady = True
        self.lastFrameTime = 0
        self.sbusBuff = bytearray(1)  # 用于同步的单个字节
        self.sbusFrame = bytearray(25)  # 单个SBUS数据帧，25个字节
        self.sbusChannels = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 接收到的各频道值
        self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE
        
        
    def get_rx_channels(self):
        """
        用于读取最后的SBUS通道值
        返回:由18个无符号短元素组成的数组，包含16个标准通道值+ 2个数字(ch17和18)
        """
        return self.sbusChannels

    def get_rx_channel(self, num_ch):
        """
        用于读取最后的SBUS某一特定通道的值
        num_ch: 要读取的某个通道的通道序号
        返回:某一通道的值
        """
        return self.sbusChannels[num_ch]

    def get_failsafe_status(self):
        """
        用于获取最后的FAILSAFE状态
        返回: FAILSAFE状态值
        """
        return self.failSafeStatus


    def decode_frame(self):
        """
        对每帧数据进行解码，每个通道的值在两个或三个不同的字节之间，要读取出来很麻烦
        不过futaba已经发布了下面的解码代码
        """      
        #CH1 = [data2]的低3位 + [data1]的8位（678+12345678 = 678,12345678）
        self.sbusChannels[0]  = ((int(self.sbusFrame[1])    	|int(self.sbusFrame[2])<<8)									& 0x07FF);
        #CH2 = [data3]的低6位 + [data2]的高5位（345678+12345 = 345678,12345 ）
        self.sbusChannels[1]  = ((int(self.sbusFrame[2])>>3 	|int(self.sbusFrame[3])<<5)									& 0x07FF);
        #CH3 = [data5]的低1位 + [data4]的8位 + [data3]的高2位（8+12345678+12 = 8,12345678,12）
        self.sbusChannels[2]  = ((int(self.sbusFrame[3])>>6 	|int(self.sbusFrame[4])<<2 |int(self.sbusFrame[5])<<10)		& 0x07FF);
        
        self.sbusChannels[3]  = ((int(self.sbusFrame[5])>>1 	|int(self.sbusFrame[6])<<7)									& 0x07FF);
        self.sbusChannels[4]  = ((int(self.sbusFrame[6])>>4 	|int(self.sbusFrame[7])<<4)									& 0x07FF);
        self.sbusChannels[5]  = ((int(self.sbusFrame[7])>>7 	|int(self.sbusFrame[8])<<1 |int(self.sbusFrame[9])<<9)   	& 0x07FF);
        self.sbusChannels[6]  = ((int(self.sbusFrame[9])>>2 	|int(self.sbusFrame[10])<<6)									& 0x07FF);
        self.sbusChannels[7]  = ((int(self.sbusFrame[10])>>5	|int(self.sbusFrame[11])<<3)									& 0x07FF);
        self.sbusChannels[8]  = ((int(self.sbusFrame[12])   	|int(self.sbusFrame[13])<<8)									& 0x07FF);
        self.sbusChannels[9]  = ((int(self.sbusFrame[13])>>3	|int(self.sbusFrame[14])<<5)									& 0x07FF);
        self.sbusChannels[10] = ((int(self.sbusFrame[14])>>6	|int(self.sbusFrame[15])<<2|int(self.sbusFrame[16])<<10)	& 0x07FF);
        self.sbusChannels[11] = ((int(self.sbusFrame[16])>>1	|int(self.sbusFrame[17])<<7)									& 0x07FF);
        self.sbusChannels[12] = ((int(self.sbusFrame[17])>>4	|int(self.sbusFrame[18])<<4)									& 0x07FF);
        self.sbusChannels[13] = ((int(self.sbusFrame[18])>>7	|int(self.sbusFrame[19])<<1|int(self.sbusFrame[20])<<9)		& 0x07FF);
        self.sbusChannels[14] = ((int(self.sbusFrame[20])>>2	|int(self.sbusFrame[21])<<6)									& 0x07FF);
        self.sbusChannels[15] = ((int(self.sbusFrame[21])>>5	|int(self.sbusFrame[22])<<3)									& 0x07FF);

        #17频道，第24字节的最低一位
        if int(self.sbusFrame[23])  & 0x0001 :
            self.sbusChannels[16] = 2047
        else:
            self.sbusChannels[16] = 0
        #18频道，第24字节的低第二位，所以要右移一位
        if (int(self.sbusFrame[23]) >> 1) & 0x0001 :
            self.sbusChannels[17] = 2047
        else:
            self.sbusChannels[17] = 0

        #帧丢失位为1时，第24字节的低第三位，与0x04进行与运算
        self.failSafeStatus = self.SBUS_SIGNAL_OK
        if int(self.sbusFrame[23]) & (1 << 2):
            self.failSafeStatus = self.SBUS_SIGNAL_LOST
        #故障保护激活位为1时，第24字节的低第四位，与0x08进行与运算	
        if int(self.sbusFrame[23]) & (1 << 3):
            self.failSafeStatus = self.SBUS_SIGNAL_FAILSAFE
            
    def update(self):
        """
        我们需要至少2帧大小，以确保找到一个完整的帧
        所以我们取出所有的缓存（清空它），读取全部数据，直到捕获新的数据
        首先找到END BYTE并向后查找SBUS_FRAME_LEN，看看它是否是START BYTE
        """
        
        while self.ser.inWaiting() >= self.SBUS_FRAME_LEN:

            tempFrame = self.ser.read(self.SBUS_FRAME_LEN)
            headChar = tempFrame[0]
            if 15 == headChar:
                self.sbusFrame = tempFrame[len(tempFrame)-self.SBUS_FRAME_LEN:len(tempFrame)-1]
                self.decode_frame() #调用解码函数
            else:
                c = ' '
                print ("serial misaligned!")
                while not c == self.START_BYTE:
                    c = self.ser.read(1)
                self.ser.read(self.SBUS_FRAME_LEN-1)
            
    def close(self):
            self.ser.close()


if __name__ == "__main__":

    if len(sys.argv) == 2:

        comport = sys.argv[1]
    else:
        print ("usage: " + sys.argv[0] + " port")
        sys.exit(-1)
        
    #comport = 'com1'
    pygame.init()
    sbus = SBUSReceiver(comport)

    while 1:

        event = pygame.event.poll()

        # TODO: Allow exit via keystroke.

        if event.type == pygame.QUIT:
            sbus.close()
            break


        sbus.update()
        print(sbus.get_failsafe_status(), sbus.get_rx_channels(), str(sbus.ser.inWaiting()).zfill(4) , (time.time()-sbus.lastFrameTime))
    
        # TODO: If system load is too high, increase this sleep time.
        pygame.time.delay(10)



