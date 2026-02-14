import sensor
import image    #其实sensor已经隐式的引入了，但是还是留着吧
import time
from machine import UART
uart = UART(1, baudrate=115200, bits=8, parity=None, stop=1)

#---核心参数---
REAL_WID = 5.0        # 目标真实边长
FOCAL_LENGTH = 289      # 焦距（校过了）
BLACK_THRESH = (0, 10)  # 黑色灰度阈值
MIN_AREA = 200          # 最小色块面积
WHITE_FIND = (230,255)  #后有反选所以在找区块的时候要搞一个白的
SAMPLE_RAN = 5
INTER_TIME = 1
OFFSET = 10  # 向内偏移的像素数，可根据实际情况调整


#--摄像头初始化--
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 开灰度
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=500)    #跳过开头0.1秒
sensor.set_auto_gain(False)    # 关闭自动增益，固定阈值生效
sensor.set_auto_whitebal(False) # 关闭白平衡
#clock = time.clock() 计算帧率可以加上


#---测距函数---
def get_distance(pixel_w):
    """相似三角形测距原理:
        D = (真实 * 焦距) / 像素"""
    if pixel_w < 1:
        return 0.0
    dist = (REAL_WID * FOCAL_LENGTH) / pixel_w
    return round(dist, 2)


#---单次测距函数---
def detect_once():
    img = sensor.snapshot() #取图像
    img_binary = img.binary([BLACK_THRESH], invert=True)#灰度处理
    blobs = img_binary.find_blobs([WHITE_FIND], area_threshold=MIN_AREA, invert=True, merge=True)
    if blobs:
        target = max(blobs, key=lambda b: b.area())
        x, y, w, h = target.rect()
        ratio = min(w, h) / max(w, h)#正方形长宽比为1，但是好像没啥用
        if 0.8 < ratio < 1.2:
            return get_distance(w)
    return 0.0

#---功能二的实现，最后输出ERROR值---
def detect_error():
    """
    核心检测函数：持续检测色块，计算左右高度差（ERROR）
    返回值：tuple (H_left, H_right, ERROR) 或 None（未检测到目标）
    """
    img = sensor.snapshot()
    img_binary = img.binary([BLACK_THRESH])
    blobs = img_binary.find_blobs([(255, 255)],area_threshold=MIN_AREA,merge=True)

    if blobs:
        blob = max(blobs, key=lambda x: x.area())
        x0, y0, w, h = blob.rect()

        # 计算偏移后的左右列
        left_col = x0 + OFFSET
        right_col = x0 + w - 1 - OFFSET

        # 确保偏移后的列仍在色块范围内
        if left_col >= x0 + w or right_col < x0:
            print("偏移过大，超出色块范围")

        # 统计左列的高度
        left_col_pixels = []
        for cy in range(y0, y0 + h):
            if img_binary.get_pixel(left_col, cy) == 255:
                left_col_pixels.append(cy)

        # 统计右列的高度
        right_col_pixels = []
        for cy in range(y0, y0 + h):
            if img_binary.get_pixel(right_col, cy) == 255:
                right_col_pixels.append(cy)

        # 计算高度
        if left_col_pixels:
            H_left = max(left_col_pixels) - min(left_col_pixels)
        else:
            H_left = h

        if right_col_pixels:
            H_right = max(right_col_pixels) - min(right_col_pixels)
        else:
            H_right = h

        ERROR = H_left - H_right
        uart.write(f"ERROR:{ERROR}\n")
        uart.write(ERROR)
        time.sleep(0.2)
    else:
        print("未检测到目标")
        uart.write("ERROR:NONE\n")
        time.sleep(1)


#-----主循环-----
while True:
    if uart.any():
        # 读取1个字节
        data_byte = uart.read(1)
        if data_byte is None:  # 只加这1行，过滤空数据
               continue
        print("收到字节:", data_byte)
        if(data_byte == b'1'):
            dis_list = []
            #---核心逻辑---
            a = 1
            for i in range(SAMPLE_RAN):
                d = detect_once()
                if d > 0:  # 只加入有效距离
                    #print(a,d)
                    a += 1
                    dis_list.append(d)
                #time.sleep(INTER_TIME)  # 采样间隔
            timestamp = time.ticks_ms()
            if dis_list:
                avg_dist = sum(dis_list) / len(dis_list)
                uart.write(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}\n")
                print(f"timestamps: {timestamp}, result: {round(avg_dist, 2)}")
            else:
                uart.write(f"timestamps: {timestamp}, result: NONE\n")
                print(f"timestamps: {timestamp}, result: NONE")  # 无有效检测
        #else(data_byte !=)
        elif(data_byte == b'2'):
            detect_error()

"""
剩下的未解决的问题
1. 重复发送 ERROR + 数字直接发串口报错
问题：uart.write(f"ERROR:{ERROR}\n") 已经发了字符串，后面又加了 uart.write(ERROR) —— 直接发数字会触发 TypeError（UART 只能发字符串 / 字节）。
改法：删掉 uart.write(ERROR) 这一行就行。

2. 偏移过大仍会计算无效 ERROR
问题：判断 “偏移过大” 后只打印提示，没终止函数，依然会算 ERROR 并发送。
改法：在 print("偏移过大...") 后加 2 行：

uart.write("ERROR:OFFSET_ERR\n")  # 发偏移错误标识
return  # 终止函数，不执行后续计算

3. detect_once 返回值不区分 “没找到色块” 和 “形状不对”
问题：两种情况都返回 0.0，调试时不知道为啥测距失败。
改法（可选）：
没找到色块：return -1
形状不对：return -2
有效距离：return 实际值
（不影响功能，只是方便调试，可改可不改）
"""
