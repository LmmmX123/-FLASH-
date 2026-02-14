import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

BLACK_THRESH = (0, 15)
MIN_AREA = 200
OFFSET = 10  # 向内偏移的像素数，可根据实际情况调整


def detect_blob_and_calc_error():
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
        print(f"L:{H_left},R:{H_right},ERROR:{ERROR}")
        time.sleep(0.2)
    else:
        print("未检测到目标")
        time.sleep(1)
while True:
    detect_blob_and_calc_error()
