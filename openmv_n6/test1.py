import csi
import time
import sensor
import image
import math
# ===== 颜色阈值（LAB格式） =====
RED_THRESH   = [(20, 40, 28, 48, 18, 38)]
GREEN_THRESH = [(26, 46, -25, -5, 1, 21)]
BLUE_THRESH  = [(29, 49, -20, 0, -28, -8)]
SUB_COUNT = 6   # 分成 6 个等份

# ===== 颜色提取函数（放在循环外，避免每次重定义） =====
def rgb_to_lab(r, g, b):
    # 1. 归一化到 0~1
    r = r / 255.0
    g = g / 255.0
    b = b / 255.0

    # 2. sRGB 到线性 RGB（伽马校正）
    def gamma_correct(c):
        if c > 0.04045:
            return ((c + 0.055) / 1.055) ** 2.4
        else:
            return c / 12.92

    r_lin = gamma_correct(r)
    g_lin = gamma_correct(g)
    b_lin = gamma_correct(b)

    # 3. 线性 RGB 到 XYZ（D65 白点）
    x = r_lin * 0.4124564 + g_lin * 0.3575761 + b_lin * 0.1804375
    y = r_lin * 0.2126729 + g_lin * 0.7151522 + b_lin * 0.0721750
    z = r_lin * 0.0193339 + g_lin * 0.1191920 + b_lin * 0.9503041

    # 4. 归一化到 D65 白点
    x /= 0.95047
    y /= 1.00000
    z /= 1.08883

    # 5. 非线性变换
    def f(t):
        if t > 0.008856:
            return t ** (1/3)
        else:
            return (7.787 * t) + (16 / 116)

    fx = f(x)
    fy = f(y)
    fz = f(z)

    # 6. 计算 L*, a*, b*
    L = (116 * fy) - 16
    a = 500 * (fx - fy)
    b_val = 200 * (fy - fz)

    return L, a, b_val

def is_color_in_thresh(lab, thresh):
    L, a, b_val = lab
    L_min, L_max, A_min, A_max, B_min, B_max = thresh
    return L_min <= L <= L_max and A_min <= a <= A_max and B_min <= b_val <= B_max

def get_color_name(center_x, center_y, img):
    """根据 LAB 阈值判断颜色（红/绿/蓝/UNKNOWN）"""
    # ---------- 关键改动：使用元组传参 ----------
    pixel = img.get_pixel((center_x, center_y))
    # ------------------------------------------
    #调试可以取消注释下面这行，查看中心点的像素值
    #print(f"中心点{center_x}, {center_y}: 像素值: {pixel}")


    lab = rgb_to_lab(pixel[0], pixel[1], pixel[2])

    if is_color_in_thresh(lab, RED_THRESH[0]):
        return "RED"
    elif is_color_in_thresh(lab, GREEN_THRESH[0]):
        return "GREEN"
    elif is_color_in_thresh(lab, BLUE_THRESH[0]):
        return "BLUE"
    else:
        return "UNKNOWN"

def task1():

    # ---------- 如果您想确保彩色，建议改用 csi0 ----------
    # img = csi0.snapshot()   # 取消注释这行，并注释掉下一行
    img = sensor.snapshot()   # 保留原有，但可能导致灰度图
    # ---------------------------------------------------

    # ---------- 裁剪中央 80% 感兴趣区（ROI），屏蔽边缘同色背景 ----------
    # 注意：N6 的 crop 是就地修改，先 copy 避免破坏帧缓冲；roi 须用关键字传参
    W, H = img.width(), img.height()
    roi = (W // 10, H // 10, W - 2 * (W // 10), H - 2 * (H // 10))
    img = img.copy()
    img.crop(roi=roi)
    print("ROI:", roi)
    # --------------------------------------------------------

    original = img.copy()

    # 创建空白掩膜
    mask = img.copy()
    mask.clear()

    tmp = img.copy()
    tmp.binary(RED_THRESH)
    mask.b_or(tmp)

    tmp = img.copy()
    tmp.binary(GREEN_THRESH)
    mask.b_or(tmp)

    tmp = img.copy()
    tmp.binary(BLUE_THRESH)
    mask.b_or(tmp)

    #mask.close(2)


    #此处取消注释可以显示掩膜图像，便于调试
    #img.draw_image(mask, 0, 0)

    #框出矩形
    mask_gray = mask.to_grayscale()
    blobs = mask_gray.find_blobs([(255, 255)], area_threshold=100)
    if blobs:
        x_min = img.width()
        y_min = img.height()
        x_max = 0
        y_max = 0
        for blob in blobs:
            x, y, w, h = blob.rect
            if x < x_min: x_min = x
            if y < y_min: y_min = y
            if x + w > x_max: x_max = x + w
            if y + h > y_max: y_max = y + h
        overall_rect = (x_min, y_min, x_max - x_min, y_max - y_min)
        print("画框:", overall_rect)
        img.draw_rectangle(overall_rect, color=(255, 0, 0), thickness=4)

        sub_h = (y_max - y_min) // SUB_COUNT

        # 提取并打印六个颜色
        colors = []
        for i in range(SUB_COUNT):
            sub_y = y_min + i * sub_h
            center_x = x_min + (x_max - x_min) // 2
            center_y = sub_y + sub_h // 2
            color = get_color_name(center_x, center_y, img)
            colors.append(f"{i+1},{color}")


            # 画出中心点（小方块）
            img.draw_rectangle((center_x, center_y, 3, 3), color=(255, 0, 0), thickness=4)

        # 颜色转数字：绿→1，蓝→2，红→3
        color_to_num = {"GREEN": "1", "BLUE": "2", "RED": "3", "UNKNOWN": "0"}
        result = "".join([color_to_num.get(c.split(",")[1], "0") for c in colors])
        print(result)
        return result
    else:
        print("未检测到连通域")
        return "000000"
