# 临时诊断脚本：纯 YOLO 检测，不联网 / 不推流 / 不开串口
# 用途：区分"检测不到球"是【模型或摄像头本身的问题】还是【后来加的 RTSP/TCP 在干扰】
# 跑法：和 best.kmodel 一起拷到 SD 卡根目录运行（不需要 libs/WBCRtsp.py）
import time
import nncase_runtime as nn
import ulab.numpy as np
from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLOv8
import image
import os, sys
import gc

kmodel_path = "/sdcard/best.kmodel"
labels = ["ball"]
confidence_threshold = 0.3
nms_threshold = 0.45
model_input_size = [320, 320]
rgb888p_size = [320, 180]
send_to_ide = True
target_fps = 25
display_size = None
display_mode = "lcd"
sensor_fps = 30

pl = None
try:
    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size,
                  display_mode=display_mode)
    pl.create(fps=sensor_fps, to_ide=send_to_ide)
    display_size = pl.get_display_size()

    yolo = YOLOv8(task_type="detect", mode="video", kmodel_path=kmodel_path,
                labels=labels, rgb888p_size=rgb888p_size,
                model_input_size=model_input_size, display_size=display_size,
                conf_thresh=confidence_threshold, nms_thresh=nms_threshold,
                max_boxes_num=50, debug_mode=0)
    yolo.config_preprocess()
    print(">>> 模型加载完成，开始检测（把球放到镜头前）")

    frame_interval_ms = 1000 // target_fps
    t_stat = time.ticks_ms()
    n = 0
    t_print = time.ticks_ms()
    sec_cnt = 0          # 这1秒内出现过的框总数
    sec_max = 0.0        # 这1秒内的最高置信度
    while True:
        t0 = time.ticks_ms()
        img = pl.get_frame()
        res = yolo.run(img)

        # ---- 诊断统计：模型这一帧看到了什么 ----
        if res:
            c = len(res[0])
            if c > 0:
                sec_cnt += c
                s = max(res[2])
                if s > sec_max:
                    sec_max = s

        yolo.draw_result(res, pl.osd_img)
        pl.show_image()
        gc.collect()

        # 每秒打印一次：框数 + 最高分（最高分最关键）
        if time.ticks_diff(time.ticks_ms(), t_print) >= 1000:
            t_print = time.ticks_ms()
            print("1秒内: %d 个框, 最高分 %.3f" % (sec_cnt, sec_max))
            sec_cnt = 0
            sec_max = 0.0
            n += 1

        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        if elapsed < frame_interval_ms:
            time.sleep_ms(frame_interval_ms - elapsed)
except KeyboardInterrupt:
    print("user stop")
except Exception as e:
    sys.print_exception(e)
finally:
    if pl:
        pl.destroy()
