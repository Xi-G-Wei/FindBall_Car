# K230D 乒乓球检测 —— YOLOv8n 320x320
# 【第 3 版】在第 2 版基础上加：WiFi 连接（自检版）
#   第 1 版：YOLO 检测 → 发坐标给 STM32
#   第 2 版：指令通道（send_cmd / CMD_*）
#   第 3 版：WiFi          ← 当前
#   后面：TCP 连后端 → 收到 move 转成指令 → 加推流
# 含 RTSP/TCP 的完整版存在 main_full_backup.py，别删
import time
import nncase_runtime as nn
import ulab.numpy as np
from libs.PipeLine import PipeLine, ScopedTiming
from libs.YOLO import YOLOv8
import image
import os, sys
import gc
from machine import UART
from machine import FPIOA
import network
import socket,json

def dump_exc(e):
    # 本固件的 sys 模块没有 print_exception，做个兜底
    # 否则"报错时打印错误"会二次崩溃，把无害的失败变成程序猝死
    try:
        sys.print_exception(e)
    except:
        print("EXCEPTION:", repr(e))

# ---------------- 配置 ----------------
kmodel_path = "/sdcard/best.kmodel"   # kmodel 路径
labels = ["ball"]                     # 类别名，和训练时 data.yaml 一致
confidence_threshold = 0.2            # 置信度阈值（0.3→0.2：框闪的主因是分数在阈值线上飘；
                                      #   实测干扰物最高 0.153，降到 0.2 仍有安全余量。
                                      #   误检变多就往回加到 0.25）
nms_threshold = 0.45
model_input_size = [320, 320]         # 和训练/编译时 imgsz 一致

# 降温三件套 ------------------------------------------------
rgb888p_size = [320, 180]             # AI 采集（480x270→320x180；球看不清就改回 [480,270]）
send_to_ide = True                    # True=往 IDE 传预览；设 False 可省 CPU/USB 降温
target_fps = 25                       # 软件限帧，降 KPU 占空比
# ----------------------------------------------------------

display_size = None                   # None=自动取 LCD 真实分辨率(800x480)
display_mode = "lcd"                  # 用 HDMI 显示器就改成 "hdmi"

sensor_fps = 30
# --------------------------------------

# ---------------- 串口上报给 STM32 ----------------
# 115200 8N1，TX=IO3，RX=IO4（官方 uart1.py / ai_uart.py 用的就是这两个脚）
# x/y 是球框中心在【显示坐标】下的像素值（800x480 就是 0~799 / 0~479）
#
# K230 只说"球在哪"，不说"该往哪走"：
#   有球   → "x,y\n"     STM32 算横向偏差，决定弧线 / 直行
#   没球   → 什么都不发   STM32 按"多久没收到坐标"自己决定 冲 / 停 / 转圈找
#
#  丢失判断只在 STM32 那一侧做。这边**不要**再算一套"丢球了几秒"，
#    两边各有一套时间必然打架。
uart_tx_pin = 3
uart_rx_pin = 4
uart_baud = 115200
DEBUG_UART = False    # True=每秒把实际发出的坐标打印出来（要查串口时临时打开）
MIN_BOX_PX = 20       # 框宽小于这么多像素就当噪点丢掉（实测出现过 6px/15px 的噪点框）
LOCK_RADIUS_PX = 100  # "锁定"判定半径：新框中心离上次锁定的位置小于这个距离，就认作同一颗球。
                      # 调大 → 咬得死，但两颗球靠很近时可能一直咬着错的那颗
                      # 调小 → 换目标快，但球滚得快时容易跟丢、频繁重新选

# ---------------- 遥控指令（和 STM32 的 USART3.c 一一对应）----------------
#    动手的五个是小写字母，切模式的两个是大写
CMD_FWD    = "Mf"    # 前进
CMD_BACK   = "Mb"    # 后退
CMD_LEFT   = "Ml"    # 左转
CMD_RIGHT  = "Mr"    # 右转
CMD_STOP   = "Ms"    # 停
CMD_MANUAL = "MM"    # 切【手动】模式（之后 STM32 才接受上面的动作指令）
CMD_AUTO   = "MA"    # 切【自动】模式（车自己追球）

# ---------------- WiFi ----------------
# 总开关：调不联网的部分时设 False，整块跳过，回到最稳的地基
ENABLE_NET    = True
WIFI_SSID     = "你的热点名"            # 照抄 main_full_backup.py 里的值，和手机设置一字不差
WIFI_PASSWORD = "你的热点密码"

#---------------后端TCP----------------
SERVER_IP="后端电脑的IP"
TCP_PORT=9999
ROBOT_TOKEN="后端给的token"
#掉线重连间隔  每connect一次阻塞1秒
RECONNECT_MS=5000

def send_cmd(c):
    """把一条指令写进串口。必须以 '\n' 结尾 —— STM32 是按行攒够才解析的"""
    uart.write((c + "\n").encode())

#----------------全局变量-----------------------
network_ip = None                 # 连上 WiFi 后填，后面 TCP 那儿要用
tcp_s=None      #后端TCP连接  None=没链接上
tcp_buf=b""     #收包缓冲  TCP字节流 要求攒出一行
t_hb=0          #上一次心跳时刻
cur_fps=0.0     #最近一次算出的fps  心跳里上报后端
t_reconnect=0   #上一次重连的时刻
pl = None

try:
    # 打开串口：FPIOA 把物理引脚"路由"到 UART1 的功能上
    # 不设的话引脚还是默认功能，串口发不出东西
    fpioa = FPIOA()
    fpioa.set_function(uart_tx_pin, fpioa.UART1_TXD, ie=1, oe=1)
    fpioa.set_function(uart_rx_pin, fpioa.UART1_RXD, ie=1, oe=1)
    uart = UART(UART.UART1, baudrate=uart_baud, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
    print("UART1 ready: TX=IO%d RX=IO%d %d" % (uart_tx_pin, uart_rx_pin, uart_baud))

    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size,
                  display_mode=display_mode)
    pl.create(fps=sensor_fps, to_ide=send_to_ide)   # fps 必须是 30
    display_size = pl.get_display_size()            # 取 LCD 真实分辨率

    yolo = YOLOv8(task_type="detect", mode="video", kmodel_path=kmodel_path,
                labels=labels, rgb888p_size=rgb888p_size,
                model_input_size=model_input_size, display_size=display_size,
                conf_thresh=confidence_threshold, nms_thresh=nms_threshold,
                max_boxes_num=50, debug_mode=0)
    yolo.config_preprocess()

    # ---- WiFi 连接----
    # 放在 YOLO 初始化之后：这样 WiFi 挂了也不影响检测，车照样能跑
    if ENABLE_NET:
        try:
            wlan = network.WLAN(network.STA_IF)

            # 板子会自动回连上次的 WiFi —— 先断开，否则 connect 可能直接失败
            try:
                wlan.disconnect()
                t = time.ticks_ms()
                while wlan.isconnected():
                    if time.ticks_diff(time.ticks_ms(), t) > 3000:
                        break
                    time.sleep_ms(100)
            except:
                pass
            time.sleep_ms(500)               # 断开后稍等，立刻 connect 容易失败

            wlan.connect(WIFI_SSID, WIFI_PASSWORD)

            #这个超时是必须的：热点没开的话，没有它程序会永远卡死在这儿
            # isconnected() 为 False 不等于"没关联上" —— 也可能关联了但 DHCP 没拿到 IP
            # 连不上时把下面两行临时打开，看看板子到底扫得到哪些热点：
            # for ap in wlan.scan(): print("   ", ap)
            t = time.ticks_ms()
            while not wlan.isconnected():
                if time.ticks_diff(time.ticks_ms(), t) > 15000:
                    # 失败时把现场打出来。不然只看到一句 timeout 没法查
                    print("wifi status=%s ifconfig=%s"
                          % (wlan.status(), wlan.ifconfig()))
                    raise OSError("wifi timeout")
                time.sleep_ms(200)

            time.sleep_ms(500)               # 等 DHCP 拿稳 IP
            network_ip = wlan.ifconfig()[0]
            print("WiFi OK, IP =", network_ip)

            #----------连接后端TCP------------
            #在链接WiFi后  一定是链接WiFi成功才能链接后端
            try:
                tcp_s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                tcp_s.settimeout(1)     #1秒后超时

                #开始连接
                tcp_s.connect((SERVER_IP,TCP_PORT))
                tcp_s.setblocking(False)

                hello={"t":"hello","dev":"k230-01","ver":"1.0",
                       "ip":network_ip,"token":ROBOT_TOKEN
                }

                tcp_s.send((json.dumps(hello)+"\n").encode())
                print("[%d] TCP hello -> %s:%d" %(time.ticks_ms(),SERVER_IP,TCP_PORT))
            except Exception as et:
                dump_exc(et)
                print("tcp connect failed,car still running")
                try:
                    tcp_s.close()
                except:
                    pass
                tcp_s=None

        except Exception as e:
            dump_exc(e)
            print("WiFi failed, YOLO still running")

    frame_interval_ms = 1000 // target_fps   # 40ms/帧
    n = 0
    t_stat = time.ticks_ms()
    t_send = time.ticks_ms()
    t_reconnect=time.ticks_ms()
    t_dbg = 0                        # 上次打印串口调试信息的时刻
    lock_cx = -1                     # 上次锁定那颗球的中心；-1 = 当前没锁定
    lock_cy = -1

    while True:
        t0 = time.ticks_ms()                 # 记录帧开始时间
        img = pl.get_frame()                 # 取一帧
        res = yolo.run(img)                  # 推理

        # ---- 从检测结果里挑一颗球上报（带"锁定"）----
        # res = (框列表, 类别id列表, 分数列表)
        # 每个框是 (左上x, 左上y, 宽, 高)，注意是【左上角+宽高】，不是中心
        #
        # 为什么要锁定：协议一行只发一对坐标，STM32 一次只能追一颗，所以 K230 必须替它选。
        # 但"每帧重新选"会导致两颗球差不多大时来回跳 → 主控收到左右摇摆的指令 → 车在两球之间摆。
        # 所以：认准一颗就咬住，直到它从画面里消失，才换下一颗。
        best_i = -1
        if res:                              # 没检测到球时 res 是空元组，len(res[0]) 会直接崩
            # ① 先找"离上次锁定位置最近"的框 —— 够近就认作同一颗，继续跟它
            if lock_cx >= 0:
                best_d = LOCK_RADIUS_PX
                for i in range(len(res[0])):
                    if res[0][i][2] < MIN_BOX_PX:      # 太小的框当噪点丢掉
                        continue
                    dx = res[0][i][0] + res[0][i][2] / 2 - lock_cx
                    dy = res[0][i][1] + res[0][i][3] / 2 - lock_cy
                    d = (dx * dx + dy * dy) ** 0.5
                    if d < best_d:
                        best_d = d
                        best_i = i

            # ② 没锁上（刚开机，或锁的那颗被收进框、消失了）→ 按【面积最大】重新选一颗，
            #    也就是最近的，先把近的收掉
            if best_i < 0:
                best_area = 0
                for i in range(len(res[0])):
                    bw = res[0][i][2]
                    if bw < MIN_BOX_PX:
                        continue
                    area = bw * res[0][i][3]
                    if area > best_area:
                        best_area = area
                        best_i = i

        if best_i >= 0:
            bx, by, bw, bh = res[0][best_i]
            cx = int(bx + bw / 2)            # 框中心，得自己算
            cy = int(by + bh / 2)
            lock_cx = cx                     # 记住这一颗，下一帧优先继续跟它
            lock_cy = cy
        else:
            cx = -1
            cy = -1
            lock_cx = -1                     # 球从画面消失 → 解除锁定，下一帧重新选
            lock_cy = -1

        yolo.draw_result(res, pl.osd_img)    # 画框到 OSD 层
        if cx >= 0:                          # 标出实际要上报的那个点，标定 AIM_X 时看这个绿圈
            pl.osd_img.draw_circle(cx, cy, 8, color=(0, 255, 0), thickness=3)
            pl.osd_img.draw_string_advanced(cx + 10, cy, 28, "%d,%d" % (cx, cy),
                                            color=(0, 255, 0))
        pl.show_image()                      # 显示

        # ---- 上报：有球发一行坐标，没球什么都不发 ----
        # 节流到一帧一次（主循环本来就是 40ms/帧，等于每帧发一次）
        if time.ticks_diff(time.ticks_ms(), t_send) >= frame_interval_ms:
            t_send = time.ticks_ms()
            if cx >= 0:
                msg = ("%d,%d\n" % (cx, cy)).encode()
                uart.write(msg)
                if DEBUG_UART and time.ticks_diff(time.ticks_ms(), t_dbg) >= 1000:
                    t_dbg = time.ticks_ms()
                    print("UART out:", msg)
        
        #---------------接收后端下发数据-----------------
        #非阻塞
        if tcp_s:
            try:
                chunk=tcp_s.recv(256)
            except OSError:
                chunk=None
            # ⚠️⚠️ 这个固件的非阻塞 recv() 在"暂时没数据"时返回 b''，
            #      和"对端关闭"的返回值【完全一样】，分不出来 ——
            #      所以【绝对不能】拿 b'' 当断线判据。
            #      （实测踩过：这么写会每 40ms 把自己踢下线一次，连接活不过一帧，
            #        现象是"连上→立刻被关→重连"的死循环，看起来像后端在关我们）
            #      掉线改由【心跳发送失败】来发现，见下面心跳那一段。
            if chunk:
                tcp_buf+=chunk
                while b"\n" in tcp_buf:
                    line,tcp_buf=tcp_buf.split(b"\n",1)
                    try:
                        #解码
                        obj=json.loads(line.decode())
                    except:
                        #解不出来跳过本轮
                        continue
                    print("[%d] recv: %s" % (time.ticks_ms(), obj))
        elif ENABLE_NET and network_ip and\
            time.ticks_diff(time.ticks_ms(),t_reconnect)>=RECONNECT_MS:
            t_reconnect=time.ticks_ms()
            try:
                tcp_s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                tcp_s.settimeout(1)     #限制1秒超时
                tcp_s.connect((SERVER_IP,TCP_PORT))
                tcp_s.setblocking(False)

                #重连清除旧缓冲
                tcp_buf=b""
                hello={"t":"hello","dev":"k230-01","ver":"1.0",
                        "ip":network_ip,"token":ROBOT_TOKEN
                }

                tcp_s.send((json.dumps(hello)+"\n").encode())
                print("[%d] tcp reconnected" % time.ticks_ms())
                t_hb=time.ticks_ms()
            except Exception:
                try:
                    tcp_s.close()
                except:
                    pass
                tcp_s=None
        #------------心跳(每秒一次)----------
        if tcp_s and time.ticks_diff(time.ticks_ms(),t_hb)>=1000:
            t_hb=time.ticks_ms()
            print("[%d] hb" % time.ticks_ms())
            try:
                tcp_s.send((json.dumps({
                    "t":"status",
                    "fps":int(cur_fps),
                    "detected":len(res[0]) if res else 0,
                    })+"\n").encode())
            except OSError:
                #发不出去=连接坏了  标记掉线  准备重连 
                #心跳只有 1 秒一条、几十字节，缓冲区满（EAGAIN）基本不可能
                print("tcp send failed")
                try:
                    tcp_s.close()
                except:
                    pass
                tcp_s=None        
        gc.collect()
        # 限帧：本帧处理不足 40ms 就睡到 40ms
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        if elapsed < frame_interval_ms:
            time.sleep_ms(frame_interval_ms - elapsed)

        # 统计真实帧率
        # 打印频率：每 200 帧一次约 8 秒
        n += 1
        if n % 200 == 0:
            dt = time.ticks_diff(time.ticks_ms(), t_stat)
            if dt > 0:
                cur_fps=n*1000.0/dt     #存下来用于心跳
                
                print("fps = %.1f" % cur_fps)
            n = 0
            t_stat = time.ticks_ms()

except KeyboardInterrupt:
    print("user stop")
except Exception as e:
    dump_exc(e)
finally:
    if pl:
        pl.destroy()

# ==================== 第 2 版的临时测试块（保留备查）====================
# 想单独验"K230 → STM32 的指令通道"时，把下面这段放回主循环里、
# 同时把坐标上报的 uart.write(msg) 注释掉（一轮循环只能发一行，别让两者抢串口）
#
#     TEST_SEQ = [CMD_MANUAL, CMD_FWD, CMD_STOP, CMD_RIGHT, CMD_STOP, CMD_AUTO]
#     test_i = 0
#     t_test = time.ticks_ms()
#
#     # ---- 主循环里，gc.collect() 之前 ----
#     if time.ticks_diff(time.ticks_ms(), t_test) >= 3000:
#         t_test = time.ticks_ms()
#         c = TEST_SEQ[test_i % len(TEST_SEQ)]
#         test_i += 1
#         print("test cmd ->", c)
#         send_cmd(c)
# ======================================================================
