# test_robot_tcp.py —— 板子↔后端 TCP 链路测试
import network, socket, json, time, os

SSID, PASSWORD = "你的热点名", "你的热点密码"
SERVER_IP   = "后端电脑的IP"    # 改成后端（或假后端电脑）的IP
SERVER_PORT = 9999
TOKEN       = "后端给的token"        # 联调前换成后端给的

os.exitpoint(os.EXITPOINT_ENABLE)

def wifi_connect():
    wlan = network.WLAN(network.STA_IF)
    # 板子会自动回连上次的WiFi；且connect是异步的，切换瞬间可能读到旧IP。
    # 所以：先显式断开 → 再连接 → 连上后等DHCP稳定
    try:
        wlan.disconnect()
        t = time.ticks_ms()
        while wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t) > 3000:
                break
            time.sleep_ms(100)
    except:
        pass
    wlan.connect(SSID, PASSWORD)
    t = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), t) > 10000:
            print("wifi connect timeout")
            return None          # 10秒连不上就放弃，外层3秒后重来
        time.sleep_ms(200)
    time.sleep_ms(500)           # 等DHCP把IP拿稳
    return wlan.ifconfig()[0]

# ===== 外层循环：负责连 =====
while True:
    s = None
    try:
        network_ip = wifi_connect()
        if network_ip is None:
            raise OSError("wifi failed")
        print("wifi ok:", network_ip)

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((SERVER_IP, SERVER_PORT))
        s.setblocking(False)
        print("tcp connected")

        hello = {"t":"hello","dev":"k230-01","ver":"1.0","ip":network_ip,"token":TOKEN}
        msg = (json.dumps(hello) + "\n").encode()
        print("send:", repr(msg))      # 看末尾有没有 \n
        s.send(msg)
        t0 = time.ticks_ms()           # hello发出时刻，测FIN何时到

        buf = b""                      # 接收缓冲区，循环外定义一次
        t_hb = time.ticks_ms()         # 上次心跳时间

        # ===== 内层循环：负责活 =====
        while True:
            os.exitpoint()

            now = time.ticks_ms()
            if time.ticks_diff(now, t_hb) >= 1000:
                t_hb = now
                s.send((json.dumps({"t":"status","mode":"manual","fps":0}) + "\n").encode())

            try:
                chunk = s.recv(1024)
            except OSError:
                chunk = None

            if chunk == b"":           # 空字节串 = 对端关闭
                print("server closed after %d ms" % time.ticks_diff(time.ticks_ms(), t0))
                break
            elif chunk:
                print("recv:", repr(chunk))   # 原样打印，错误帧也不漏
                buf += chunk

            while b"\n" in buf:        # \n 才是消息边界
                line, buf = buf.split(b"\n", 1)
                try:
                    obj = json.loads(line.decode())
                except:
                    continue
                if obj.get("t") == "move":
                    print("move:", obj["dir"])
                elif obj.get("t") == "hello" and obj.get("ok"):
                    print("hello ok")

            time.sleep_ms(20)

    except KeyboardInterrupt:
        raise                          # 用户停止就直接退出，不重连
    except Exception as e:
        import sys
        sys.print_exception(e)
    finally:
        if s:
            try: s.close()
            except: pass

    print("reconnect in 3s...")
    time.sleep(3)
