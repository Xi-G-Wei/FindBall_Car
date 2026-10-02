import network, time
from media.sensor import *
from media.vencoder import *
from media.media import *
import multimedia as mm
import os,uctypes

#连接WiFi
wlan = network.WLAN(network.STA_IF)     # STA = 当客户端，连手机热点
wlan.active(True)
wlan.connect("你的热点名", "你的热点密码")
while not wlan.isconnected():           # connect是异步的，轮询等它连上
    time.sleep_ms(200)
network_ip = wlan.ifconfig()[0]         # ifconfig返回(IP,掩码,网关,DNS)，[0]取IP
print("RTSP:rtsp://%s:8554/video" % network_ip)

#配置摄像头参数
sensor=Sensor()
sensor.reset()
sensor.set_framesize(width=640,height=352,alignment=12)
sensor.set_pixformat(Sensor.YUV420SP)

#配置编码参数
encoder=Encoder()
encoder.SetOutBufs(8,640,352)
chnAttr=ChnAttrStr(Encoder.PAYLOAD_TYPE_H264,Encoder.H264_PROFILE_MAIN,
                   640,352,
                   bit_rate=512,gopLen=20)
encoder.Create(chnAttr)

link=MediaManager.link(sensor.bind_info()['src'],
                       (VIDEO_ENCODE_MOD_ID,VENC_DEV_ID,encoder.chn))
#开启编码器和摄像头
encoder.Start()
sensor.run()

#配置RTSP服务器
rtspserve=mm.rtsp_server()
#绑定端口
rtspserve.rtspserver_init(8554)
rtspserve.rtspserver_createsession("video",
                                   mm.multi_media_type.media_h264,
                                   False)
rtspserve.rtspserver_start()
try:
    streamData=StreamData()
    while True:
        os.exitpoint()
        encoder.GetStream(streamData)
        for i in range(streamData.pack_cnt):
            data=bytes(uctypes.bytearray_at(streamData.data[i],
                                            streamData.data_size[i]))
            rtspserve.rtspserver_sendvideodata("video",data,
                                            streamData.data_size[i],1000)
        encoder.ReleaseStream(streamData)
except KeyboardInterrupt:
    print("User Stop")
finally:
    #反序关闭
    sensor.stop()
    link.destroy()
    encoder.Stop()
    encoder.Destroy()
    rtspserve.rtspserver_stop()
    rtspserve.rtspserver_deinit()
    print("stopped")
