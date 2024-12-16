

# license removed for brevity
import rospy
from std_msgs.msg import String
import serial  # 引用pySerial模組

COM_PORT = '/dev/ttyACM0'    # 指定通訊埠名稱
BAUD_RATES = 9600    # 設定傳輸速率
ser = serial.Serial(COM_PORT, BAUD_RATES)   # 初始化序列通訊埠
def talker():
    pub = rospy.Publisher('chatter_step', String, queue_size=100)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    while not rospy.is_shutdown():
         while ser.in_waiting:          # 若收到序列資料…
            data_raw = ser.readline()  # 讀取一行
            data = data_raw.decode()   # 用預設的UTF-8解碼
            pub.publish(data)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass