from pinpong.board import Board,Pin,Servo
import time


Board("uno").begin()

s1=Servo(Pin(Pin.D9))
s2=Servo(Pin(Pin.D10))

s1.write_angle(90)
s2.write_angle(90)

# s1.write_angle(90)
# time.sleep(1)
# s1.write_angle(91)
# time.sleep(1)
# s1.write_angle(90)
# time.sleep(1)
# s1.write_angle(89)
# time.sleep(1)
# s1.write_angle(90)
# time.sleep(1)