import rospy
import numpy as np

# getch function is copied form
# https://stackoverflow.com/questions/27750536/python-input-single-character-without-enter
def getch():
    import termios
    import sys, tty
    def _getch():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    return _getch()

# speed=0
# angle=0

print()
print("Use w, a, s, d to move the turtle. press q to exit")
print()
# while not rospy.is_shutdown():
def getdata(speed=0, angle=0):
    a=getch()
    if a=="w":
        speed+=1
        # rospy.loginfo("w")
    elif a=="s":
        speed-=1
        # rospy.loginfo("s")
    elif a=="a":
        angle+=np.pi/180
        # rospy.loginfo("a")
    elif a=="d":
        angle-=np.pi/180
        # rospy.loginfo("d")
    elif a==" ":

        print("space")
    elif a=="q":
        rospy.loginfo("script ended")
        exit()
    return speed, angle
    rospy.sleep(0.1)