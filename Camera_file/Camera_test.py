import cv2
import time

xmin, xmax = 280, 1000
ymin, ymax = 0, 720

#exposuretimerange="34000     358733000"  gainrange="10 16"
#wbmode=1 saturation=1.0 gainrange="1 22" ispdigitalgainrange="2 5"\exposuretimerange="60000000 80000000" exposurecompensation=1.5 aelock=false ispdigitalgainrange="2 8" gainrange="3 8" ispdigitalgainrange="2 16" 12500000 
#wbmode=3 saturation=1.0 exposuretimerange="125000000 142000000" exposurecompensation=1.0 aelock=true gainrange="120000 150000" ispdigitalgainrange="5 15"
#GST_STR = 'nvarguscamerasrc saturation=1.5 ispdigitalgainrange="2 8" gainrange="3 8"\
GST_STR = 'nvarguscamerasrc saturation=2.0 gainrange="8 20" gainrange="3 8"\
        ! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)60/1 \
        ! nvvidconv ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx \
        !videobalance  \
        ! videoconvert \
        ! appsink max-buffers=1 drop=True'
	
def main():
    capture = cv2.VideoCapture(GST_STR, cv2.CAP_GSTREAMER)
    pretime = 0
    try:
        while cv2.waitKey(1) < 0:
            _, frame = capture.read()
            cv2.imshow('result', frame[:,xmin:xmax])
            dt = time.perf_counter() - pretime
            pretime = time.perf_counter()
            print(f"{dt:.3} {1/dt:.3}")
        
    except KeyboardInterrupt:
        pass
    capture.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
	main() 
