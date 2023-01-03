import threading
from scipy.spatial import distance
from imutils import face_utils
import imutils
import dlib
import cv2
import time
import RPi.GPIO as GPIO
from time import sleep
from gpiozero import InputDevice
import smbus
import serial
import pynmea2
import math
import board
import neopixel

def function1():
    def eye_aspect_ratio(eye):
        A = distance.euclidean(eye[1], eye[5])
        B = distance.euclidean(eye[2], eye[4])
        C = distance.euclidean(eye[0], eye[3])
        ear = (A + B) / (2.0 * C)
        return ear
        
    thresh = 0.25
    frame_check = 20
    detect = dlib.get_frontal_face_detector()
    predict = dlib.shape_predictor("models/shape_predictor_68_face_landmarks.dat")# Dat file is the crux of the code

    (lStart, lEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["left_eye"]
    (rStart, rEnd) = face_utils.FACIAL_LANDMARKS_68_IDXS["right_eye"]
    cap=cv2.VideoCapture(0)
    flag=0
    while True:
        ret, frame=cap.read()
        frame = imutils.resize(frame, width=450)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        subjects = detect(gray, 0)
        for subject in subjects:
            shape = predict(gray, subject)
            shape = face_utils.shape_to_np(shape)#converting to NumPy Array
            leftEye = shape[lStart:lEnd]
            rightEye = shape[rStart:rEnd]
            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)
            ear = (leftEAR + rightEAR) / 2.0
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            if ear < thresh:
                flag += 1
#                 print (flag)
                if flag >= 10:
                    print("tehlikeli")
#                     pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
#                     pixels1.fill((255, 255, 255))
#                     pixels1[0] = (0, 255, 0)
#                     pixels1[1] = (0, 255, 0)
#                     pixels1[2] = (0, 255, 0)
#                     pixels1[3] = (255, 255, 0)
#                     pixels1[4] = (255, 255, 0)
#                     pixels1[5] = (255, 255, 0)
#                     pixels1[6] = (255, 0, 0)
#                     pixels1[7] = (255, 0, 0)
                    #print ("Drowsy")
            else:
                flag = 0
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
    cv2.destroyAllWindows()
    cap.release() 

def function2():
    TRIG_PIN_1 = 17
    ECHO_PIN_1 = 27
    TRIG_PIN_2 = 23
    ECHO_PIN_2 = 24

    no_rain = InputDevice(22)
    isik = InputDevice(25)
    ses = InputDevice(18)
     

    # GPIO modülünü kullanıma hazır hale getirin
    GPIO.setmode(GPIO.BCM)

    # Trig ve Echo pinlerini input ve output olarak ayarlayın
    GPIO.setup(TRIG_PIN_1, GPIO.OUT)
    GPIO.setup(ECHO_PIN_1, GPIO.IN)
    GPIO.setup(TRIG_PIN_2, GPIO.OUT)
    GPIO.setup(ECHO_PIN_2, GPIO.IN)

    def distance(trig_pin, echo_pin):
        # Trig pinini LOW seviyesine ayarlayın
        GPIO.output(trig_pin, False)
        time.sleep(0.1)

        # Trig pinini HIGH seviyesine ayarlayın ve 10 µs süreyle açık tutun
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        # Echo pininin sinyal durumunu dinleyin
        pulse_start = time.time()
        while GPIO.input(echo_pin)==0:
            pulse_start = time.time()

        pulse_end = time.time()
        while GPIO.input(echo_pin)==1:
            pulse_end = time.time()

        # Ölçülen süreyi hızın yarısına çarpıp mesafeyi hesaplayın
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        distance = round(distance, 2)
        
        return distance



    while True:
        hizson=1
        port="/dev/ttyAMA0"
        ser=serial.Serial(port, baudrate=9600, timeout=0.5)

        newdata = ser.readline().decode('unicode_escape')
        if newdata[0:6] == "$GPRMC":
            data = pynmea2.parse(newdata)
            speed_knot = data.spd_over_grnd
            speed_kmh = speed_knot * 1.852  # 1 knot = 1.852 km/h
            print(f'Hız: {speed_kmh:.2f} km/h')
        hiz=35
        if hiz>=35 and hiz<45:
            hizson=(math.sin(math.radians(hiz))**5)*100
            print(hizson)
        elif hiz<35:
            hizson=0
            print(hizson)
        elif hiz>=45 and hiz<50:
            hizson=(math.sin(math.radians(hiz))**5.5)*100
            print(hizson)
        elif hiz>=50 and hiz<55:
            hizson=(math.sin(math.radians(hiz))**5.8)*100
            print(hizson)
        elif hiz>=55 and hiz<60:
            hizson=(math.sin(math.radians(hiz))**5.4)*100
            print(hizson)
        elif hiz>=60 and hiz<70:
            hizson=(math.sin(math.radians(hiz))**5.3)*100
            print(hizson)
        elif hiz>=70 and hiz<90:
            hizson=(math.sin(math.radians(hiz))**5)*100
            print(hizson)
        elif hiz>=90:
            hizson=120
            print(hizson)
        mesafe1 = distance(TRIG_PIN_1, ECHO_PIN_1)
        mesafe2 = distance(TRIG_PIN_2, ECHO_PIN_2)

        if mesafe1< mesafe2:
            if mesafe1 <=15 and mesafe1>=10:
                riskMesafe=math.log(8000,mesafe1)**4
                print(riskMesafe)
            elif mesafe1 >15 and mesafe1<=20:
                riskMesafe=math.log(9000,mesafe1)**4.1
                print(riskMesafe)
            elif mesafe1 >20 and mesafe1<=25:
                riskMesafe=math.log(9000,mesafe1)**4.2
                print(riskMesafe)
            elif mesafe1 >25 and mesafe1<=30:
                riskMesafe=math.log(9000,mesafe1)**4.3
                print(riskMesafe)
            elif mesafe1 >30 and mesafe1<=35:
                riskMesafe=math.log(9000,mesafe1)**4.4
                print(riskMesafe)
            elif mesafe1 >35 and mesafe1<=40:
                riskMesafe=math.log(9000,mesafe1)**4.5
                print(riskMesafe)
            elif mesafe1 <10 and mesafe1 >=1:
                riskMesafe=1500
                print(riskMesafe)
            elif mesafe1 >250:
                riskMesafe=0
                print(riskMesafe)
            else:
                riskMesafe=math.log(9000,mesafe1)**4.6
                print(riskMesafe)
        elif mesafe2< mesafe1:
            if mesafe2 <=15 and mesafe2>=10:
                riskMesafe=math.log(8000,mesafe2)**4
                print(riskMesafe)
            elif mesafe2 >15 and mesafe2<=20:
                riskMesafe=math.log(9000,mesafe2)**4.1
                print(riskMesafe)
            elif mesafe2 >20 and mesafe2<=25:
                riskMesafe=math.log(9000,mesafe2)**4.2
                print(riskMesafe)
            elif mesafe2 >25 and mesafe2<=30:
                riskMesafe=math.log(9000,mesafe2)**4.3
                print(riskMesafe)
            elif mesafe2 >30 and mesafe2<=35:
                riskMesafe=math.log(9000,mesafe2)**4.4
                print(riskMesafe)
            elif mesafe2 >35 and mesafe2<=40:
                riskMesafe=math.log(9000,mesafe2)**4.5
                print(riskMesafe)
            elif mesafe2 <10 and mesafe2 >=1:
                riskMesafe=1500
                print(riskMesafe)
            elif mesafe2 >250:
                riskMesafe=0
                print(riskMesafe)
            else:
                riskMesafe=math.log(9000,mesafe2)**4.6
                print(riskMesafe)
        elif mesafe1 == mesafe2:
            if mesafe1 <=15 and mesafe1>=10:
                riskMesafe=math.log(8000,mesafe1)**4
                print(riskMesafe)
            elif mesafe1 >15 and mesafe1<=20:
                riskMesafe=math.log(9000,mesafe1)**4.1
                print(riskMesafe)
            elif mesafe1 >20 and mesafe1<=25:
                riskMesafe=math.log(9000,mesafe1)**4.2
                print(riskMesafe)
            elif mesafe1 >25 and mesafe1<=30:
                riskMesafe=math.log(9000,mesafe1)**4.3
                print(riskMesafe)
            elif mesafe1 >30 and mesafe1<=35:
                riskMesafe=math.log(9000,mesafe1)**4.4
                print(riskMesafe)
            elif mesafe1 >35 and mesafe1<=40:
                riskMesafe=math.log(9000,mesafe1)**4.5
                print(riskMesafe)
            elif mesafe1 <10 and mesafe1 >=1:
                riskMesafe=1500
                print(riskMesafe)
            elif mesafe1 >250:
                riskMesafe=0
                print(riskMesafe)
            else:
                riskMesafe=math.log(9000,mesafe1)**4.6
    #                 print(riskMesafe)

    #         print(f"Sensor 1: {dist1} cm")
    #         print(f"Sensor 2: {dist2} cm")
        
        if no_rain.is_active == False:
    #             print("Yağmur yağıyor")
            ymr=1
        else:
    #             print("Yağmur yağmıyor")
            ymr=0
        if isik.is_active == False:
    #             print("ışık var")
            isk=1
        else:
    #             print("ışık yok")
            isk=0
        if ses.is_active == False:
    #             print("ses yok")
            ss=0
        else:
    #             print("ses var")
            ss=1
        
        if ymr ==1:
            yagmurRisk=15
            print(yagmurRisk)
        elif ymr==0:
            yagmurRisk=0
            print(yagmurRisk)
        if ss ==1:
            sesRisk=10
            print(sesRisk)
        elif ss ==0:
            sesRisk=0
            print(sesRisk)
        if isk ==1:
            isikRisk=10
            print(isikRisk)
        elif isk==0:
            isikRisk=0
            print(isikRisk)

        
        while True:
            
        
        revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
        bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)
    
        EARTH_GRAVITY_MS2   = 9.80665
        SCALE_MULTIPLIER    = 0.004
    
        DATA_FORMAT         = 0x31
        BW_RATE             = 0x2C
        POWER_CTL           = 0x2D
    
        BW_RATE_1600HZ      = 0x0F
        BW_RATE_800HZ       = 0x0E
        BW_RATE_400HZ       = 0x0D
        BW_RATE_200HZ       = 0x0C
        BW_RATE_100HZ       = 0x0B
        BW_RATE_50HZ        = 0x0A
        BW_RATE_25HZ        = 0x09
    
        RANGE_2G            = 0x00
        RANGE_4G            = 0x01
        RANGE_8G            = 0x02
        RANGE_16G           = 0x03
    
        MEASURE             = 0x08
        AXES_DATA           = 0x32
    
        class ADXL345:
    
            address = None
    
            def __init__(self, address = 0x53):        
                self.address = address
                self.setBandwidthRate(BW_RATE_100HZ)
                self.setRange(RANGE_16G)
                self.enableMeasurement()
    
            def enableMeasurement(self):
                bus.write_byte_data(self.address, POWER_CTL, MEASURE)
    
            def setBandwidthRate(self, rate_flag):
                bus.write_byte_data(self.address, BW_RATE, rate_flag)
    
            def setRange(self, range_flag):
                value = bus.read_byte_data(self.address, DATA_FORMAT)
    
                value &= ~0x0F;
                value |= range_flag;  
                value |= 0x08;
    
                bus.write_byte_data(self.address, DATA_FORMAT, value)
                

            def getAxes(self, gforce = False):
                bytes = bus.read_i2c_block_data(self.address, AXES_DATA, 6)
                    
                x = bytes[0] | (bytes[1] << 8)
                if(x & (1 << 16 - 1)):
                    x = x - (1<<16)
    
                y = bytes[2] | (bytes[3] << 8)
                if(y & (1 << 16 - 1)):
                    y = y - (1<<16)
    
                z = bytes[4] | (bytes[5] << 8)
                if(z & (1 << 16 - 1)):
                    z = z - (1<<16)
    
                x = x * SCALE_MULTIPLIER 
                y = y * SCALE_MULTIPLIER
                z = z * SCALE_MULTIPLIER
    
                if gforce == False:
                    x = x * EARTH_GRAVITY_MS2
                    y = y * EARTH_GRAVITY_MS2
                    z = z * EARTH_GRAVITY_MS2
    
                x = round(x, 4)
                y = round(y, 4)
                z = round(z, 4)
    
                return {"x": x, "y": y, "z": z}
    
        if __name__ == "__main__":

            adxl345 = ADXL345()
                
            axes = adxl345.getAxes(True)
            
            ivmex="   x = %.3fG" % ( axes['x'] )
            ivmey="   y = %.3fG" % ( axes['y'] )
            ivmez="   z = %.3fG" % ( axes['z'] )

            print (ivmex)
            print (ivmey)
            print (ivmez)
            if ivmex >= 0.330 and ivmex <0.400:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*7)**2
                print(ivmeOlcDeger)
            elif ivmex <= 0.350:
                ivmeOlcDeger=0
                print(ivmeOlcDeger)
            elif ivmex >= 0.400 and ivmex <0.500:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*7.7)**2
                print(ivmeOlcDeger)
            elif ivmex >= 0.500 and ivmex <0.600:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*8.5)**2
                print(ivmeOlcDeger)
            elif ivmex >= 0.600 and ivmex <0.700:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*9.2)**2
                print(ivmeOlcDeger)
            elif ivmex >= 0.700 and ivmex <0.800:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*10.2)**2
                print(ivmeOlcDeger)
            elif ivmex >= 0.800:
                ivmeOlcDeger=(math.sin(math.radians(ivmex*100))*11)**2
                print(ivmeOlcDeger)

            #ivme Sensörü y değeri 
            if ivmey >= 0.350 and ivmey <0.400:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*7)**2
                print(ivmeOlcDeger2)
            elif ivmey <= 0.350:
                ivmeOlcDeger2=0
                print(ivmeOlcDeger2)
            elif ivmey >= 0.400 and ivmey <0.500:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*7.7)**2
                print(ivmeOlcDeger2)
            elif ivmey >= 0.500 and ivmey <0.600:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*8.5)**2
                print(ivmeOlcDeger2)
            elif ivmey >= 0.600 and ivmey <0.700:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*9.2)**2
                print(ivmeOlcDeger2)
            elif ivmey >= 0.700 and ivmey <0.800:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*10.2)**2
                print(ivmeOlcDeger2)
            elif ivmey >= 0.800:
                ivmeOlcDeger2=(math.sin(math.radians(ivmey*100))*11)**2
                print(ivmeOlcDeger2)

    #sonuc=(riskMesafe*(math.log(riskMesafe,2)/5))+(hizson*(math.log(hiz,2)/10))+(ivmeOlcDeger*(math.log(ivmeOlcDeger,2)/5))+(ivmeOlcDeger2*(math.log(ivmeOlcDeger2,2)/5))+(riskKamera*1)+(yagmurRisk*1)+(isikRisk*1)+(sesRisk*1)

        toplam=isikRisk+sesRisk+yagmurRisk+riskMesafe+hizson+ivmeOlcDeger+ivmeOlcDeger2
        print("toplam",toplam)
        
        if toplam<=12.5:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
        elif toplam<=25:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
        elif toplam<=37.5:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
        elif toplam<=50:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
            pixels1[3] = (255, 255, 0)
        elif toplam<=62.5:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
            pixels1[3] = (255, 255, 0)
            pixels1[4] = (255, 255, 0)
        elif toplam<=75:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
            pixels1[3] = (255, 255, 0)
            pixels1[4] = (255, 255, 0)
            pixels1[5] = (255, 255, 0)
        elif toplam<=87.5:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
            pixels1[3] = (255, 255, 0)
            pixels1[4] = (255, 255, 0)
            pixels1[5] = (255, 255, 0)
            pixels1[6] = (255, 0, 0)
        elif toplam<=100:
            pixels1 = neopixel.NeoPixel(board.D12, 55, brightness=0.05)
            pixels1.fill((255, 255, 255))
            pixels1[0] = (0, 255, 0)
            pixels1[1] = (0, 255, 0)
            pixels1[2] = (0, 255, 0)
            pixels1[3] = (255, 255, 0)
            pixels1[4] = (255, 255, 0)
            pixels1[5] = (255, 255, 0)
            pixels1[6] = (255, 0, 0)
            pixels1[7] = (255, 0, 0)
        else:
            pass
    #     if sonuc >=60:
    #         print(sonuc,"Uyarı Risk Yüksek")
    #     elif sonuc < 60:
    #         print("sonuc: ",sonuc)
            
        # Ölçümler arasında bir süre bekleyin
        time.sleep(0.5)
# İki fonksiyonu ayrı ayrı thread olarak çalıştırın
thread1 = threading.Thread(target=function1)
thread2 = threading.Thread(target=function2)

# Thread'leri başlatın
thread1.start()
thread2.start()