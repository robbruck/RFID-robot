# program for Raspberry Pi 4, controlling a 2-wheeled robot with a camera,
# an rfid tag reader and a servo-movable magnet to pick up the tag and move it.
# data logging is performed to AWS EC2.
# Autor: Robert Bruckmeier, Munich
# Date: November 2020 - April 2021
# License: MIT
# more information: https://github.com/robbruck/RFID-robot
import socket
import picamera
import numpy as np
import time
import math
import threading
import sys
import struct
import serial
import cv2
from datetime import datetime
import copy

#support infrastructure ###############################################################################
debug=False
dev=False #used to deactivate code used for development
class Cont(object):
    """Container class. Defines object with some content at start, may be enriched with attributes later"""
    def __init__(self, **kwarg):
        for kw, content in kwarg.items():
            setattr(self, kw, content)
    def __repr__(self):
        return "Cont(" + ",".join([kw + "=" + repr(value) for kw, value in vars(self).items()]) + ")"
if debug:
    co=Cont(y=17)
    co.x=10
    assert(repr(co)==repr(Cont(y=17,x=10))) #both objects print the same but are not "==", as they are different objects. see https://stackoverflow.com/questions/1227121/compare-object-instances-for-equality-by-their-attributes-in-python
def pp(ex, n=1, showarray=True):
    """returns string of expression ex for pretty-printing, with floats rounded to n digits"""
    if type(ex)==list: return "[" + ", ".join([pp(x, n, showarray) for x in ex]) + "]"
    if type(ex)==dict: return "{" + ", ".join([pp(key, n, showarray) + ":" + pp(value, n, showarray) for key, value in ex.items()]) + "}"
    if type(ex)==tuple: return "(" + pp(ex[0],n,showarray) + ",)" if len(ex) == 1 else "(" + ", ".join([pp(x, n, showarray) for x in ex]) + ")"
    if type(ex)==type(Cont()): return "Cont(" + ", ".join([key + "=" + pp(value,n,showarray) for key, value in vars(ex).items()]) + ")"
    if type(ex)==type(np.array([1])):
        s1="np.array(" if showarray else ""
        s2=")" if showarray else ""
        return s1 +"[" + ", ".join([pp(x, n, False) for x in ex]) + "]" + s2
    if type(ex) in [float,np.float64,np.float32]: return str(round(ex,n))
    return repr(ex) #to handle quotes around strings correctly
if debug:
    assert(pp([1, 5.123, "x", b"y", (), (1,), (1, 2), {"a": 3, "b": True}]) == "[1, 5.1, 'x', b'y', (), (1,), (1, 2), {'a':3, 'b':True}]")
    assert(pp([[], [1], [1, 2], np.ones((2, 2))]) == '[[], [1], [1, 2], np.array([[1.0, 1.0], [1.0, 1.0]])]')
    assert(pp(Cont(a=0.123,b=""))=="Cont(a=0.1, b='')")
class Abort_error(Exception):
    """raised when robot cannot continue, e.g. when no target is found"""
    pass
def getcallername(levelsup=1):
    """returns name of calling function, levelsup levels up in call stack"""
    # noinspection PyUnresolvedReferences
    frame=sys._getframe() #caveat: calling a private function, may not work anymore after refactoring of sys
    for _ in range(levelsup+1):
        frame=frame.f_back
    return frame.f_code.co_name

#basic input/output to main server, i.e. for logging###########################################################
shutdowncalls=[]
def shutdown():
    """ shuts down all the elements used by this program, registered in shutdowncalls"""
    logthis("shutting down")
    for cmd in shutdowncalls:
        #print("executing "+str(cmd))
        cmd()
    print("done shutting down")
def fatal(errorcode):
    """exit from fatal error"""
    shutdown()
    sys.exit(errorcode)
class LogState(Cont):
    """collects and handles logging-related data"""
    def __init__(self, **kwarg):
        super().__init__(**kwarg)
    def __repr__(self):
        return super().__repr__()
    def close(self):
        self.logging = False
        self.connection = None
        self.client_socket = None
        return self
logstate=LogState().close()

#logging happens from 2 threads, the main thread and a background thread for regular state monitoring
#when using logstate.connection.write: always
# 1. ensure logstate.logging == True
# 2. put a loglock lock around all writes of one command + flush (protect from competing writes)
loglock = threading.Lock()   #from: https://stackoverflow.com/questions/11177018/python-socket-and-threads#:~:text=4%20Answers&text=Unfortunately%2CThe%20socket%20shared%20by,operate%20on%20with%20no%20lock.&text=If%20you%20wanna%20implement%20this,them%20through%20the%20only%20socket.
def writetostreamatomic(data):
    """thread-safe writing of data to logger"""
    if logstate.logging:
        with loglock:
            logstate.connection.write(data)
            logstate.connection.flush()
def formattedvalue(fmt, value):
    # import struct
    return struct.pack(fmt, value)
def writetexttostream(txt):
    """write text to logger"""
    if logstate.logging:
        bytestring = txt.encode("utf-8")
        #in next line: 2 is command of sending a string, then length of string then string
        data=formattedvalue("<L", 2) +  formattedvalue("<L", len(bytestring)) + bytestring
        writetostreamatomic(data)
def logthis(text):
    """double logging output to AWS and console"""
    print(text)
    writetexttostream(text)
def writeimagetostream(img):
    """transfer img to logger"""
    if logstate.logging:
        # 1 is write image command
        data=formattedvalue("<L", 1) + \
            formattedvalue("<L", img.shape[0]) + \
            formattedvalue("<L", img.shape[1]) + \
            formattedvalue("<L", img.shape[2])
        with loglock:
            logstate.connection.write(data)
            logstate.connection.write(img)
            logstate.connection.flush()
def stoplogging():
    """end logger"""
    if logstate.logging:
        logthis("ending logging.")
        data=formattedvalue("<L", 0)  # Write 0 to signal end, and allow server to stop
        writetostreamatomic(data)
        logstate.connection.close()
        logstate.client_socket.close()
        logstate.close()
def startlogging(waituntilstarted=True):
    """start logger"""
    client_socket = socket.socket()
    ip_aws="3.125.116.141" #my AWS EC2 instance
    portstart, portend = 8000, 8100
    waitingmessage=True
    ok=False
    while not ok:
        for port in range(portstart,portend):
            try:
                #todo 3 - up to now no probem. this can potentially block. If e.g. port 8000 was left "unattended", this will block
                client_socket.connect((ip_aws, port)) #select where to send to
                ok=True
                break
            except:
                pass
        else:
            if not waituntilstarted:
                print("no port found. could not start logging. proceeding without logging anyway...")
                logstate.close()
                return #ok to return from mid-method.
            else:
                if waitingmessage:
                    print("server port not found. Please start server. Waiting...",end="")
                    waitingmessage=False
                else: print(".",end="")
    #no-logging case exits via "return" above
    if not waitingmessage: print()
    connection = client_socket.makefile('wb')
    logstate.logging = True
    logstate.connection=connection
    logstate.client_socket=client_socket
    # noinspection PyUnboundLocalVariable
    print(f"connected to port {port}")
    logthis(f"connected to port {port}, started logging.")
    shutdowncalls.append(stoplogging)  # notice order: append to end, want to end logger last.
startlogging()

#rules for logging writers added via addlog: 1. any I/O (pins, logging) within lock 2. use writetexttostream, not logthis (no errors, but clutters it)
class Regularlog():
    """organizes background logging via a separate thread"""
    def __init__(self,delaytime=1): #background logging cycle delay time in seconds.
        self.delaytime=delaytime
        self.regularlogs=[]
        self.regularlogdriveractive=True
        self.t1 = threading.Thread(target=self.regularlogdriver)
        self.t1.start()
        shutdowncalls.insert(0, self.regularlogdriverstop)  # note order, stop early
    def addlog(self,target):
        self.regularlogs.append(target)
    def regularlog(self):
        for log in self.regularlogs:
            log()
    def regularlogdriver(self):
        while self.regularlogdriveractive:
            self.regularlog()
            time.sleep(self.delaytime) #not pause(), otherwise log output would appear
    def regularlogdriverstop(self):
        self.regularlogdriveractive=False
        self.t1.join()
reglog=Regularlog()
def regularlog(): reglog.regularlog()
def pause(seconds):
    caller=getcallername()
    if caller=="pausemotion": caller=getcallername(3)
    logthis(f"pause({seconds:5.2f}) called in {caller}") 
    time.sleep(seconds)
def pausemotion(seconds): #discriminates from communications-related pauses, could implement different behavior later on
    pause(seconds)

imagetable=[] #list of Cont(image=...,filename=...)
def imwrite(filename,image,loginfo=True): #logging and saving in RAM for performance reasons
    if loginfo: logthis(f"    image '{filename}' saved to RAM in {getcallername()}")
    imagetable.append(Cont(filename=filename, image=image))
def saveimages():
    for co in imagetable:
        cv2.imwrite(co.filename, co.image)
        logthis(f"    image '{co.filename}' saved to flash")
    imagetable.clear()
shutdowncalls.insert(0,saveimages)  # notice order: append to end, want to end logger last.

#power and voltage readout #################################################################################
powerlock = threading.Lock()
def logpower():
    """log Rpi power information"""
    with powerlock:
        from ina219 import INA219  #used only here
        SHUNT_OHMS = 0.05
        ina = INA219(SHUNT_OHMS)
        ina.configure()
        message=f"    Rpi power: {{voltage_V={ina.voltage():5.3f}, current_A={ina.current()/1000.0:6.4f}}}"
        writetexttostream(message)
reglog.addlog(logpower)

#camera/image handling ############################################################################################
#note: numpy uses BGR, while matplotlib uses RGB
logimagestoserver=0
logimagestoRPi=0
def getimagefromcamera(cam):
    output = np.empty((shapey, shapex, depth), dtype=np.uint8)
    cam.capture(output, 'bgr')
    #output=output[:,int(9/27.5*shapex)*0:,:]  #crop image
    if logimagestoserver:
        output=np.ascontiguousarray(output) #necessary to be able to .write it to stream. reference: https://stackoverflow.com/questions/29947639/cheapest-way-to-get-a-numpy-array-into-c-contiguous-order/29948246
        writeimagetostream(output)
    else: pass #logthis(f"image taken, but not logged due to logimagestoserver={logimagestoserver}")
    if logimagestoRPi: imwrite("imgs/current_img.jpg", output)
    return output
def initcamera():
    camera = picamera.PiCamera()
    shapey=480
    shapex=640
    depth=3
    assert(shapey % 16==0 and shapex % 32==0) #note that camera capture does some rounding otherwise, see: https://picamera.readthedocs.io/en/release-1.12/recipes2.html
    camera.resolution = (shapex, shapey)
    camera.framerate = 24
    camera.exposure_mode = "sports" #reduces blur due to robot motion
    # Start a preview and let the camera warm up for 2 seconds
    camera.start_preview()
    #print("stabilizing camera for 0.5s..."); pause(0.5)
    return camera,shapex,shapey,depth #make values global at main level. global does not fully work in console
camera,shapex,shapey,depth=initcamera() #shapex needed for getimagefromcamera... #no regularlogs or shutdowncalls needed
def pixtochess(pixelpos):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    mtx=np.array([[310.07007216,   0., 331.16770804],[  0., 321.54459426, 243.03416767],[0.,0.,1.]])
    dist=np.array([[-3.28214599e-01,  1.20159465e-01, -1.37039108e-04,6.81548978e-04, -2.06272092e-02]])
    objp = np.zeros((5 * 7, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:5].T.reshape(-1, 2)
    recalc=False
    if recalc:
        #position checkerboard printout at well-defined location and take some images when robot self-stabilizes
        for i in range(15):
            img = getimagefromcamera(camera)
            imwrite(f"chessboard_position_measurements/image_orig{i:>02}.jpg", img)
        #look at images and identify the one with ~average position (due to fluctuations in self-stabilization)
        fname="chessboard_position_measurements/image_orig10.jpg"
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, (7, 5), None)
        if ok!=True: print("chess board not found")
        else:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            # Find the rotation and translation vectors.
            ok, rvecs, tvecs, inliers  = cv2.solvePnPRansac(objp, corners2, mtx, dist)
            if ok!=True: print("calibration failed")
            else:
                print("rvecs=",pp(rvecs))
                print("tvecs=",pp(tvecs))
                #replace next two lines with this output to avoid recalculations
    else:
        rvecs = np.array([[-0.51056618], [-0.27566012], [-0.73814322]])
        tvecs = np.array([[-3.96573948], [-0.09738995], [6.523614]])
    # camera notation: https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/tr98-71.pdf
    guess=[0.,0.] #estimated coordinates in chessboard-space
    diff=1.0
    for _ in range(3):
        points = [[guess[0], guess[1], 0.], [guess[0]+diff, guess[1], 0.], [guess[0], guess[1]+diff, 0.]] #origin, x and y
        # noinspection PyUnboundLocalVariable
        imgpoints, _ = cv2.projectPoints(np.float32(points), rvecs, tvecs, mtx, dist)
        uvec=(imgpoints[1]-imgpoints[0])/diff  #in pixel-space
        vvec=(imgpoints[2]-imgpoints[0])/diff
        inv=np.linalg.inv(np.array([uvec[0],vvec[0]]).T) #uvec.shape=(1,2)
        pixeldiff=pixelpos-imgpoints[0][0]
        dguess=np.dot(inv,pixeldiff)
        guess+=dguess
    return guess #,np.linalg.norm(dguess) #2nd should be small to show convergence
def chesstocm(chesspos):
    xcm=3.75; ycm=3.60 #measured spacing values of chess printout that was photographed
    #chess board measured to start at cmpos (-9.9,7.1)
    x0=-9.9-4*xcm #0 chess position is 4th corner.
    y0=7.1
    return np.array([x0+chesspos[1]*xcm,y0+chesspos[0]*ycm]) #note indices, sorry using different axes notation/order
def apply_kmeans(K=10):
    #from: https://docs.opencv.org/master/d1/d5c/tutorial_py_kmeans_opencv.html
    #HLS: Hue Lightness Saturation. https://commons.wikimedia.org/wiki/File:HSL_color_solid_dblcone_chroma_gray.png
    i = 2
    img = getimagefromcamera(camera)
    imwrite(f"imgs/floor_{i}.jpg", img)
    Z = img.reshape((-1,3))
    Z = np.float32(Z)
    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    # Now convert back into uint8, and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))
    imwrite(f"imgs/kmeans{K}.jpg", res2)
if dev: apply_kmeans(K=10)
#todo 2: transfer from jupyter notebook to here now to measure camera distortion correction, see openCV-Python Tutorials Documentation, section 1.7.
if dev:
    for i in range(15):
        img=getimagefromcamera(camera)
        imwrite("chessboard/image_orig"+str(i)+".jpg", img)
        print("saved"+str(i))
        pause(5)
    for i in range(15):
        img=getimagefromcamera(camera)
        imwrite(f"chessboard_position_measurements/image_orig{i:>2}.jpg", img)
        print("saved"+str(i))
        pause(0)

#magnet/servo section ######################################################################################
import RPi.GPIO as GPIO #used only in this section
class Gpio_umbrella(): users=0
gpio_umbrella=Gpio_umbrella()
def gpio_increase():
    if gpio_umbrella.users==0:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)  # if previous run was stopped withour proper .close(), this line stops warning messages from sequential .setup() calls
    gpio_umbrella.users += 1
def gpio_reduce():
    gpio_umbrella.users -= 1
    if gpio_umbrella.users == 0:
        GPIO.cleanup()  # SimpleMFRC522 uses GPIO, not sure what to cleanup specifically

magnet_timeout=60 #in seconds, magnet will be dropped then
class Magnet():
    def __init__(self,magnetpin):
        self.magnetpin=magnetpin
        self.magnetstatus=0
        self.on_time=None
        self.magnetlock=threading.Lock()
        gpio_increase()
        GPIO.setup(self.magnetpin, GPIO.OUT)
        self.setmagnet(0)
        shutdowncalls.insert(0, self.close)  # added after Servo to keep right order (GPIO.cleanup)
        reglog.addlog(self.logmagnet)
    def setmagnet(self, active=1):
        with self.magnetlock:
            if self.magnetstatus==0 and active==1:
                self.on_time=time.time()
            onsince=f", was on for{time.time()-self.on_time:5.2f}s" if self.magnetstatus==1 and active==0 else ""
            logthis(f"setmagnet({active}) called"+onsince)
            GPIO.output(self.magnetpin,active)
            self.magnetstatus = active
    def logmagnet(self):
        if self.magnetstatus:
            with self.magnetlock:
                writetexttostream(f"    Rpi magnet: {self.magnetstatus}")
            if time.time()-self.on_time>magnet_timeout: #dont put this part in self.magnetlock, deadlock will occur. magnetlock handled in setmagnet
                self.setmagnet(0)
                writetexttostream(f"    Rpi magnet ERROR: turned off after timeout of {magnet_timeout}s.")
                #console output may be cluttered, but operation stays threadsafe.
    def close(self):
        self.setmagnet(0)
        GPIO.cleanup(self.magnetpin)  # better than generic GPIO.cleanup()
        gpio_reduce()
magnet=Magnet(magnetpin=11) #magnet control pin connected to servo motor
def setmagnet(active=1): magnet.setmagnet(active)
dc_up=2.8
dc_down=7.9
class Servo():
    def __init__(self,servopin):
        self.servopin=servopin
        self.dc_from=dc_up #should be the position in which the last run ended
        shutdowncalls.insert(0,self.close)  # notice order
        gpio_increase()
    def moveservo(self,dc_to,speed=100.0): #min is 2.5, max 12.5. must stop PWM otherwise servo will fluctuate/vibrate
        # naming: "dc" stands for duty cycle of PWM in percent. servo spec allows for 2.5-12.5 (in percent)
        dc_min,dc_max=2.8,8.5 #these are the mechanical limits of the magnet lever in the vehicle (less than pure servo limits)
        self.dc_from=max(dc_min,min(dc_max,self.dc_from))
        dc_to=max(dc_min,min(dc_max,dc_to))
        GPIO.setup(self.servopin, GPIO.OUT) #strangely, when this line is put in __init__, calling magnet.close() first causes error with servo.close()
        freq=50 #50 is the PWM repeat frequency as required by servo spec
        p = GPIO.PWM(self.servopin, freq) #"p: PWM" possible if "from RPi._GPIO import PWM" used, but uses private _GPIO
        steps=math.floor(0.99+abs(dc_to-self.dc_from)/5*30 * 100/speed)
        for i in range(1,steps+1):
            dutycycle=self.dc_from+(dc_to-self.dc_from)/steps*i
            # noinspection PyArgumentList
            (p.start if i == 1 else p.ChangeDutyCycle)(dutycycle)  #call the right function
            #time.sleep(0/freq) #increasing this does not improve wiggling
            if 1==1: #for pwmcycles in range(2): #repeating this does not help wiggling...
                #iteration number for a full pwm loop is ~20000
                for _ in range(50000):
                    if GPIO.input(self.servopin) == 1: break
                else:
                    logthis(f"ERROR: servo pin did not reach 1 in time. Unclear why. Could cause unlikely wild servo motion.")
                for _ in range(50000):
                    if GPIO.input(self.servopin) == 0: break
                else:
                    logthis(f"ERROR: servo pin did not reach 0 in time. Unclear why. Could cause unlikely wild servo motion.")
                time.sleep(1/freq*(1-(dutycycle/100))) #wait for end of PWM cycle, in order not to start the next PWM phase too early
            #time.sleep(0 / freq) #increasing this does not improve wiggling
        p.stop()
        GPIO.cleanup(self.servopin)
        if self.dc_from!=dc_to:
            logthis(f"servo moved from {self.dc_from} to {dc_to} at speed {speed}")
        self.dc_from=dc_to #save for next move
    def close(self):
        moveservoup() # using "self." would be an error here. Need to use wrapper.
        gpio_reduce()
servo=Servo(servopin=16) #PWM pin connected to servo motor
def moveservo(dc_to,speed=100.0): servo.moveservo(dc_to,speed)
def moveservoup(speed=100.0):
    x=1.2
    moveservo(dc_up,speed*x)
def moveservodown():
    x=1.2
    if servo.dc_from==dc_up:
        moveservo(dc_down*0.8+dc_up*0.2,100*x)
    pause(0.05) #lever does about 7 vibrations/s. 0.05 is a little less than half a vibration, dampening the vibration
    moveservo(dc_down,20*x)
def wavehand():
    moveservo(dc_up)
    delay=0.1
    for _ in range(2): time.sleep(delay); moveservo(dc_up+2); time.sleep(delay); moveservo(dc_up);
if dev:
    #below: code to analyze output of GPIO.PWM:
    GPIO.cleanup(16)
    GPIO.setup(16, GPIO.OUT)
    le=1000000
    #GPIO.input(16) is always 0 here
    p = GPIO.PWM(16, 50)
    #GPIO.input(16) is always 0 here
    p.start(4.0)
    #GPIO.input(16) risies quickly from 0 to 1. li2 sometimes catches the first rise, sometimes not
    li2 = [GPIO.input(16) for i in range(le)]
    p.stop()
    #GPIO.input(16) is always 0 here
    for li in [li2]:
        print(len(li),li[0],li[-1],sum([1 for x in li if x==0]),sum([1 for x in li if x==1]))
        rising=[i for i in range(len(li)-1) if li[i]==0 and li[i+1]==1]
        if rising[0]<10: rising=rising[1:] #li2 sometimes catches the first rise, sometimes not
        print(rising)
        falling=[i for i in range(len(li)-1) if li[i]==1 and li[i+1]==0]
        print(falling)
        for i in range(len(falling)-2): print(rising[i+1]-rising[i], end=" ")
        print()
        for i in range(len(falling)-2): print(falling[i+1]-rising[i],end=" ")
        print()


#RFID section ################################################################################################
def status_encode(status,repeats=10):
    """status: 0-9, repeats """
    out=""
    for i in range(repeats):
        out+=chr(ord("A")+status+i)+chr(ord("z")-status-i)
    return out
def status_decode(readout):
    errors=len(readout) % 2
    reads=[]
    for i in range(len(readout)//2):
        read1=ord(readout[i*2])-ord("A")-i
        read2=ord("z")-ord(readout[i*2+1])-i
        if read1==read2: reads+=[read1]
        else: errors+=1
    maxcount=-1
    maxread=-1
    for read in reads:
        count=len([1 for r in reads if r==read])
        if count>maxcount:
            maxcount=count
            maxread=read
    if maxcount<0: errors+=1
    else: errors+=len(reads)-maxcount
    return maxread,errors,maxcount,reads
from mfrc522 import SimpleMFRC522 #used only in this section
reader = SimpleMFRC522()
def rfid_there(): #center of the blue tag needs to be +- 2cm under the center of the reader
    #strangely, one of 2 calls is always nok
    id1,text1=reader.read_no_block()
    id2,text2=reader.read_no_block()
    return id1 or id2, text1 or text2 #returns ID or None,text or None
def rfid_write(status_nbr):
    id,_ = rfid_there()
    if id:
        text = status_encode(status_nbr)
        id2, text2 = reader.write_no_block(text)
        logthis(f"RFID write: {{id={id}, id2={id2}, text='{text}', text2='{text2}'}}")
        if id2:
            return id2
        else:
            logthis("ERROR: Tag initially found, but then could not write.")
            return None
    else:
        #print("Tag not close enough.")
        return None
def rfid_read(): #return decoded, e.g. (1, 0, 10, [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]), or None
    id,text = rfid_there()
    if id:
        text=text.rstrip()
        decoded=status_decode(text)
        logthis(f"RFID read: {{id={id}, status={decoded}, text='{text}'}}")
        return decoded #e.g. (1, 0, 10, [1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
    else:
        logthis("RFID tag not read")
        return None
if dev:
    rfid_read()
    rfid_write(0)
    rfid_write(1)
    for it in range(100): print(it,rfid_there())

#Arduino & balance section ######################################################################################
"""#Arduino serial output created by:
#Arduino start-up "\n" + ".started...\n"
#errors: always start with "!". angle>30, Checkfreemem, usmax, + diverse command line error responses,
#        i.e. "comma missing", addTbl, unknown key, unknown 2nd chars to n/y/k commands
#command line echo: sent if not muted due to E0 command. always starts with ">"
#command responses: single line responses for the following commands only: o, q/q, ... (see list below)
#usage from Rpi: set E0, filter startup/errors from data, pick up all responses syncronously (avoiding overlaps)"""
def checkresponses(cmd):
    cause_responses=["o","q","m","M","a","np","nP","nc","nC","nl","nt","nI"]
    twolettercommands_noresponses=["Im","If","It","ka","kg","kr","yp","yP","yd","yD"]
    pos=cmd.find("#")
    if pos>=0: cmd=cmd[:pos] #discard comments
    for twolett in twolettercommands_noresponses:
        cmd=cmd.replace(twolett,"")
    responses=[]
    for cause_response in cause_responses:
        if cmd.find(cause_response)>=0: responses.append(cause_response)
    return responses
if debug:
    assert(checkresponses("Im")==[]) #"m" not detected as single letter command
    assert(checkresponses("m")==["m"]) #"m" detected
    assert(checkresponses("#o")==[]) #comments working
    assert(checkresponses("q#o")==["q"]) #comments working
    assert(checkresponses(";")==[]) #others not detected
    assert(checkresponses("oq")==["o","q"]) #multiples shown
def print_all():
    for cmd in ["nt","nl","nc","nC","np","nP","nI"]:
        ans=cmd_ans(cmd)[-1]
        print(f'"{cmd}": {ans}')
communication=[] #all communication
data=[] #only lines starting with " " or "|"
def get_answer(timeout=2,serial=None,allowabort=True): #read all present lines (e.g. errors), and then stop if maxdatalines data lines have been read or timeout 1-s-timeouts have occured
    # check for errors with cmd_ans("o")
    if serial is None: serial=ser
    maxdatalines=1
    startlen=len(data)
    while (serial.inWaiting()>0) or (timeout>0 and maxdatalines>0): #inWaiting needed for error messages waiting to be picked up
        s=serial.readline().decode("UTF-8") #this can "hang" if within the timout period characters (e.g. ".") are repeatedly being sent, without sending a newline. This is a sender problem, not a recipient problem... Now avoided in Arduino
        if s=="": timeout-=1
        else:
            s=s.replace("\r","").replace("\n","")
            communication.append("recv:"+s)
            if (len(s)>0 and s[0]=="!"):
                print(s)  #always print errors (they start with "!")
                logthis("RPi ERROR message:"+s)
                if s.find("angle>30")>=0 and allowabort:
                    raise Abort_error
            if len(s)>0 and s[0] in ([" ","|"]):
                data.append(s)
                maxdatalines-=1
    return data[startlen:]
def cmd_send(cmd,nowarning=False,serial=None): #do not use for commands that send response. Use cmd_ans instead.
    if serial is None: serial=ser
    if not nowarning and checkresponses(cmd)!=[]:
        logthis(f"warning: use cmd_ans for command(s) '{checkresponses(cmd)}', unless explicitely picked up by get_answer()")
    serial.write((cmd+"\n").encode("UTF-8"))
    serial.flush()
    communication.append("send:"+cmd)
def cmd_ans(cmd, timeout=2,allowabort=True):
    if checkresponses(cmd)==[]:
        logthis(f"warning from cmd_ans: cmd='{cmd}' does not contain a command that is requesting an answer. not sending it.")
        return []
    else:
        cmd_send(cmd,nowarning=True)
        return get_answer(timeout=timeout,allowabort=allowabort)
def waitcheck_ok(timeout=1,allowabort=True):
    while (ser.inWaiting()>0): ans=ser.readline(); logthis(f"standup, read from serial: {ans}"); time.sleep(0.05)
    #logthis(f"     waitcheck_ok called in {getcallername()}")
    ans=cmd_ans("o",timeout=timeout+1,allowabort=allowabort)
    if ans != [" ok"]:
        logthis(f'  waitcheck_ok ERROR: answer to "o" is "{str(ans)}". communication[-10:]={pp(communication[-10:])}')
def standup():
    logthis(f"standup(): started at {datetime.now()}")
    # discard previous stuff, esp. "angle>30" errors etc
    waitcheck_ok(timeout=7,allowabort=False) #if anything is in the buffer, get it, log it and discard.
    time.sleep(0.05) #todo 3 errors are send after " ok", so pick up with 2nd try. refactor in Arduino
    waitcheck_ok(allowabort=False) #if anything is in the buffer, get it, log it and discard.
    cmd_send("e")
    cmd_send("E0;x1;P90;D1;i0.25;p50;j0.15;b0.03;d10")
    cmd_send("T1028.14;j4;r0.15;R15;V15;v5;A0.34;Im200;If100;It80;i0.4")
    ans=cmd_ans("a20") #quick angle measurement
    if len(ans[0])>0:
        ans=valuedict(ans[0])["angleDeviMeas_avg"] #leaning on "good" side returns ~-11.7, "bad" ~27
        if abs(ans)<15:
            cmd_send("s")
            waitcheck_ok()
            standup.active=True
            logthis("standup")
        else: logthis("standup ERROR: robot leaning too much")
    else: logthis(f"standup ERROR: no answer to 'a20' command, ans={str(ans)}")
    totalmotion.clear()
def comm(back=10):
    for strx in communication[-back:]: print(strx)
def valuedict(stri):
    strl=stri.split(" ")
    strl=[s for s in strl if s!=""] #happens if "  " (2 spaces) is in stri
    i=0
    while i<len(strl):
        if strl[i].find(":")==len(strl[i])-1: #":" is last char
            if i+1<len(strl): #if there is a next string...
                strl=strl[:i]+[strl[i]+strl[i+1]]+strl[i+2:] #combine (string ending with ":") with next string
        i+=1
    dic={}
    for st in strl:
        pos=st.find(":")
        if pos>0:
            try:
                dic[st[:pos]]=float(st[pos+1:])
            except ValueError:
                dic[st[:pos]]=st[pos+1:]
        else: dic[st]=None
    return dic
if debug:
    assert(valuedict('. 0:-1.1 a:x b: 1.5 y c:2.1') == {'.': None, '0': -1.1, 'a': 'x', 'b': 1.5, 'y': None, 'c': 2.1})
    assert(valuedict(" xpos:   20 ypos: -286 xyangle/deg:-361.00 ")=={'xpos': 20.0, 'ypos': -286.0, 'xyangle/deg': -361.0})
if dev:  # measure/calibrate angleVertical value, necessary after mechanical changes
    moveservoup()
    print("pls. check that cables are disconnected.")
    cmd_send("A0")  # clear current value
    pause(2)  # A0-command may cause robot to move, wait a bit to stabilize
    ans = cmd_ans("a5000", timeout=10)  # a command measures stable angle, number is measurement time in ms
    print("value to use in A-command:",-valuedict(ans[0])["angleDeviMeas_avg"] if len(ans[0]) > 0 else f"ERROR: ans={str(ans)}")
def interextrapolate(x,table):
    """ input: table: list of at least 2 (x,y) pairs with increasing x
        output: predicted y, linearly interpolated between points and linearly extrapolated outside"""
    assert(len(table)>=2)
    #find i with table[i][0] <= x < table[i+1][0], if it exists
    i=0
    while (i<len(table)-2) and (x>=table[i+1][0]): i+=1  # max(i)=len(table)-2
    x0,y0=table[i]
    x1,y1=table[i+1]
    return y0+(y1-y0)/(x1-x0)*(x-x0)
if debug:
    assert(interextrapolate( -1,((0,0),(1,1)))==-1)
    assert(interextrapolate(  0,((0,0),(1,1)))==0)
    assert(interextrapolate(0.5,((0,0),(1,1)))==0.5)
    assert(interextrapolate(  1,((0,0),(1,1)))==1)
    assert(interextrapolate(  2,((0,0),(1,1)))==2)
    assert(interextrapolate( -1,((0,0),(1,1),(2,0)))==-1)
    assert(interextrapolate(  0,((0,0),(1,1),(2,0)))==0)
    assert(interextrapolate(0.5,((0,0),(1,1),(2,0)))==0.5)
    assert(interextrapolate(  1,((0,0),(1,1),(2,0)))==1)
    assert(interextrapolate(1.5,((0,0),(1,1),(2,0)))==0.5)
    assert(interextrapolate(  2,((0,0),(1,1),(2,0)))==0)
    assert(interextrapolate(  3,((0,0),(1,1),(2,0)))==-1)
    assert(interextrapolate(2.5,((0,0),(1,1),(2,0),(3,2)))==1)
    assert(interextrapolate(3.5,((0,0),(1,1),(2,0),(3,2)))==3)
def laydown():
    if getattr(standup,"active",False):
        cmd_send("s;F-3000") #don't add "l" and wait until done
        pausemotion(0.2) #wait just a bit for robot to start moving a bit
        cmd_send("e") #and then stop, robot will always fall "forward"
        standup.active=False
        logthis("laydown")
def ar_shutdown():
    laydown()
    ser.flush()
    ser.close()
    logthis("ar_shutdown")
def open_serial():
    for device in ['/dev/ttyACM0','/dev/ttyAMA0']:
        try:
            ser = serial.Serial(device, 115200, timeout=1) #was 9600 originally
            break
        except serial.serialutil.SerialException:
            pass
    else: #no break was met, i.e. all devices were in error
        logthis("ERROR: could not open USB connection to Arduino")
        fatal(-1)
    # noinspection PyUnboundLocalVariable
    ser.flush()
    try:
        _=ser.inWaiting()
    except OSError:
        logthis("ERROR: Arduino does not respond")
        fatal(-1)
    shutdowncalls.insert(0, ar_shutdown)  # notice order
    cmd_send("o",nowarning=True,serial=ser)
    ans=get_answer(timeout=5,serial=ser)
    if (ans!=[" ok"] and communication[-1]!="recv:.started..."):
        logthis(f"WARNING: open_serial(): ans={str(ans)}, communication[-3:]={str(communication[-3:])}")
    return ser
ser=open_serial()
def printIvalues():
    ans1 = valuedict(cmd_ans("nC")[-1])
    posDevi = ans1["posDevi"]
    speedPID = ans1["speedPID"]
    ans2 = valuedict(cmd_ans("nI")[-1])
    Integral = ans2["Integral"]
    pwm = (ans1["pwm1"] + ans1["pwm2"]) / 2
    logthis(f"posDevi:{posDevi:>7},  Integral:{Integral:7.2f}, speedPID:{speedPID:6.2f}, pwm:{pwm:>5}")
#optimize control parameters
def sgn(x): return (x>0) - (x<0)
def waitdrive():
    start=time.time()
    cmd_send("l")
    waitcheck_ok(timeout=10) #wait time to complete driving
    return time.time()-start
def pose():
    """gets the pose of the robot. returns a Cont object with attributes xr, yr and degr, e.g. Cont(xr=1.2,yr=0.5,degr=17)"""
    logthis(f"pose() called in {getcallername()}")
    ans=cmd_ans("nc")
    if ans==[]: logthis("pose() raised communication error"); raise Abort_error #communication broken, typically happens ~1 Minute after startup of RPi/Arduino. Re-run python on RPi
    pos = valuedict(ans[-1])  # pose() is currently only used in dev sections. no need to handle errors
    co = Cont(xr=-pos["xpos"] / countspercm, yr=pos["ypos"] / countspercm, degr=-pos["xyangle/deg"])
    return co
countspercm=139.7
totalmotion=[] #record all moves, to allow going back
def forward(cm): #positive is opposite from RFID sensor
    logthis(f"forward({cm:5.1f}): called in {getcallername()}")
    totalmotion.append(("fwd", cm))
    dist=int(cm*countspercm)
    if dist != 0:  # otherwise problem in Arduino...
        cmd_send("F" + str(dist) + "l")
        waitdrive()
if dev:
    #measure hystheresis curve of moving forward & backward. This is how I discovered that both the surface roughness and soft wheel rubber mattered
    xy=[]
    pose()
    cmsum=0
    cm0=1
    delay=0
    for maxsteps in [5,5]: #range(1,5):
        for steps in range(maxsteps+1):
            forward(cm0)
            cmsum+=cm0
            pause(delay)
            xy.append((cmsum,round(pose().xr,2)))
        for steps in range(maxsteps+1):
            forward(-cm0)
            cmsum-=cm0
            pause(delay)
            xy.append((cmsum,round(pose().xr,2)))
        for steps in range(maxsteps+1):
            forward(-cm0)
            cmsum-=cm0
            pause(delay)
            xy.append((cmsum,round(pose().xr,2)))
        for steps in range(maxsteps+1):
            forward(cm0),
            cmsum+=cm0
            pause(delay)
            xy.append((cmsum,round(pose().xr,2)))
    print(xy) #printout analyzed in jupyter
countsperdeg=6475.0/360
def turn(degrees): #positive turns right
    logthis(f"turn({degrees:6.1f}): called in {getcallername()}")
    totalmotion.append(("trn", degrees))
    counts=int(-degrees*countsperdeg)
    if counts!=0: #otherwise problem in Arduino...
        cmd_send("C" + str(counts)) # + "l")
        waitdrive()
if dev:
    cmd_send("Im200") #up to 380 tested (>255!) and ok, 200 good to compensate drift of 4 cm
    cmd_send("If100") #at Im200 and It100, oscillations start at if300.
    cmd_send("It80")  #at Im200 and If200, oscillations start below 80, but only double even to 1
    for _ in range(50):
        print(datetime.now(),end="")
        printIvalues()
        pause(0.1)
def makemove(move,sign=1):
    if move[0] == "fwd":
        forward(move[1] * sign)
    elif move[0] == "trn": turn(move[1]*sign)
def reverse_motion():
    for move in totalmotion[::-1]:
        makemove(move,-1)
    totalmotion.clear()
def movetopose(frompose:Cont,topose:Cont):
    def anglenorm(angle_deg): angle=(angle_deg/360-math.floor(angle_deg/360)) * 360; return angle-360 if angle > 270 else angle #returns -90..270
    dx=topose.xr-frompose.xr
    dy=topose.yr-frompose.yr
    dist=math.sqrt(dx**2+dy**2)
    traveldir=math.atan2(dy,dx)*180/math.pi
    turn1=anglenorm(traveldir-frompose.degr)
    if turn1>90: dist*=-1; turn1-=180 #move backward
    turn(turn1)
    if abs(dist)>0.2: forward(dist)
    turn2=anglenorm(topose.degr-(frompose.degr+turn1))
    turn(turn2)
if dev:
    cmd_send("N")  # reset pose
    totalmotion.clear()
    turn(20)
    forward(-20)
    #reverse_motion()

#CV ###################################################################################################
color_lookup_BGR = {'red': np.array([0, 25, 255], dtype=np.uint8),  # was 19, 24, 98 - BGR!
                'green': np.array([135,255,0], dtype=np.uint8),  # was 67, 82, 10
                'yellow': np.array([0,219,248], dtype=np.uint8),  # was 103, 130, 210
                'blue': np.array([255,101,0], dtype=np.uint8)}  # was 82, 20,  0
save_analyzeimage_result_image=True
def analyzeimage(image, list_RGB_thresh=False, colors=None, calibrate_sat_thresh=False, minarea=10):
    """
    finds colored objects and returns their area & center coordinates
    :param image: BGR image
    :param list_RGB_thresh: boolean, scans RGB_thresh and stores images with different saturation thresholds
    :param colors: list of colors names that need to be analyzed
    :param calibrate_sat_thresh: boolean, scans sat_thresh and stores resulting images
    :param minarea: minimum area in pixels to identify an object
    :return: {color:(area,(y,x))}"""
    if colors is None: colors=color_lookup_BGR.keys()
    image = image & imgmask #label: "disable_imgmask"

    # first, transform to HSV space an make S=saturation extreme, then set V to S (so create maximum brightness for saturated pixels)
    # algo e.g.: https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/
    # note: in RGB, one Channel is max (setting "value"), one is min (setting "saturation") and the left-over zig-zags in-between, setting hue
    # as hue is 0-179 covering the 3 "zig" and 3 "zag", a hue difference of 30 covers a full 255 swing of the "in between" color
    for sat_thresh in ([70] if (not calibrate_sat_thresh) else range(40, 200, 10)):  # for calibration set to False, check imgs/saturated_eroded_x.jpg
        img3 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, sat_map = cv2.threshold(img3[:, :, 1], sat_thresh, 255, cv2.THRESH_BINARY)  # if value >135, set to 255. 135 good (beyond 150, objects get lost. below 80, background appears)
        for layer in [1, 2]:  # 1=S, 2=V
            img3[:, :, layer] = sat_map[:, :]
        # careful: black has hue=0, which is normally red. So cannot work with hue only. -> use BGR
        img3 = cv2.cvtColor(img3, cv2.COLOR_HSV2BGR)  # transform back to BGR
        img3 = cv2.erode(img3, kernel=np.ones((3, 3), np.uint8),iterations=1)  # now kill bordering pixels, takes out noise
        if calibrate_sat_thresh: imwrite(f"imgs/saturated_eroded_{sat_thresh:03}.jpg", img3)

    objs=[]
    for color in colors:
        if list_RGB_thresh: print(f"{color:>7}: ")

        dthre = 20
        for RGB_thresh in (range(0,250,dthre) if list_RGB_thresh else [70]):
            # noinspection PyUnboundLocalVariable
            ref=np.zeros_like(img3)
            color_HSV=cv2.cvtColor(np.array([[color_lookup_BGR[color]]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0, 0]
            color_HSV[1:3]=255 #max Saturation & "Value" (~Intensity)
            color_BGR=cv2.cvtColor(np.array([[color_HSV]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
            ref[:,:,:]=color_BGR
            diff=cv2.absdiff(img3,ref)
            diff=np.amax(diff,axis=2) #now a 2D = greyscale img
            #blur=3; diff = cv2.medianBlur(diff,blur); diff = cv2.medianBlur(diff,blur)
            diff = 255-diff
            thresh=255-RGB_thresh
            _, diff = cv2.threshold(diff, thresh, 255, cv2.THRESH_BINARY)  # much faster than diff[diff<thresh]=0; diff[diff>=thresh]=255
            if calibrate_sat_thresh:
                imwrite(f"imgs/thresh_{color}_{RGB_thresh:>03}.jpg", diff)

            totarea=0
            found = 0
            contours, hierarchy  = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 400us on PC
            for cont in contours:
                convcont = cv2.convexHull(cont)  # includes street-colored areas of objects, but also more street/shadows
                area = cv2.contourArea(convcont)
                totarea+=area
                if area > minarea:
                    m = cv2.moments(convcont)
                    cx = m['m10'] / m['m00']
                    cy = m['m01'] / m['m00']
                    inds = (int(round(cy)), int(round(cx)))  # (y,x)
                    objs+=[(color,area,inds)]
                    found+=1
            if list_RGB_thresh: print(f"    {RGB_thresh:>2}: found={found:>3}  area={int(totarea):>05} bgr={str(color_BGR)}")
        # img = np.zeros_like(diff)
        #cv2.drawContours(img, [convcont], -1, 255, 1)
        #imshowrgb('objects', cv2.add(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB), scan.trackobjimg))

    maxobjs={} #note: if listthresh, then objs holds results of all different values of RGB_thresh
    for color in colors:
        maxarea = max([-1]+[obj[1] for obj in objs if obj[0]==color])
        maxobj = [obj for obj in objs if obj[0]==color and obj[1]==maxarea]
        if len(maxobj)>0: maxobjs[color] = maxobj[0][1::]

    missingcolors = []
    fileindextext=""
    if save_analyzeimage_result_image:
        try:
            with open("analyzeimage_fileindex.int.text", "r") as f:
                fileindex = int(f.readline())
        except (FileNotFoundError, ValueError): fileindex = 0
        image2=copy.deepcopy(image) #necessary to not overwrite original in imagetable, takes 4.6-5.1 ms
        imwrite(f"imgs/analyzeimage_{fileindex:>04}_original.jpg", image2, False) #todo 3: maybe take out later. currently helpful
        for color in colors:
            if color in maxobjs:
                pixelpos = maxobjs[color][1][::-1] #now (x,y)
                cvcol=tuple(map(int,color_lookup_BGR[color]))
                image = cv2.line(image, (pixelpos[0]-8, pixelpos[1]-8), (pixelpos[0]+8, pixelpos[1]+8),cvcol, 9)
                image = cv2.line(image, (pixelpos[0]-8, pixelpos[1]-8), (pixelpos[0]+8, pixelpos[1]+8),(0,0,0), 5)
                image = cv2.line(image, (pixelpos[0]+8, pixelpos[1]-8), (pixelpos[0]-8, pixelpos[1]+8),cvcol, 9)
                image = cv2.line(image, (pixelpos[0]+8, pixelpos[1]-8), (pixelpos[0]-8, pixelpos[1]+8),(255,255,255), 5)
            else: missingcolors+=[color]
        imwrite(f"imgs/analyzeimage_{fileindex:>04}_result.jpg", image, False) #todo 3: maybe take out later. currently helpful
        fileindextext=f". last file index is {fileindex:>04}"
        with open("analyzeimage_fileindex.int.text", "w") as f: f.write(str(fileindex+1))
    missingcolortext=f". {missingcolors} missing." if missingcolors else ""
    logthis(f"analyzeimage(colors={pp(colors)}) returns {pp(maxobjs)}{missingcolortext}{fileindextext}")
    return maxobjs
imgmask=np.zeros((480,640,3),dtype=np.uint8)+255 #default: everything visible
polygons = [[[0,0],[0,150],[400,0]],
            [[300,0],[639,0],[639,150]],
            [[160,479],[250,420],[375,410],[450,470],[560,400],[615,320],[639,240],[639,479]],
            ]
for polygon in polygons: cv2.fillPoly(imgmask, np.array([polygon],dtype=np.int32), (0,0,0), 8) #careful, draws into imgmask directly. linetype 8 means fill.
def getcmpos(color, precise=True):
    def getcmpos_base(color):
        """calculates the coordinates of a colored object in cm-coordinates (robot-fixed). Returns (x,y) or None"""
        img = getimagefromcamera(camera)
        ai = analyzeimage(img, list_RGB_thresh=False, colors=[color])
        if color in ai:
            pixelpos = ai[color][1][::-1]  # now (x,y)
            chesspos = pixtochess(pixelpos)
            cmpos = chesstocm(chesspos)
            # print(pixelpos, chesspos, cmpos)  # all 2D vectors
            return cmpos
        else:
            return None

    moveservoup()
    cmpos = getcmpos_base(color)
    if precise:
        precisedelay=0.63
        pausemotion(precisedelay) #delay of 0 happens to be half a oscillation cycle
        cmpos2 = getcmpos_base(color)
        if (cmpos is None) or (cmpos2 is None):
            cmpos=None
        else:
            cmpos=(cmpos+cmpos2)/2
    return cmpos #which can be None
def getcmpos_scandelay():
    def cmposdiff(delay):
        li=[]
        for _ in range(5):
            cmpos1 = getcmpos("blue",precise=False)
            pausemotion(delay)
            cmpos2 = getcmpos("blue",precise=False)
            li+=[cmpos1[0]-cmpos2[0]]
        return li
    li2=[]
    for i in range(18):
        dely=0.40+i*0.3/18
        li=cmposdiff(dely)
        li2+=[(dely,max(np.abs(np.array(li))))]
    print(li2)
if dev:
    # calibration of precisedelay (delay between 2 pictures to reduce effect of mechanical oscillations)
    # put blue tag before camera and record maximum deviation of recognized position, depending on different delay times between images
    standup()
    getcmpos_scandelay()
    #typical result: i.e. lower fluctuations around delays of ~0 and ~0.5 (offset 0 is coincidence, difference matches osc. frequency of ~2.0 Hz)
    #[(0.4, 3.412976171732325), (0.4166666666666667, 4.327216855615461), (0.43333333333333335, 3.3155701906308224), (0.45, 3.9071428594494293), (0.4666666666666667, 3.318646049696085), (0.48333333333333334, 4.052077132797706), (0.5, 1.7082072626301539), (0.5166666666666667, 1.7287505693943928), (0.5333333333333333, 3.31249700459972), (0.55, 0.47033850330212346), (0.5666666666666667, 2.1005846275996163), (0.5833333333333334, 0.8422507008273783), (0.6, 2.073062641836806), (0.6166666666666667, 0.6769262441579116), (0.6333333333333333, 0.6258059353587342), (0.65, 0.5754728685998032), (0.6666666666666667, 1.6930903307469123), (0.6833333333333333, 1.2138487171992765)]
    #given output, choose delay time 0.63 with minimum deviation and set "precisedelay" accordingly above

if dev:
    #calibrate sat_thresh: Illuminate right. select img source. run this. Look at files "imgs/saturated_eroded...".
    standup()
    pause(1)
    img=getimagefromcamera(camera)
    #img=cv2.imread("imgs/analyzeimage_0512_original.jpg")
    res1 = analyzeimage(img, calibrate_sat_thresh=True)
    saveimages()
    # -> with new bright white LED lamps, background goes away at sat_thresh >=40. Yellow borders start to die at >=80, green ant >=110
    # -> decide for sat_thresh = 70. Change value in "for sat_thresh in ([<here>]...", and in filenames below.
    # (previously, 100 was good during daytime, 140 during nighttime)

    #calibrate for RGB_thresh: run this. look at files "thresh_color_..."
    img = cv2.imread("imgs/saturated_eroded_070.jpg")
    res = analyzeimage(img, list_RGB_thresh=True, calibrate_sat_thresh=True)
    saveimages()
    #example case:
    #color  thesh_color_complete    other_color appears_a_bit@thresh    appears_a_lot@thresh
    #blue   120                     green       160                     160
    #green  60                      blue        140                     180
    #red    40                      yellow      20                      240
    #yellow 100                     red         220                     220
    #       column max = 120                                            column min = 160
    #conclusion: a bit other color cannot be avoided. pick RGB_thresh near 1st column max (last pixels not so important), avoid last column (must avoid, may override wanted color)
    #-> decide for RGB_thres = 120. Enter in analyzeimage at "... list_RGB_thresh else [120]):"
    # with new LED lights (!), all 4 colors are ok between 40 and 100. -> decide for 70.

    #check Saturation/Value for hue
    image = cv2.imread("imgs/saturated_eroded_080.jpg")
    image = image & imgmask #label: "disable_imgmask"
    calibrate=False
    for sat_thresh in ([85] if (not calibrate) else range(50,200,10)): #for calibration set to False, check imgs/saturated_eroded_x.jpg
        # noinspection PyRedeclaration
        img5 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        _, sat_map = cv2.threshold(img5[:, :, 1], sat_thresh, 255,cv2.THRESH_BINARY)  # if value >135, set to 255. 135 good (beyond 150, objects get lost. below 80, background appears)

        for layer in [1, 2]: #1=S, 2=V
            img5[:, :, layer] = sat_map[:, :]
            count = np.zeros(256, dtype=np.uint32)
            for v in img5[:, :,layer].ravel(): count[v] += 1
            print(pp(count))

        #careful: black has hue=0, which is normally red. So cannot work with hue only. -> use BGR
        img5 = cv2.cvtColor(img5, cv2.COLOR_HSV2BGR) #transform back to BGR
        img5 = cv2.erode(img5, kernel=np.ones((5, 5), np.uint8), iterations=1) # now kill bordering pixels, takes out noise
        #if calibrate: imwrite(f"imgs/saturated_eroded_{sat_thresh:03}.jpg",img5)

    #timing measurements
    for _ in range(5):
        tic = time.time()
        getcmpos("blue",precise=True) #-> 1.75s! ~ 2x0.51 (get image) + 0.63 (precise wait) + 0.16 (analyze)
        print(time.time()-tic)
    for _ in range(5):
        tic = time.time()
        img = getimagefromcamera(camera) #0.51s
        print(time.time() - tic)
    for _ in range(5):
        tic = time.time()
        analyzeimage(img,colors=color_lookup_BGR.keys()) #-> 0.052 - 0.076 seconds for 1 color, 0.145-0.189 s for 4 colors
        print(time.time() - tic)
    img=getimagefromcamera(camera)
    for _ in range(5):
        tic=time.time()
        img2=copy.deepcopy(img) #0.005 s
        print(time.time()-tic)

#calibrate & motion ###################################################################################################
def vec3d(x,y): return np.array([x,y,1],dtype=float)
if dev: #calibrate magnet position, execute in steps
    moveservodown()
    #manually position yellow part below magnet here
    moveservoup()
    color="yellow"
    li=[]
    for _ in range(10):
        li+=[getcmpos(color, True)]
    xis,yis=np.round(np.median(np.array(li),axis=0),3)
    print(f"pos_magnet=vec3d({xis},{yis})")
    #copy/past printed line as next line
pos3d_magnet=vec3d(-1.803, 20.692) #as measured by camera.
if dev: #calibrate rfid sensor position, execute in steps
    #manually position yellow part where sensor is seen
    #manually disable imgmask at label: "disable_imgmask"
    color="yellow" #xyz
    li=[]
    for _ in range(10):
        li+=[getcmpos(color, True)]
    xis,yis=np.round(np.median(np.array(li),axis=0),3)
    print(f"pos3d_rfid=vec3d({xis},{yis})")
    #manually ENABLE imgmask at label: "disable_imgmask"
    #copy/past printed line as next line
    #pos3d_rfid=vec3d(-7.126, -3.072) #re-defined later
# see: https://en.wikipedia.org/wiki/Transformation_matrix
# design decision: keep 0's and 1's in matrices/vectors for simpler development, costs some memory/performance
def M_trans(x,y): return np.array([[1,0,x],[0,1,y],[0,0,1]])
def M_rot(deg): rad=deg/180*math.pi; s=math.sin(rad); c=math.cos(rad); return np.array([[c,-s,0],[s,c,0],[0,0,1]]) #rotates left
def M_move(fwd,deg,n=8): #preferrably n is a power of 2, then matrix_power can be computed faster
    return np.linalg.matrix_power(np.dot(M_rot(deg/n),M_trans(fwd/n,0)),n)

def moveover(device_robot, target, stopdist=0, maxdeviation=0.7):
    """move a robot device, which is at device_robot in robot coordinates, over target (also in robot coordinates),
    stopping stopdist (typ. 2cm) before target to allow for fine-tuning in a 2nd move.
    maxdeviation is the allowable deviation permissible for a 1-move motion, otherwise a 2-move motion is calculate
    returns: motion to reach desired position"""

    def moveover2(device_robot,target,stopdist=0):
        #not necessary: transform real-space to robot coordinates -> target_robot
        def reducedist(dist, stopdist):
            return sgn(float(dist)) * max(0, abs(dist) - stopdist)  # reduce dist by stopdist, keeping sign

        if dev: #transformation of target_real_coordinates:
            robot_pose=pose()
            target_robot=np.linalg.multi_dot((M_rot(-robot_pose.degr),M_trans(-robot_pose.xr,-robot_pose.yr),target))
        else: target_robot=target
        #then, rotate robot so that target_robot is just forward/backward of installation_robot.
        radius2_target= target_robot[0] ** 2 + target_robot[1] ** 2
        fwd2= radius2_target - device_robot[1] ** 2
        #this may geometrically not be possible (if target is too close to center), in this case must move first, then rotate...
        #assert (device_robot[1] > 0)  # 0 or negative values not needed for my robot, this is true both for magnet and rfid
        if fwd2<0: #move first
            fwd2=device_robot[0]**2+device_robot[1]**2-target_robot[1]**2
            travel_dist=target_robot[0]+math.sqrt(fwd2)  #travel_dist defined to have opposing sign. pick negative sqrt, so target is always at negative x (the side of the devices)
            target_angle=math.atan2(math.sqrt(fwd2),target_robot[1]) #is >pi/2 if target_robot is negative
            device_angle=math.atan2(-device_robot[0],device_robot[1])
            rot_angle=target_angle-device_angle
            travel_dist=reducedist(travel_dist,stopdist) #reduce travel distance by stopdist, but keep sign (or at least 0)
            motion=[("fwd",travel_dist),("trn",rot_angle*180/math.pi)]
        else: #rotate first
            target_angle=math.atan2(-target_robot[0],target_robot[1]) #as robot moves in x, this is the angle vs. y-Axis. Note unusual indices! also note that atan2 handles all 4 sign combinations correctly
            deviceX_angle=math.atan2(math.sqrt(fwd2),device_robot[1]) #always positive
            #if abs(target_angle-device_angle)>abs(target_angle+device_angle): device_angle=-device_angle #pick sign so rotation angle is the smaller one
            rot_angle=target_angle-deviceX_angle
            target_robot2=np.dot(M_rot(-rot_angle*180/math.pi),target_robot) #rotate robot such that targetg gets in front of device
            assert(abs(target_robot2[1]-device_robot[1])<1e-10) #"in front of" means no y-difference, as robot moves in x
            travel_dist=reducedist(target_robot2[0]-device_robot[0],stopdist)
            motion= [("trn",rot_angle*180/math.pi),("fwd",travel_dist)]
        #logthis(f"moveover2({pp(device_robot)},{pp(target)},{stopdist:5.1f}) -> {pp(motion)}")
        return motion

    def moveover1(device_robot,target):
        """try to only rotate and get close. returns motion,deviation"""
        target_angle = math.atan2(-target[0], target[1])  # as robot moves in x, this is the angle vs. y-Axis. Note unusual indices! also note that atan2 handles all 4 sign combinations correctly
        device_angle = math.atan2(-device_robot[0], device_robot[1])
        rot_angle=target_angle-device_angle
        target2 = np.dot(M_rot(-(rot_angle) * 180 / math.pi),target)  # rotate robot such that targetg gets in front of device
        deviation=np.linalg.norm(target2-device_robot)
        motion=[("trn",rot_angle*180/math.pi)]
        #logthis(f"moveover1({pp(device_robot)},{pp(target)}). deviation={deviation:5.1f} -> {pp(motion)}")
        return motion,deviation

    motion,deviation=moveover1(device_robot,target) #ignores stopdist, fix if it turns out to be an issue
    if deviation>maxdeviation: motion=moveover2(device_robot,target,stopdist=stopdist)
    logthis(f"moveover({pp(device_robot)},{pp(target)},{stopdist:5.1f},{maxdeviation:4.1f}. deviation={deviation:5.1f}, returns {pp(motion)}")
    return motion
def robot_motion(motion): #motion: list of moves
    """execute the moves listed in motion"""
    max_fwd=max([0]+[move[1] for move in motion if move[0]=="fwd"])
    if max_fwd < 70:
        for move in motion:
            makemove(move)
    else: logthis(f"too far: {motion[1]}")
def pos_antimotion(pos,motion):
    """move vector pos with opposite moves (in same order) defined in motion, return resulting pos"""
    for move in motion:
        if move[0]=="fwd": pos=np.dot(M_trans(-move[1],0),pos)
        elif move[0]=="trn": pos=np.dot(M_rot(-move[1]),pos)
        else: fatal(move[0]+" is wrong move code")
    return pos
def test_moveover():
    """testing for the correctness of moveover calculations (all 4 angle quadrants etc.)"""
    import random #used only for testing, not production
    zero_pos = np.array([0.0, 0.0, 1.0])  # note 1 in 3rd dim, this is a standard vector
    x1_pos = np.array([1.0, 0.0, 1.0])
    xy_pos = np.array([math.sqrt(0.5), math.sqrt(0.5), 1.0])
    y1_pos = np.array([0.0, 1.0, 1.0], dtype=float)
    for _ in range(10000):
        target=vec3d(random.random()-0.5,random.random()-0.5)
        if random.random()>0.8: target=zero_pos
        if random.random()>0.8: target=x1_pos
        if random.random()>0.8: target=y1_pos
        if random.random()>0.8: target=xy_pos
        device = vec3d(random.random()-0.5, random.random()-0.5)
        if random.random()>0.8: device=zero_pos
        if random.random()>0.8: device=x1_pos
        if random.random()>0.8: device=xy_pos
        if random.random()>0.8: device=y1_pos
        motion=moveover(device,target,stopdist=0,maxdeviation=0.7)
        deviation=pos_antimotion(target,motion)-device
        if np.linalg.norm(deviation)>1e-13:
            print(f"moveover err:",(target,device,motion,deviation))
if debug: test_moveover()

#testing/main loop ###################################################################################################
def calcmotion(cmpos, stopdist, pos3d_device,maxdeviation):
    """calculate the motion necessary to move the device at pos3d_device above cmpos"""
    target = np.array([cmpos[0], cmpos[1], 1])
    motion = moveover(pos3d_device, target, stopdist,maxdeviation)
    logthis(f"calcmotion({pp(cmpos)},{stopdist},{pos3d_device})->{pp(motion)}")
    return motion
def findcmpos(color,firstmotions=None,precise=True):
    """try to ensure that the object is seen. move robot if necessary. returns cmpos or aborts"""
    if firstmotions is None: firstmotions=[[("trn",-45)],
                                           [("trn",90)],
                                           [("trn",-45),("fwd",20)]]
    motions=firstmotions+[[("trn",-45)] for _ in range(7)]+ [[("trn",135),("fwd",20)]] + [[("trn",-45)] for _ in range(7)]
    for motion in motions:
        cmpos= getcmpos(color, precise)
        if not cmpos is None: break
        robot_motion(motion)
        pausemotion(0.5) #fwd is rare in motion, accept short waiting times
    else: logthis(f"findcmpos ERROR: {color} not seen. Aborting..."); raise Abort_error
    return cmpos
if dev:
    #turn stepwise and display distance to yellow tag. This is to check correct positioning of objects
    #turn(-10)
    angsum=0
    angdiff=-10
    color="yellow"
    for _ in range(10):
        poseis=pose()
        cmpos= getcmpos(color, True)
        logthis(f"angsum={angsum:>4}, total angle={poseis.degr:4.0f}, dist={np.linalg.norm(cmpos):5.1f}, pixpos={pp(cmpos)}")
        turn(angdiff)
        pausemotion(1)
        angsum+=angdiff
def findmoveover(color, device_pos, stopdist, precise, maxdeviation, firstmotions=None):
    """execites the motion necessary to move the device at device_pos above the object of color color"""
    logthis(f"findmoveover({color},{pp(device_pos)},{stopdist},{pp(firstmotions)},{precise})")
    cmpos = findcmpos(color,firstmotions,precise)
    motion = calcmotion(cmpos, stopdist, device_pos,maxdeviation)
    robot_motion(motion) #pausemotion not called here, call after calling findmoveover
def tryreadout():
    """try to move the RFID reader above the tag and read it. Returns (readout,decoded)"""
    logthis("\ntry to read out RFID")
    for bigmove in range(1):
        if 0==1: #try to move near first. works well, but this is an unintuitive action for on-lookers
            findmoveover("blue", pos3d_rfid, stopdist=12, precise=False, maxdeviation=0.7, firstmotions=None)  # move somewhat close first
            pausemotion(0.5)  # dampen
        findmoveover("blue", pos3d_rfid, stopdist=1, precise=True, maxdeviation=0.7, firstmotions=None)  # 2: typically overshoots
        micromoves = [[("trn", 10)], [("trn", -20)], [("trn", 10), ("fwd", 2)], [("fwd", -5)], []]
        tryreadtime = 1.0
        for micromove in micromoves:  # iterate if the tag is seen, but RFID does not read
            starttime = time.time()
            trycount = 0
            while time.time() - starttime < tryreadtime:  # try reading for some time while robot still moves a bit
                decoded=rfid_read()
                readout = not (decoded is None)
                trycount += 1
                if readout:
                    decoded = decoded[0]  # 0 or 1 # todo 3: could implement some error tolerance. Never needed so far
                    break  # inner loop
            else:
                logthis(f"  RFID not read out in {trycount} tries, micromove={pp(micromove)}, bigmove={bigmove}")
                robot_motion(micromove)
                tryreadtime = 2.0 if [move for move in micromove if move[0] == "fwd"] else 1.0  # try a little longer if "fwd"
            if readout: break  # middle loop
        if readout: break  # outer loop
        forward(15)  # complete new retry...
    else:
        logthis(f"\nRFID not read out. Aborting..."); raise Abort_error
    logthis(f"tag read out with decoded={decoded}")
    return readout,decoded

def trypickup(pickup_stopdist):
    """try to pick up the tag including moving the robot there"""
    logthis("\nmove magnet over tag")
    motion = calcmotion(pos3d_rfid[:2], stopdist=pickup_stopdist, pos3d_device=pos3d_magnet,maxdeviation=0.7)
    robot_motion(motion)
    logthis("\ntry to pick up tag")
    moveservoup()
    stopdist=0
    color = "yellow"
    for bigmove in range(6):
        #stopdist = 3 if bigmove==0 else 0 #magnet should already be very close since we were at RFID reader before
        findmoveover(color, pos3d_magnet, stopdist, True, 0.7, firstmotions=[[("trn", -10)]])
        pausemotion(1.3)  # let robot converge to magnet position
        cmpos2 = findcmpos(color, None, True)
        offset = np.linalg.norm(cmpos2 - pos3d_magnet[:2])
        logthis(f"  OFFSET={offset:4.1f}~{pp(cmpos2 - pos3d_magnet[:2])}")
        if offset < 1.1:
            setmagnet(True)
            moveservodown()  # this is also wait time for magnet to acquire field
            pausemotion(0.3)  # robot oscillates mechanically. this may help magnet to catch
            moveservoup(20)  # at x% of speed to not shake tag off
            cmpos = getcmpos(color, True)
            if (cmpos is None) or (np.linalg.norm(cmpos - pos3d_magnet[:2])>10): break  #... or there is another tag >x cm away
        logthis(f"  not picked up. bigmove={bigmove}. retrying...")
    else: logthis(f"\nERROR. tag not picked up. Aborting..."); raise Abort_error
    logthis(f"tag picked up.")
    #setmagnet(0)
#pos3d_rfid=vec3d(-7.126, -3.072) #measured via camera, in old camera calibration. geometric measurement: (-6.1, 1.0)

if dev: #used to optimize stopdist, then adapt in 3rd line of trypickup
    startpose = pose()
    pos3d_rfid = vec3d(-8, 1.5)
    motion = calcmotion(pos3d_rfid[:2], stopdist=3, pos3d_device=pos3d_magnet,maxdeviation=0.7)
    robot_motion(motion)
    moveservodown()
    moveservoup()
    movetopose(pose(), startpose)
    decoded=1

def tryland(decoded):
    """try to put the tad in the landing zone described by decoded"""
    logthis("\ntry to move to landing zone")
    target = "green" if decoded == 1 else "red"
    pos_laydown = vec3d(pos3d_magnet[0]-3, pos3d_magnet[1] - 3)
    # offsets used for tuning the laydown position. Necessary as the magnet lever obscures some part of the landing zone
    # x: +3 places more to RFID side, -3 more opposite.
    # y: -3 places  it more on the opposite side of the robot...
    findmoveover(target, pos_laydown, stopdist=0, precise=False, maxdeviation=1.2, firstmotions=[[("trn", -10)]])
    pausemotion(1)  # relaxtime)  # let robot converge to magnet position
    findmoveover(target, pos_laydown, stopdist=0, precise=True, maxdeviation=1.2, firstmotions=[[("trn", -10)]])
    moveservodown()
    setmagnet(0)
    moveservoup()

def finish():
    """wave goodbye and lay down"""
    pause(1)
    wavehand()
    logthis("\ndone")
    pausemotion(3)
    laydown()

print() #set breakpoint here to start experimenting
standup()
#turn(-60)
#forward(15)
#laydown()


#regular operation find, readout, pickup, transport & deposit
pause(13)
pos3d_rfid = vec3d(-8, 1.5)
if 1==1:
    try:
        standup()
        pausemotion(1) #minimum wait to stabilize view
        startpose = pose()
        for tagnr in range(2):
            readout,decoded=tryreadout()
            trypickup(-2)
            tryland(decoded)
            movetopose(pose(), startpose)
        finish()
    except Abort_error:
        setmagnet(0)
        logthis("Abort_error caught")
    finally:
        laydown()
        saveimages()

if debug:#debug: stress test logging
    logthis("starting stress testing of logging, part 1 - intensified 'standard' logging")
    testtime=10 #seconds
    save = reglog.delaytime
    reglog.delaytime = 0
    def stresstestlog():
        import random #used only for testing
        start = time.time()
        while time.time() - start < testtime:
            # if random.randint(0,2)==0: turn(0)
            # if random.randint(0,2)==0: forward(0)
            if random.randint(0, 2) == 0: regularlog()
            if random.randint(0, 2) == 0: img = getimagefromcamera(camera)
            if random.randint(0, 2) == 0: setmagnet(1 - magnet.magnetstatus)
            if random.randint(0, 2) == 0: moveservo(servo.dc_from)
            logthis("stresstestlogging")
    stresstestlog()

    logthis("starting stress testing of logging, part 2 - logging from 3 threads")
    def stresstestlog2():
        start=time.time()
        while time.time()-start<testtime:
            regularlog()
            logthis("stresslog1")
    t1 = threading.Thread(target=stresstestlog)
    t1.start()
    t2 = threading.Thread(target=stresstestlog2)
    t2.start()
    t1.join() #without the joins, the shutdown will run in conflicts, as it mixes stresslog1/2 items
    t2.join()
    reglog.delaytime = save
    del(testtime)
    logthis("stress testing of logging ended")
shutdown()
#in addition - RPi shutdown: sudo shutdown -h now, wait until blinking stops +5s, then power off.
