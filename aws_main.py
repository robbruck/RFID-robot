import numpy as np
import sys
sys.path+=["/home/ec2-user/.local/lib/python3.7/site-packages"]
import cv2
import socket

import logging
logging.basicConfig(filename="robot.log",level=logging.DEBUG,format='%(asctime)s %(message)s')
logging.info("") #for better visibility/separation of sections in log file
logging.info("aws_main.py started")

class EOFerror(Exception):
    """raised when input stream is prematurely/unexpectedly reaching eof"""
    pass

def readstream(conn,length):
    received=conn.read(length)
    #read() is blocking, unless other side closes/aborts connection, which causes an EOF being sent
    #then read() returns less then length bytes
    if len(received)<length:
        logging.debug(f"connection unexpectedly sent EOF: {length} bytes expected, {len(received)} bytes received.")
        raise EOFerror
    return received

def getfromstream(conn, fmt):
    import struct
    received = readstream(conn,struct.calcsize(fmt))
    return struct.unpack(fmt, received)[0]
def getstring(conn):
    length = getfromstream(conn, "<L")
    stri = readstream(conn,length).decode("utf-8")
    logging.info(f'string received: "{stri}"')
    return stri
def getimage(conn):
    shapey = getfromstream(conn, "<L")
    shapex = getfromstream(conn, "<L")
    depth = getfromstream(conn, "<L")
    image_len = shapey * shapex * depth
    image = readstream(conn,image_len)
    img = np.frombuffer(image, dtype=np.uint8).reshape(shapey, shapex, depth)
    logging.info(f'image received: format={shapex}x{shapey}x{depth}')
    return img
imagenumber = -1
def saveimage(img):
    global imagenumber
    if imagenumber < 0:
        import glob
        filenames = glob.glob("images/image*.jpg")
        imagenumber = max([int(fn.replace("images/image", "").replace(".jpg", "")) for fn in filenames])
    imagenumber += 1
    filename = "images/image" + str(imagenumber) + ".jpg"
    cv2.imwrite(filename, img)  # write file
    logging.info(f'image written: filename={filename}, shape={img.shape.__repr__()}')

keeprunning=True
while keeprunning:
    server_socket = socket.socket()
    server_socket.setblocking(True)
    portstart,portend=8000,8100
    for port in range(portstart,portend):
        try:
            server_socket.bind(('0.0.0.0', port)) #0.0.0.0 means all interfaces
            logging.info(f"port {port} selected")
            print(f"port {port} selected") #todo: put print into handler
            break
        except:
            logging.debug(f"error with port {port}, trying next...")
    else:
        logging.critical(f"no port in range({portstart},{portend}) found")
        print(1/0) #cause error, fix it if it happens once
    server_socket.listen(0)
    print("listening")
    # Accept a single connection and make a file-like object out of it
    connection = server_socket.accept()[0].makefile('rb')
    print("accepted")
    logging.info("accepted")

    images=[]

    try:
        while True:
            cmd=getfromstream(connection,"<L")
            if cmd==0: break #end of sending, shutdown this connection
            if cmd==1:
                image=getimage(connection)
                saveimage(image)
                images += [image]
            if cmd==2:
                stri=getstring(connection)
                if stri.find("err")>=0 or stri.find("ERR")>=0:
                    print("  ",stri)
    except EOFerror:
        logging.debug(f"caught EOFerror.")
    connection.close()
    server_socket.close()
    logging.info("connection closed.")
    print("connection closed.")

logging.info("aws_main.py ending...")
print("done.")
logging.shutdown()
