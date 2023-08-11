from socket import *
from numpy import loadtxt
import struct

def initpack():
    data=[0,1]
    pack=''.encode()
    for i in data:
        print(pack)
        pack+=struct.pack('>I',i)
    return pack

def endpack():
    data=[2,1]
    pack=''.encode()
    for i in data:
        pack+=struct.pack('>I',i)
    return pack

def explainRobData(data):
    pack=list([])
    # packet type
    temp=struct.unpack('>I',data[0:4])# unsigned int
    pack.append(temp[0])
    # version no
    temp=struct.unpack('>I',data[4:8])
    pack.append(temp[0])
    # sequence no
    temp=struct.unpack('>I',data[8:12])
    pack.append(temp[0])
    # status
    temp=struct.unpack('>B',data[12:13])# unsigned char
    pack.append(temp[0])
    # read io type
    temp=struct.unpack('>B',data[13:14])# unsigned char
    pack.append(temp[0])
    # read io index
    temp=struct.unpack('>H',data[14:16])# unsigned short
    pack.append(temp[0])
    # read io mask
    temp=struct.unpack('>H',data[16:18])# unsigned short
    pack.append(temp[0])
    # read io value
    temp=struct.unpack('>H',data[18:20])# unsigned short
    pack.append(temp[0])
    # time stamp
    temp=struct.unpack('>I',data[20:24])
    pack.append(temp[0])
    # X [mm]
    temp=struct.unpack('>f',data[24:28])# float
    pack.append(temp[0])
    # Y [mm]
    temp=struct.unpack('>f',data[28:32])
    pack.append(temp[0])
    # Z [mm]
    temp=struct.unpack('>f',data[32:36])
    pack.append(temp[0])
    # W [deg]
    temp=struct.unpack('>f',data[36:40])
    pack.append(temp[0])
    # P [deg]
    temp=struct.unpack('>f',data[40:44])
    pack.append(temp[0])
    # R [deg]
    temp=struct.unpack('>f',data[44:48])
    pack.append(temp[0])
    # ext axis 1
    temp=struct.unpack('>f',data[48:52])
    pack.append(temp[0])
    # ext axis 2
    temp=struct.unpack('>f',data[52:56])
    pack.append(temp[0])
    # ext axis 3
    temp=struct.unpack('>f',data[56:60])
    pack.append(temp[0])
    # J1
    temp=struct.unpack('>f',data[60:64])
    pack.append(temp[0])
    # J2
    temp=struct.unpack('>f',data[64:68])
    pack.append(temp[0])
    # J3
    temp=struct.unpack('>f',data[68:72])
    pack.append(temp[0])
    # J4
    temp=struct.unpack('>f',data[72:76])
    pack.append(temp[0])
    # J5
    temp=struct.unpack('>f',data[76:80])
    pack.append(temp[0])
    # J6
    temp=struct.unpack('>f',data[80:84])
    pack.append(temp[0])
    # J7
    temp=struct.unpack('>f',data[84:88])
    pack.append(temp[0])
    # J8
    temp=struct.unpack('>f',data[88:92])
    pack.append(temp[0])
    # J9
    temp=struct.unpack('>f',data[92:96])
    pack.append(temp[0])
    # J1 mot cur [A]
    temp=struct.unpack('>f',data[96:100])
    pack.append(temp[0])
    # J2 mot cur [A]
    temp=struct.unpack('>f',data[100:104])
    pack.append(temp[0])
    # J3 mot cur [A]
    temp=struct.unpack('>f',data[104:108])
    pack.append(temp[0])
    # J4 mot cur [A]
    temp=struct.unpack('>f',data[108:112])
    pack.append(temp[0])
    # J5 mot cur [A]
    temp=struct.unpack('>f',data[112:116])
    pack.append(temp[0])
    # J6 mot cur [A]
    temp=struct.unpack('>f',data[116:120])
    pack.append(temp[0])
    # J7 mot cur [A]
    temp=struct.unpack('>f',data[120:124])
    pack.append(temp[0])
    # J8 mot cur [A]
    temp=struct.unpack('>f',data[124:128])
    pack.append(temp[0])
    # J9 mot cur [A]
    temp=struct.unpack('>f',data[128:132])
    pack.append(temp[0])
    return pack

def commandpack(data):
    sequence_no = data[0]
    last_data = data[1] # whether last command data
    data_style = data[2] # joint or cartesian
    jnt_data = data[3] # all joints position (cartesian or joint + ext axis)
    pack=''.encode()
    # packet type
    pack+=struct.pack('>I',1)
    # version no
    pack+=struct.pack('>I',1)
    # sequence no
    pack+=struct.pack('>I',sequence_no)
    # last data
    pack+=struct.pack('>B',last_data)
    # read io type
    pack+=struct.pack('>B',0) # don't read io
    # read io index
    pack+=struct.pack('>H',0) # not reading now
    # read io mask
    pack+=struct.pack('>H',0) # not reading now
    # data style
    pack+=struct.pack('>B',data_style)
    # write io type
    pack+=struct.pack('>B',0) # currently not writing
    # write io index
    pack+=struct.pack('>H',0) # currently not writing
    # write io mask
    pack+=struct.pack('>H',0) # currently not writing
    # write io value
    pack+=struct.pack('>H',0) # currently not writing
    # unused 2 byte
    pack+=struct.pack('>H',0)
    # X or J1
    pack+=struct.pack('>f',jnt_data[0])
    # Y or J2
    pack+=struct.pack('>f',jnt_data[1])
    # Z or J3
    pack+=struct.pack('>f',jnt_data[2])
    # W or J4
    pack+=struct.pack('>f',jnt_data[3])
    # P or J5
    pack+=struct.pack('>f',jnt_data[4])
    # R or J6
    pack+=struct.pack('>f',jnt_data[5])
    # ext axis 1
    pack+=struct.pack('>f',jnt_data[6])
    # ext axis 2
    pack+=struct.pack('>f',jnt_data[7])
    # exit axis 3
    pack+=struct.pack('>f',jnt_data[8])
    return pack

def getStatus(data):
    accept_cmd = (data[3] & 0b0001)>0
    received_cmd = (data[3] & 0b0010)>0
    sysrdy = (data[3] & 0b0100)>0
    rbt_inmotion = (data[3] & 0b1000)>0
    status = [accept_cmd, received_cmd, sysrdy, rbt_inmotion]
    return status

if __name__ == "__main__":
    #UDP_IP = "192.168.7.2"#"192.168.0.112"#"127.0.0.1"#"192.168.0.112"
    #UDP_PORT = 60015
    # UDP_IP = '127.0.0.1'#"172.22.32.1"#"192.168.1.200"
    UDP_IP = '192.168.1.100'
    UDP_PORT = 60015

    sock = socket(AF_INET,SOCK_DGRAM)
    sock.connect((UDP_IP,UDP_PORT)) # connect not bind since robot seems started server

    # read data
    datafilename = 'sample_7l.txt' #'sample_0pos.txt'
    jntdata = loadtxt(datafilename,delimiter='\t',unpack=False)
    jntdata = jntdata.tolist()
    datalen = len(jntdata)
    print(datalen)
    # send init pack
    data = initpack()
    print(data)
    print(len(data))
    sock.sendto(data,(UDP_IP,UDP_PORT))
    #print('data sent')
    fbdata = sock.recv(132)
    #print(fbdata)
    fbdata=explainRobData(fbdata)
    print(getStatus(fbdata))
    print(fbdata[2])
    """
    for i in range(1000):
        data = commandpack([fbdata[2],0,1,fbdata[18:27]])
        sock.sendto(data,(UDP_IP,UDP_PORT))
        fbdata = sock.recv(132)
        fbdata = explainRobData(fbdata)
        print(getStatus(fbdata))
    """
    for j in range(1):
        for i in range(datalen):
            # motion data
            home = [0,0,0,0,0,0]#[90,0,0,0,-90,0]
            temp = jntdata[i]
            for j in range(6):
                temp[j]+=home[j]
            data = commandpack([fbdata[2],0,1,temp])
            #if (i == 0):    
            #    data = commandpack([fbdata[2],0,1,temp])#data = commandpack([1,0,1,temp])
            #elif (i < datalen-1):
                #data = commandpack([fbdata[2],0,1,temp])
                #data = commandpack([2,0,1,jntdata[i]])
            #else:
            #    data = commandpack([fbdata[2],1,1,jntdata[i]])
            sock.sendto(data,(UDP_IP,UDP_PORT))
            #print('data sent')
            fbdata = sock.recv(132)
            #print(fbdata)
            fbdata=explainRobData(fbdata)
            #print(fbdata[2])
            #print(getStatus(fbdata))
    data = commandpack([fbdata[2],1,1,temp])
    sock.sendto(data,(UDP_IP,UDP_PORT))
    fbdata = sock.recv(132)
    data = endpack()
    sock.sendto(data,(UDP_IP,UDP_PORT))
    sock.close()
