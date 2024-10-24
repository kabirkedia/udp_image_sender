import socket
import time
import os
import sys


host = ""
port = 6000

def ServerPut(clientAddr,s,filename):
    print("Sending Acknowledgment of command.")
    msg = "Valid Put command. Let's go ahead "
    msgEn = msg.encode('utf-8')
    s.sendto(msgEn, clientAddr)
    print("Message Sent to Client.")
    print("In Server, Put function")
    BigSAgain = open(filename, "wb")
    d = 0
    print("Receiving packets will start now if file exists.")
    #print("Timeout is 15 seconds so please wait for timeout at the end.")
    try:
        Count, countaddress = s.recvfrom(512)  # number of packet
    except ConnectionResetError:
        print(
            "Error. Port numbers not matching. Exiting. Next time enter same port numbers.")
        sys.exit()
    except:
        print("Timeout or some other error")
        sys.exit()
    print("Count: ", Count)
    tillI = Count.decode('utf8')
    tillI = int(tillI)
    while tillI != 0:
        ServerData, serverAddr = s.recvfrom(512)
        dataS = BigSAgain.write(ServerData)
        d += 1
        tillI = tillI - 1
        print("Received packet number:" + str(d))

    BigSAgain.close()
    print("New file closed. Check contents in your directory.")

def main():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Server socket initialized")
        s.bind((host, port))
        print("Successful binding. Waiting for Client now.")
        # s.setblocking(0)
        # s.settimeout(15)
    except socket.error:
        print("Failed to create socket")
        sys.exit()

    while True:
        try:
            data, clientAddr = s.recvfrom(4096)
        except ConnectionResetError:
            print(
                "Error. Port numbers not matching. Exiting. Next time enter same port numbers.")
            sys.exit()
        text = data.decode('utf8')
        ServerPut(clientAddr,s,text)

if __name__ == "__main__":
    main()