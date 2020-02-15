import subprocess
import time
import threading
import collections


def read_output(p, append):
    for line in iter(p.stdout.readline, ""):
        append(line)

def cal():
    p = subprocess.Popen(
        "NordenBombsight\\build\\NordenBombsight.exe", 
        
        stdout=subprocess.PIPE, 
        stdin=subprocess.PIPE, 
        stderr=subprocess.PIPE, 
        close_fds = True)
    poll = p.poll()
    if poll == None:
        print ("subprocess is alive")
    q = collections.deque(maxlen=1)
    #output = p.stdout.readline()
    t = threading.Thread(target=read_output, args=(p,q.append))
    #print ("here")
    t.daemon = True
    #print ("here2")
    t.start()
    #print ("here3")
    count = 0
    #while True:
    #    #stop_go = input("another frame? (y/n")
    #    print (q)
    #    print ("count: ", count)
    #    if count%1000 == 0:
    #        user = input("more frames? (enter for yes, n for terminate)")
    #        if user == 'n':
    #            print ("terminating....")
    #            break
                    
    #    count += 1
    while True:
        frame_data = b''.join(q).decode("utf-8")
        frame_data_list = frame_data.split()
        if frame_data_list:
            print (frame_data_list[1])
        else:
            print("no c++ input")
    p.kill()
    print ("terminated")
    #while True:
    #    output = p.stdout.readline()
    #    if output == '' and process.poll() is not None:
    #        break
    #    if output:
    #        print ("--------------------------------------------------------------")
    #        print ("python", output.strip() , "<---this is da wey")
    
def choose():
    while True:
        user_input = input("want to run an .exe? (y/n)")
        if user_input == "y":
            cal()
            break
        if user_input == "n":
            print("ok pussy")
            break

if __name__ == '__main__':
    choose()