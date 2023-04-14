from lidar import *
from turtle import *
import math
import time

#----Menu Iniziale----
print "Selezionare un comando:"
print "Comandi:"
print "polar  ->    visualizza coordinate polari"
print "reset  ->    fa un reset delle coordinate polari"
print "lidar  ->    visualizza profondita"
print "help   ->    lista comandi"
print "quit   ->    esce dal programma"

while(1):
    comando = raw_input("Comando> ")

    if comando == []:
        continue
        
    if comando == 'help':
        print ("Comandi:")
        print ("polar  ->    visualizza coordinate polari")
        print ("reset  ->    fa un reset delle coordinate polari")
        print ("lidar  ->    visualizza profondita")
        print ("help   ->    lista comandi")
        print ("quit   ->    esce dal programma")
    
    
    elif comando == 'polar':
        turtle = Turtlebot()
        turtle.open()
        turtle.setSpeeds(20,5)
        tempo = 0.0
        startTime = time.time()
        durata = 10
        while time.time() - startTime < durata:
            pose = turtle.getPose()
            raggio = math.sqrt(pose.x * pose.x + pose.y * pose.y)
            theta = math.atan2(pose.y,pose.x)
            print "Raggio:",raggio,"Theta:",theta
            time.sleep(1)
        turtle.setSpeeds(0,0)

    elif comando == 'reset':
        turtle = Turtlebot()
        turtle.open()
        turtle.setPose(0,0,0)



    elif comando == 'lidar':
        lidar = Lidar()
        lidar.scan()
        i = 0
        intensita, distanza = lidar.getData()
        for i in range(len(distanza)):
            print "Angolo:",359-i," intensita': ", int(intensita[i]), " distanza: ", int(distanza[i])
            
    elif comando == 'quit':
        sys.exit(0)
