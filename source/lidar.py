import time, sys, traceback, serial
import numpy as np

class Lidar:
    
    def __init__(self):
  
        self.port = "/dev/ttyUSB0"
        self.baud = 230400
        # Connessione alla porta seriale
        self.seriale = serial.Serial(self.port, self.baud, timeout=1)
        # Dati Relativi al LiDAR
        self.speedRPM = 0  # velocita di rotazione del lidar (RPM)
        self.intensita = np.zeros(360)  #array per memorizzare la distanza.
        self.distanza_mm = np.zeros(360)  # array per la memorizzazione l'intensita.
        

        print('----------------------------------------------------')
        print("Connessione alla porta seriale riuscita.")
        print('----------------------------------------------------')

    def compute_speed(self, data):
        
        # Funzionamento:
        # Il secondo simpbolo del package data e shiftato di 8 bit a SX
        # entrambi i simboli sono confrontati con l'operatore binario OR
        # il risultato e poi convertito in float e diviso per 64
        speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
        return speed_rpm

    def dataFromBytes(self, angle, data):
        """Interpreto i bytes in una forma leggibile"""
        # bytes individuali
        byte0 = data[0]
        byte1 = data[1]
        byte2 = data[2]
        byte3 = data[3]


        intensity = byte0 + (( byte1 & 0x3f) << 8) 
        dist_mm = byte2 + (byte3 << 8) 
        self.intensita[int(angle)] = intensity
        self.distanza_mm[int(angle)] = dist_mm
        
        
    def scan(self, durata=1):
		
        interpLevel = 0  # Un contatore che indica a che livello di interpretazione siamo
        positionIdx = 0  # Un indice per calcolare l'angolo della misura
        startTime = time.time()  # Inizio del tempo per controllare la lunghezza dello scan
        while time.time() - startTime < durata:
            try:
                time.sleep(0.00001)
                if interpLevel == 0:
                    # Legge un singolo byte
                    byte  = ord(self.seriale.read(1))
                    
                    # Controlla se il byte e un byte iniziale
                    if byte == 0xFA:
                        interpLevel = 1
                    else:
                        interpLevel = 0
                elif interpLevel == 1:
                    # Position index
                    byte = ord(self.seriale.read(1))
                    if byte >= 0xA0 and byte <= 0xDB:
                        positionIdx = byte - 0xA0
                        interpLevel = 2
                    # Nel caso il byte non e' tra 160 e 249,
                    # e non e' un byte di inizio,
                    # inizializziamo init_level
                    elif byte != 0xFA:
                        interpLevel = 0
                elif interpLevel == 2:
                    # Speed byte
                    byte_speed = [ord(byte) for byte in self.seriale.read(2)]
                    # Data bytes
                    byte_data0 = [ord(byte) for byte in self.seriale.read(6)]
                    byte_data1 = [ord(byte) for byte in self.seriale.read(6)]
                    byte_data2 = [ord(byte) for byte in self.seriale.read(6)]
                    byte_data3 = [ord(byte) for byte in self.seriale.read(6)]
                    byte_data4 = [ord(byte) for byte in self.seriale.read(6)]
                    byte_data5 = [ord(byte) for byte in self.seriale.read(6)]
            
                    self.speedRPM = self.compute_speed(byte_speed)
                    self.dataFromBytes(positionIdx * 6 + 0, byte_data0)
                    self.dataFromBytes(positionIdx * 6 + 1, byte_data1)
                    self.dataFromBytes(positionIdx * 6 + 2, byte_data2)
                    self.dataFromBytes(positionIdx * 6 + 3, byte_data3)
                    self.dataFromBytes(positionIdx * 6 + 4, byte_data4)
                    self.dataFromBytes(positionIdx * 6 + 5, byte_data5)        
                    # Inizializziamo il contatore di livello e di posizione, in attesa del prossimo package di byte
                    interpLevel = 0
                    positionIdx = 0
            except:
                traceback.print_exc(file=sys.stdout)
                break
            

    def getData(self):
        # Ritorna l'array corrente del LiDar
        return self.intensita,self.distanza_mm
