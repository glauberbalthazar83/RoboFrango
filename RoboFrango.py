####################################################################
##### Source code for the 6th Prototype: RoboFrango (RoboChicken) ##
############### Autor: Glauber da Rocha Balthazar ##################
####################################################################

import RPi.GPIO as GPIO        
import time
import cv2
import numpy as np
import sys
import os
from threading import Thread
from rplidar import RPLidar, RPLidarException
import matplotlib.pyplot as plt
import statistics
import math
from datetime import datetime
from pyzbar.pyzbar import decode

#--PARAMETROS GLOBAIS--
#motorizacao
in1 = 35 #vermelho24
in2 = 36 #laranja23
enA = 33 #marrom25
in3 = 37 #amarelo22
in4 = 38 #verde27f
enB = 32 #azul17

#camera
cap=cv2.VideoCapture(0)
# Pega as propriedades do vídeo
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
# Variáveis para controlar o zoom
zoom_factor = 1.0
zoom_step = 0.1

#motor_atuador
in1_atuador = 7 #verde
in2_atuador = 11 #branco

#-------SETUP----------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

#------MOTORIZACAO-----
#motores
GPIO.setup(enA,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enB,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
GPIO.output(enA,GPIO.HIGH)
GPIO.output(enB,GPIO.HIGH)

#-------LIDAR------
PORT_NAME = '/dev/ttyUSB0'
file1 = open("distancias.txt", "w")
texto = "MF;MT;MD;ME;S\n"
file1.write(texto)
file1.close() 
#FLAG para Jetson Camera
pinoFLAG = 23
GPIO.setup(pinoFLAG, GPIO.OUT, initial=GPIO.HIGH) 

#FLAG para Sensor Ambiental
pinoSensorAmbientalFLAG = 24
GPIO.setup(pinoSensorAmbientalFLAG, GPIO.OUT, initial=GPIO.HIGH)
vetorPosicoesSensoriamento = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]

#DIRETORIOS
capturas_dir = 'capturas'
video_dir = os.path.join(capturas_dir, 'video')
fotos_dir = os.path.join(capturas_dir, 'fotos')

#----------------------------------FUNCOES-------------------------------------------  
def stop():
    print("stop")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)          
#------------------------------------------------------------------------------------
def toForward():
    print("forward")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW) 
    time.sleep(4)
#------------------------------------------------------------------------------------
def toBackward():
    print("backward")
    GPIO.output(in1,GPIO.HIGH)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.HIGH)
#------------------------------------------------------------------------------------
def toLeft():
    print("left")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.HIGH)
    GPIO.output(in3,GPIO.LOW)
    GPIO.output(in4,GPIO.LOW)
#------------------------------------------------------------------------------------
def toRight():
    print("right")
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.output(in3,GPIO.HIGH)
    GPIO.output(in4,GPIO.LOW)      
#------------------------------------------------------------------------------------
def task_camera():
    now = datetime.now()
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output_filename = os.path.join(video_dir, f"output_video_{current_time}.avi")
    out = cv2.VideoWriter(output_filename,fourcc, 15.0, (640,480))

    # Tempo para controle das capturas
    tempo_ultima_captura = time.time()

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            out.write(frame)#gravando o video
            # Intervalo de captura (15 segundos)
            intervalo_captura = 5
            # Verificar se é hora de fazer uma nova captura
            tempo_atual = time.time()
            if tempo_atual - tempo_ultima_captura >= intervalo_captura:
                current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                image_filename = os.path.join(fotos_dir, f"captura_{current_time}.jpg")
                cv2.imwrite(image_filename, frame)
                tempo_ultima_captura = tempo_atual

            #capturar QR-Code
            for barcode in decode(frame):
                mydata = barcode.data.decode('utf-8')
                if(mydata=='Linha1Ponto1' and vetorPosicoesSensoriamento[0]==-1):
                    print('achou ponto 11')
                    vetorPosicoesSensoriamento[0]=1
                if(mydata=='Linha1Ponto2' and vetorPosicoesSensoriamento[1]==-1):
                    print('achou ponto 12')
                    vetorPosicoesSensoriamento[1]=1
                if(mydata=='Linha1Ponto3' and vetorPosicoesSensoriamento[2]==-1):
                    print('achou ponto 13')  
                    vetorPosicoesSensoriamento[2]=1        
                if(mydata=='Linha2Ponto1' and vetorPosicoesSensoriamento[3]==-1):
                    print('achou ponto 21')
                    vetorPosicoesSensoriamento[3]=1
                if(mydata=='Linha2Ponto2' and vetorPosicoesSensoriamento[4]==-1):
                    print('achou ponto 22')
                    vetorPosicoesSensoriamento[4]=1
                if(mydata=='Linha2Ponto3' and vetorPosicoesSensoriamento[5]==-1):
                    print('achou ponto 23') 
                    vetorPosicoesSensoriamento[5]=1  
                if(mydata=='Linha3Ponto1' and vetorPosicoesSensoriamento[6]==-1):
                    print('achou ponto 31')
                    vetorPosicoesSensoriamento[6]=1
                if(mydata=='Linha3Ponto2' and vetorPosicoesSensoriamento[7]==-1):
                    print('achou ponto 32')
                    vetorPosicoesSensoriamento[7]=1
                if(mydata=='Linha3Ponto3' and vetorPosicoesSensoriamento[8]==-1):
                    print('achou ponto 33')  
                    vetorPosicoesSensoriamento[8]=1  
                if(mydata=='Linha4Ponto1' and vetorPosicoesSensoriamento[9]==-1):
                    print('achou ponto 41')
                    vetorPosicoesSensoriamento[9]=1
                if(mydata=='Linha4Ponto2' and vetorPosicoesSensoriamento[10]==-1):
                    print('achou ponto 42')
                    vetorPosicoesSensoriamento[10]=1
                if(mydata=='Linha4Ponto3' and vetorPosicoesSensoriamento[11]==-1):
                    print('achou ponto 43')
                    vetorPosicoesSensoriamento[11]=1     
                if(mydata=='Linha5Ponto1' and vetorPosicoesSensoriamento[12]==-1):
                    print('achou ponto 51')
                    vetorPosicoesSensoriamento[12]=1
                if(mydata=='Linha5Ponto2' and vetorPosicoesSensoriamento[13]==-1):
                    print('achou ponto 52')
                    vetorPosicoesSensoriamento[13]=1
                if(mydata=='Linha5Ponto3' and vetorPosicoesSensoriamento[14]==-1):
                    print('achou ponto 53')  
                    vetorPosicoesSensoriamento[14]=1                                                                                        
                pts = np.array([barcode.polygon],np.int32)
                pts = pts.reshape((-1,1,2))
                cv2.polylines(frame,[pts],True,(255,0,255),5)
                pts2 = barcode.rect
                cv2.putText(frame,mydata,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.9,(255,0,255))

            cv2.imshow('frame',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()
#------------------------------------------------------------------------------------    
def movimentar():
    print("\n")
    print("The default speed & direction of motor is LOW & Forward.....")
    print("s-stop f-forward b-backward l-left r-right d-down e-exit")
    print("\n")    

    while(1):

        x=input()
        
        if x=='s':
            stop()     
            x='z'

        elif x=='f':    
            toForward()    
            temp1=1
            x='z'

        elif x=='b':
            toBackward()
            temp1=0
            x='z'

        elif x=='l':
            toLeft()
            temp1=0
            x='z'

        elif x=='r':
            toRight()
            temp1=0
            x='z'
        
        elif x=='e':
            GPIO.cleanup()
            break
        
        else:
            print("<<<  wrong data  >>>")
            print("please enter the defined data to continue.....")
#----------------------------------MAIN----------------------------------------------   
def lidarRP1():
    print(f'Initializating RPLidar A1 <port: {PORT_NAME}>....')
    lidar = RPLidar(PORT_NAME, baudrate=115200)
    
    while(1):
        try:
            print('Press Crl+C to stop.')
            i=0
            global med_frente, med_tras, med_direita, med_esquerda
            med_frente = 0
            med_tras = 0
            med_direita = 0
            med_esquerda = 0
            for scan in lidar.iter_scans(max_buf_meas=5000):#(qualidade, angulo, distancia)
                i+=1
                values_frente = []
                values_tras = []
                values_direita = []
                values_esquerda = []
                x=[]
                y=[]
                for _ in range(360):
                    x.append(0)
                    y.append(0)
                    
                for t in scan:
		  angle = int(t[1]) #iterations read angle sensor LiDAR
		  if(angle<360):
		    x[angle] = int(t[2]) * math.cos(math.radians(angle))
		    y[angle] = int(t[2]) * math.sin(math.radians(angle))
		  if(t[1]>=335 or t[1]<=25):  #50º from an angle forward
		    values_forward.append(t[2])
		  if(t[1]>=155 and t[1]<=205): #50º from an angle to the back
		    values_backward.append(t[2])
		  if(t[1]>=206 and t[1]<=324): #118º from an angle to the right 
		    values_right.append(t[2])
		  if(t[1]>=26 and t[1]<=154):  #118º from an angle to the left
		    values_left.append(t[2])
		# statistics (median of set angle) determine decision to direction of robot
		  if len(values_forward)>0:
		    med_forward = statistics.median(values_forward)
		 if(med_forward>=20):
		    toForward()
		  if len(values_backward)>0:
		    med_backward = statistics.median(values_backward)
		 if(med_backward >=20):
		    toBackward()
		  if len(values_right)>0:
		    med_right = statistics.median(values_right)
		 if(med_right >=45 and med_right <=55):
		    toRight()
		  if len(values_left)>0:
		    med_left = statistics.median(values_left)
		 if(med_left >=25 and med_left <=35):
		   toLeft()
 
 		#to manual movement
                print("Situacao (s-stop; f-forward; b-backward; l-left; r-right; p-flag(Camera); j-continue)): ")
                
                situacao = input()

                if situacao=='s':
                    stop()  
                    temp1=1   

                elif situacao=='f':    
                    toForward()    
                    temp1=1

                elif situacao=='b':
                    toBackward()
                    temp1=0

                elif situacao=='l':
                    toLeft()
                    temp1=0

                elif situacao=='r':
                    toRight()
                    temp1=0
                
                elif situacao=='p':
                    enviarFlag()
                    temp1=0  

                elif situacao=='j':
                    print("Continue moviment")
                    temp1=0                            

                file1 = open("distancias.txt", "a")
                #texto = "MF;MT;MD;ME;S"
                texto = str(med_frente)+";"+str(med_tras)+";"+str(med_direita)+";"+str(med_esquerda)+";"+str(situacao)+str('\n')
                file1.write(texto)
                file1.close() 
                situacao = 'z'   
                time.sleep(2)  
                stop()       
                print('Gravando distancia \n-----------------------------------------')

        except KeyboardInterrupt:
            print('Stoping.')
        except RPLidarException:
            print('Stoping.')
            lidar.stop()
            lidar.disconnect()
            lidarRP1()
        lidar.stop()
        lidar.disconnect()
#------------------------------------------------------------------------------------   
def enviarFlag():
    GPIO.output(pinoFLAG, GPIO.HIGH) 
    print("FLAG is ON")
    time.sleep(3) 
    GPIO.output(pinoFLAG, GPIO.LOW)
    print("FLAG is OFF")
#------------------------------------------------------------------------------------  
def ligarSensorAmbiental():
    GPIO.output(pinoSensorAmbientalFLAG, GPIO.LOW)
    print("SensorAmbietal is ON")
    print("Ao ligar o SensorAmbiental deve-se aguardar 3min para sua execucao.")
#------------------------------------------------------------------------------------  
def desligarSensorAmbiental():
    GPIO.output(pinoSensorAmbientalFLAG, GPIO.HIGH)
    print("SensorAmbietal is OFF")    
#------------------------------------------------------------------------------------   
def criar_diretorios():
    # Diretório para salvar as imagens capturadas e os vídeos
    if not os.path.exists(capturas_dir):
        os.makedirs(capturas_dir)

    if not os.path.exists(video_dir):
        os.makedirs(video_dir)

    if not os.path.exists(fotos_dir):
        os.makedirs(fotos_dir)
#------------------------------------------------------------------------------------ 
if __name__ == '__main__':
    #criar diretorios
    criar_diretorios()

    #thread da camera
    t = Thread(target = task_camera)
    t.daemon = True
    t.start()    

    try:
        while True:
            lidarRP1()
            #movimentar()
            
    except KeyboardInterrupt:
        print("CTRL + C pressed")
        stop()
        GPIO.cleanup()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
        finally:
            stop()
            #ao termino, liberar a GPIO
            GPIO.cleanup()
    except:
        print("Except while")
    finally:
        stop()
        #ao termino, liberar a GPIO
        GPIO.cleanup()