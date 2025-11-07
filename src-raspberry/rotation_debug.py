from direct.showbase.ShowBase import ShowBase
from panda3d.core import Filename, AmbientLight, DirectionalLight, LVector3
from panda3d.core import Quat
import os
import time
import serial.tools.list_ports
import math

class MyApp(ShowBase):
    def moveModel(self, model, axis, value, repetitive):
            if repetitive:
                dt = globalClock.get_dt()
                quat.set_from_axis_angle(value * dt, axis)  
            else:
                quat.set_from_axis_angle(value, axis)
            model.set_quat(quat * model.get_quat()) 

    def valueForDegree(self, min, max, input):
        grad = 90
        if(input==0):
            return 0
        elif(input > 0):
            one_degree = max/grad # entspricht einem grad
            if(input/one_degree>grad):
                return grad
            else:
                return input / one_degree
        elif(input < 0):
            one_degree = min/grad # entspricht einem grad
            if(input/one_degree>grad):
                return -1*grad
            else:
                return -1*input / one_degree

    def gyroToDegree(self, one_rotation_value, input):
        one_degree = one_rotation_value/360
        return input/one_degree
        
    def lowpass(self, x, strength):
        start = 0
        start = strength * x + (1 - strength) * start
        return start

    def __init__(self):
        #Com ports
        ports = serial.tools.list_ports.comports()
        global serialInst
        serialInst = serial.Serial()

        #ports auflisten
        portsList = []
        for onePort in ports:
            portsList.append(str(onePort))
            print(str(onePort))

        #port auswählen
        #val = input("Select Port: COM")

        for x in range(0,len(portsList)):
            if portsList[x].startswith("COM" + str(9)):
                portVar = "COM" + str(9)
                print(portVar)

        serialInst.baudrate = 115200
        serialInst.port = portVar
        serialInst.open()

        #Fenster erzeugen
        ShowBase.__init__(self)

        # Schwarzer Hintergrund
        self.setBackgroundColor(0, 0, 0)

        # Pfad zur GLB-Datei im gleichen Ordner
        script_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(script_dir, "Eurofighter_Typhoon.glb")

        # Modell laden
        self.model = self.loader.load_model(Filename.from_os_specific(model_path))
        self.model.reparent_to(self.render)

        self.model2 = self.loader.load_model(Filename.from_os_specific(model_path))
        self.model2.reparent_to(self.render)
        
        # Quaternion test
        global model_x
        global model_y
        global model_z
        global quat

        quat = Quat()
        model_x = LVector3(1, 0, 0)  # Welt-X-Achse
        model_y = LVector3(0, 1, 0)  # Welt-Y-Achse
        model_z = LVector3(0, 0, 1)  # Welt-Z-Achse

        # Modell ausrichten (90° kippen, damit es richtig steht)
        self.model.set_hpr(0, 0, 0)
        self.moveModel(self.model, model_x, -90, False)

        self.model2.set_hpr(0, 0, 0)
        self.moveModel(self.model2, model_x, 90, False)
        self.moveModel(self.model2, model_y, 180, False)
        # Modell skalieren und positionieren
        self.model.set_scale(0.3)
        self.model.set_pos(0, 50, 0)
        self.model2.set_scale(0.3)
        self.model2.set_pos(0, 50, 5)

        #Beleuchtung hinzufügen
        ambient = AmbientLight("ambient")
        ambient.set_color((1, 1, 1, 1))
        self.render.set_light(self.render.attach_new_node(ambient))

        #directional = DirectionalLight("directional")
        #directional.set_color((1, 1, 1, 1))
        #directional_np = self.render.attach_new_node(directional)
        #directional_np.set_pos(0, 50, 50)
        #directional_np.look_at(0, 0, 0)
        #self.render.set_light(directional_np)

        # Task hinzufügen für kontinuierliche Rotation
        self.taskMgr.add(self.spin_task, "SpinTask")    # inaktiv
        self.taskMgr.add(self.init_task, "InitTask")
        # Bewegungsgeschwindigkeit
        self.move_speed = 10  # Einheiten pro Sekunde
        self.pitch_speed = 100 # -||-

        # Flags für gedrückte Tasten
        self.move_left_flag = False
        self.move_right_flag = False
        self.pitch_up_flag = False
        self.pitch_down_flag = False
        self.rotation_up_flag = False
        self.rotation_down_flag = False
        self.heading_up_flag = False
        self.heading_down_flag = False

        # Tastendrücke registrieren
        self.accept("a", self.set_flag, ["left", True])
        self.accept("a-up", self.set_flag, ["left", False])
        self.accept("d", self.set_flag, ["right", True])
        self.accept("d-up", self.set_flag, ["right", False])
        self.accept("w", self.set_flag, ["pitch_up", True])
        self.accept("w-up", self.set_flag, ["pitch_up", False])
        self.accept("s", self.set_flag, ["pitch_down", True])
        self.accept("s-up", self.set_flag, ["pitch_down", False])
        self.accept("arrow_left", self.set_flag, ["rotation_up", True])
        self.accept("arrow_left-up", self.set_flag, ["rotation_up", False])
        self.accept("arrow_right", self.set_flag, ["rotation_down", True])
        self.accept("arrow_right-up", self.set_flag, ["rotation_down", False])
        self.accept("arrow_up", self.set_flag, ["heading_up", True])
        self.accept("arrow_up-up", self.set_flag, ["heading_up", False])
        self.accept("arrow_down", self.set_flag, ["heading_down", True])
        self.accept("arrow_down-up", self.set_flag, ["heading_down", False])
        
        # Task für kontinuierliche Bewegung
        self.taskMgr.add(self.update_task, "UpdateTask")

    # Flags setzen
    def set_flag(self, name, value):
        if name == "left":
            self.move_left_flag = value
        elif name == "right":
            self.move_right_flag = value
        elif name == "pitch_up":
            self.pitch_up_flag = value
        elif name == "pitch_down":
            self.pitch_down_flag = value
        elif name == "rotation_up":
            self.rotation_up_flag = value
        elif name == "rotation_down":
            self.rotation_down_flag = value
        elif name == "heading_up":
            self.heading_up_flag = value
        elif name == "heading_down":
            self.heading_down_flag = value

    last = 0.0
    # What the sigma?
    float_values = [999.0, 999.0, 999.0, 999.0, 999.0, 999.0]
    string_values = [" "," "," "," "," "," "]
    string_value = " "
    counter = -1
    letzte = 0
    neue = 0
    i = 0
    w_deg = 0
    def spin_task(self, task):
        # Rotiert das Modell um die eigene Z-Achse
        angle_degrees = task.time * 0  # 30 Grad pro Sekunde
        return task.cont
    
    roll_rad = 0
    roll_deg = 0
    pitch_rad = 0
    pitch_deg = 0
    
    def init_task(self, task):

        quat_x = Quat()
        quat_x.set_from_axis_angle(90, self.model2.get_quat().xform(LVector3(1,0,0)).normalized())  # Rotation um X

        quat_y = Quat()
        quat_y.set_from_axis_angle(0, self.model2.get_quat().xform(LVector3(0,1,0)).normalized())  # Rotation um Y

        quat_z = Quat()
        quat_z.set_from_axis_angle(0, self.model2.get_quat().xform(LVector3(0,0,1)).normalized())  # Rotation um Z

        #Quaternions multiplizieren (Reihenfolge beachten!)
        final_quat = quat_y * quat_x * quat_z  # Reihenfolge kann an dein Modell angepasst werden

        current_quat = self.model2.get_quat()
        dq = Quat()

        dq.set_from_axis_angle(90, LVector3(0, 0, 1))
        
        return task.done

    # Task, der jedes Frame ausgeführt wird
     # Task: Bewegung und Pitch pro Frame
    def update_task(self, task):
        dt = globalClock.get_dt()
        x, y, z = self.model.get_pos()
        h, p, r = self.model.get_hpr()

        # X-Bewegung
        if self.move_left_flag:
            x -= self.move_speed * dt
        if self.move_right_flag:
            x += self.move_speed * dt

        # Pitch (Neigung) "s,w" 
        if self.pitch_up_flag:
            self.moveModel( self.model, model_z, 60, True) 

            self.moveModel( self.model2, model_z, 60, True)
            #p += self.pitch_speed * dt
        if self.pitch_down_flag:
            self.moveModel( self.model, model_z, -60, True) 

            self.moveModel( self.model2, model_z, -60, True)
            #p -= self.pitch_speed * dt

        # Rotation (drehung) "arrL,arrR"
        if self.rotation_up_flag:
            
            self.moveModel( self.model, model_x, 60, True)

            self.moveModel( self.model2, model_x, 60, True)
            #r += self.pitch_speed * dt
        if self.rotation_down_flag:
            self.moveModel( self.model, model_x, -60, True)

            self.moveModel( self.model2, model_x, -60, True)
            # -= self.pitch_speed * dt

        # Heading ((ausrichtung) "arrUP,arrDOWN"
        if self.heading_up_flag:
            self.moveModel( self.model, model_y, 60, True) 

            self.moveModel( self.model2, model_y, 60, True)  
            #h += self.pitch_speed * dt
        if self.heading_down_flag:
            self.moveModel( self.model, model_y, -60, True) 

            self.moveModel( self.model2, model_y, -60, True)   
            #h -= self.pitch_speed * dt
        
        # Rohdaten von Gyro und Accelerometer einlesen
        if serialInst.in_waiting:
            packet = serialInst.readline()
            self.string_value = packet.decode('utf').rstrip('\n')
           # print("string_value " + self.string_value)
            self.string_values = self.string_value.split(",")
           # print("packet " + packet.decode('utf').rstrip('\n'))
           
            for i in range(len(self.float_values)):
                try:
                    try:
                        if(i<3):
                            self.float_values[i] = float(self.string_values[i])
                        else:
                            self.float_values[i] = self.lowpass(float(self.string_values[i]),0.1)
                    except IndexError:
                        print("IndexError")
                except ValueError:
                    print("ValueError")
            
            """print(
                    f"{self.gyroToDegree(300, self.float_values[0]):.1f}",
                    f"{self.gyroToDegree(300, self.float_values[1]):.1f}",
                    f"{self.gyroToDegree(300, self.float_values[2]):.1f}",
                    f"{self.valueForDegree(-10, 10, self.float_values[3]):.1f}",
                    f"{self.valueForDegree(-10, 10,self.float_values[4]):.1f}",
                    f"{self.valueForDegree(-10, 10,self.float_values[5]):.1f}"
                ) 
            """
        
         #   float roll_rad, pitch_rad;
          #  float roll_deg, pitch_deg;

            self.roll_rad = math.atan2(self.float_values[4], self.float_values[5]);
            self.pitch_rad = math.atan2(-1* self.float_values[3], math.sqrt(self.float_values[4]*self.float_values[4] + self.float_values[5]*self.float_values[5]));

            self.roll_deg = self.roll_rad * 180.0 / math.pi
            self.pitch_deg = self.pitch_rad * 180.0 / math.pi
            print(self.roll_deg, ", ", self.pitch_deg)

        # Beispiel: deine Gyroskop-Werte
        z_deg = self.gyroToDegree(300, self.float_values[0])
        x_deg = self.gyroToDegree(300, self.float_values[1])
        y_deg = self.gyroToDegree(300, self.float_values[2])

        self.moveModel(self.model2, model_x, -1*x_deg, False)
        self.moveModel(self.model2, model_y, y_deg, False)
        self.moveModel(self.model2, model_z, -1*z_deg, False)
       
        return task.cont

# App starten
app = MyApp()
app.run()
