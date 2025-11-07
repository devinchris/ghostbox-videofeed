from logging import config
from direct.showbase.ShowBase import ShowBase
from panda3d.core import DirectionalLight, AmbientLight, Vec4
from panda3d.core import AntialiasAttrib
from panda3d.core import ColorBlendAttrib, Shader
import os
import time
from panda3d.core import TextNode, LVector3
from math import sin, cos, radians
import json

class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        
        # Hintergrund dunkelblau
        self.setBackgroundColor(0.0, 0.0, 0.0)
        
        # Kamera
        self.disable_mouse()
        self.camera.setPos(0, -40, 15)
        self.camera.lookAt(0, 0, 5)

        # Modell laden
        model = self.loader.loadModel("Eurofighter_Typhoon.glb", noCache=True)
        model.reparentTo(self.render)
        model.setScale(1.5)
        model.setPos(0, 20, -10)  # näher an Kamera
        model.setHpr(0,90, 0)  # Drehen, damit es nach vorne zeigt
        self.model = model
        
        self.cameraDistance = 60
        self.cameraAngleH = 0   # Horizontal (Yaw)
        self.cameraAngleV = 20  # Vertikal (Pitch)
        self.mouseLastX = None
        self.mouseLastY = None
        self.mouseDown = False

        self.taskMgr.add(self.updateCamera, "UpdateCamera")

        # Shader laden
        self.model.setShader(Shader.make(Shader.SL_GLSL, vertex_shader_code, fragment_shader_code))
        
        # Zeit für Shader-Animation
        self.start_time = time.time()
        self.taskMgr.add(self.update_shader_time, "UpdateShaderTime")

    

        # Antialiasing
        #self.render.setAntialias(AntialiasAttrib.MAuto)

        # Rotation
        #self.taskMgr.add(self.spinTask, "SpinTask")
        
        # === 3D-TEXT1 ERSTELLEN ===
        textNode = TextNode("3d_text")
        textNode.setTextColor(0.2, 1.0, 0.2, 1)  # Hellgrün
        textNode.setAlign(TextNode.ACenter)
        
        # Text1 in 3D Szene einfügen/  Daten
        textNodePath = self.render.attachNewNode(textNode)
        textNodePath.setScale(1.5)
        textNodePath.reparentTo(self.render)
        textNodePath.setPos(0, 100, 5)

        self.textNode = textNode
        
        self.speed =None
        self.direction = None
        self.temp = None
        self.air_pressure = None
        self.rotation =None
        self.gforce = None
        self.gradient = None
        
         # === VARIABLE AKTUALISIEREN ===
        
        self.taskMgr.add(self.lade_json_daten, "UpdateText")
        self.config_filepath ='config.json'
        self.last_modified_time = 0
        self.lade_json_daten()
        self.taskMgr.add(self.ueberwache_datei,"JSON_Ueberwachungs_Task")
        
        
        # InfoText_2
        infTex_2 = TextNode("infText")
        infTex_2.setWordwrap(20)
        infTex_2.setText("Eurofighter Typhoon:\n Modernes Mehrzweckkampfflugzeug Europas\nGeschwindigkeit über Mach 2, hohe Wendigkeit, modernste Avionik\nEinsatz: Luftüberlegenheit, Bodenangriffe, Aufklärung\n\nHENSOLDT-Technologien:\nCAPTOR-E Radar (AESA): Elektronisch gesteuertes Hochleistungsradar\nPraetorian DASS: Selbstschutzsystem gegen Radare und Raketen\nSensorfusion: Echtzeit-Verknüpfung aller Sensordaten\nEntwicklung zukünftiger Systeme: Eurofighter ECR & Future Combat Air System (FCAS)\nBeitrag zur europäischen Lufthoheit und technologischen Unabhängigkeit.")
        infTex_2.setAlign(TextNode.ALeft)
        infTex_2.setTextColor(0.2,1.0,0.2,1)
        
        infTex_2Path = self.aspect2d.attachNewNode(infTex_2.generate())
        infTex_2Path.setScale(1.5)
        infTex_2Path.reparentTo(self.render)
        infTex_2Path.setPos(40, 100, 10)
        
        # Ereignisse für Maus
        self.accept("mouse1", self.onMouseDown)
        self.accept("mouse1-up", self.onMouseUp)
        self.accept("wheel_up", self.onWheelUp)
        self.accept("wheel_down", self.onWheelDown)
        
        
       

    def spinTask(self, task):
        angle = (task.time * 5.0) % 360
        self.model.setH(angle)
        return task.cont

    def update_shader_time(self, task):
        t = time.time() - self.start_time
        self.model.setShaderInput("time", t)
        return task.cont

    
    # === Mausereignisse ===
    def onMouseDown(self):
        self.mouseDown = True
        if self.mouseWatcherNode.hasMouse():
            self.mouseLastX = self.mouseWatcherNode.getMouseX()
            self.mouseLastY = self.mouseWatcherNode.getMouseY()

    def onMouseUp(self):
        self.mouseDown = False

    def onWheelUp(self):
        self.cameraDistance = max(10, self.cameraDistance - 2)

    def onWheelDown(self):
        self.cameraDistance = min(200, self.cameraDistance + 2)

    # === Kamera-Update ===
    def updateCamera(self, task):
        if self.mouseDown and self.mouseWatcherNode.hasMouse():
            x = self.mouseWatcherNode.getMouseX()
            y = self.mouseWatcherNode.getMouseY()

            if self.mouseLastX is not None:
                dx = x - self.mouseLastX
                dy = y - self.mouseLastY
                self.cameraAngleH += dx * 100
                self.cameraAngleV -= dy * 100
                self.cameraAngleV = max(-80, min(80, self.cameraAngleV))  # Limit nach oben/unten

            self.mouseLastX = x
            self.mouseLastY = y

        # Kamera-Position berechnen (Orbit)
        radH = radians(self.cameraAngleH)
        radV = radians(self.cameraAngleV)
        x = self.cameraDistance * sin(radH) * cos(radV)
        y = -self.cameraDistance * cos(radH) * cos(radV)
        z = self.cameraDistance * sin(radV)

        self.camera.setPos(x, y, z)
        self.camera.lookAt(0, 0, 0)

        return task.cont
    
    

    def lade_json_daten(self,task=None):
        
        try:
            with open(self.config_filepath, 'r') as f:
                config = json.load(f)
                
         
                self.speed = float(config.get("speed",1.0))
                self.air_pressure=float(config.get("air_pressure",1.0))
                self.direction = int(config.get("direction",0))
                self.temp = float(config.get("temp",22.0))
                self.gradient = int(config.get("gradient",0))
                self.gforce = int(config.get("gforce",1))
                self.rotation = int(config.get ("rotation",0))
                
                self.textNode.setText(
                              f"""
                              Speed: {self.speed:6.1f} km/h\n
                              Temperatur:{self.temp:6.1f}°C\n
                              Direction: {self.direction:6.1f} °\n
                              Airpressure:{self.air_pressure:6.1f}bar\n
                              G-Force:{self.gforce:6.1f}G\n
                              Rotation:{self.rotation:6.1f}°\n
                              Gradient:{self.gradient:6.1f}%\n"""
                              )
                
        except FileNotFoundError:
            print(f"Warnung:' {self.config_filepath}'nicht gefunden.")
        except  json.JSONDecodeError:
            print(f"Fehler:Ungültiges JSON in '{self.config_filepath}")
            
    def ueberwache_datei(self,task):
        try: 
            mod_time = os.path.getmtime(self.config_filepath)
            if mod_time>self.last_modified_time:
                print("Änderung erkannt!")
                self.last_modified_time = mod_time
                self.lade_json_daten()
        except FileNotFoundError:
            pass
        return task.again
            
                


# === Shader-Code ===

vertex_shader_code = """
#version 130
uniform mat4 p3d_ModelViewProjectionMatrix;
in vec4 p3d_Vertex;
in vec3 p3d_Normal;
in vec2 p3d_MultiTexCoord0;

out vec2 texcoord;
out vec3 normal;
out vec3 worldPos;

void main() {
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
    texcoord = p3d_MultiTexCoord0;
    normal = normalize(p3d_Normal);
    worldPos = vec3(p3d_Vertex);
}
"""

# === Verbesserter Fragment-Shader-Code ===
fragment_shader_code = """
#version 130

in vec3 normal_eye;
in vec3 pos_eye;

uniform float time; // Zeit in Sekunden

// Farb-Uniforms (von außen steuerbar)
uniform vec3 color_black;
uniform vec3 color_mid;
uniform vec3 color_full;

uniform vec3 waveColor;     // Farbe der Welle (z. B. (0, 1, 0) für grün)
uniform float waveFrequency;
uniform float waveSpeed;
uniform float waveAmplitude;
uniform float alpha;

out vec4 fragColor;

void main() {
    vec3 view_dir = normalize(-pos_eye);
    float dotNV = dot(normal_eye, view_dir);

    // Winkelgrenzen in dot-Form
    float dot_0 = 1.0;
    float dot_30 = 0.866;
    float dot_70 = 0.342;
    float dot_90 = 0.0;

    vec3 final_color;

    if (dotNV >= dot_30) {
        final_color = color_black;
    }
    else if (dotNV >= dot_70) {
        float t = (dot_30 - dotNV) / (dot_30 - dot_70);
        final_color = mix(color_black, color_mid, t);
    }
    else if (dotNV >= dot_90) {
        float t = (dot_70 - dotNV) / (dot_70 - dot_90);
        final_color = mix(color_mid, color_full, t);
    }
    else {
        final_color = color_full;
    }

    // ADSS-Welle
    //float waveFrequency = 10.0;
    //float waveSpeed = 20.0;
    //float waveAmplitude = 0.3;
    //float wave = sin(pos_eye.x * waveFrequency + time * waveSpeed);
    //final_color.g += wave * waveAmplitude;

    
    float wave = sin(pos_eye.x * waveFrequency + time * waveSpeed);
    final_color += waveColor * wave * waveAmplitude;
    final_color = clamp(final_color, 0.0, 1.0);

    fragColor = vec4(final_color, alpha);
}
"""

app = MyApp()
app.run()

