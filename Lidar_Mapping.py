"""Webots_Lidar_Mapping controller."""

from controller import Supervisor
from controller import Keyboard

import math

import matplotlib.pyplot as plt
import numpy as np

# Supervisor objesi oluşturuldu.
robot = Supervisor()

timestep = 64 # simulasyon adım süresi

superRobot = robot.getFromDef("superRobot")
rotation_field = superRobot.getField("rotation") # robotun rotation alanına ulaşmak için getField metodu kullanıldı.

# robotun gideceği konumu belirten hedef objesi webots içerisinden alındı
hedef = robot.getFromDef("Hedef")

#Keyboard
keyboard = Keyboard()
keyboard.enable(timestep)
manualControl = {
    "active": True,
    "count": 0
}

#region #LİDAR TANIMLAMASI
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()
#endregion

#region #Tekerlek motorlarının tanımlanması--------------------------
left_motor = robot.getDevice("motor_2")
right_motor = robot.getDevice("motor_1")

left_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)

right_motor.setPosition(float("inf"))
right_motor.setVelocity(0.0)
#endregion #---------------------------------------------------------

#region Tekerleklerdeki Encoderin tanımlanması-----------------------
left_ps = robot.getDevice("ps_2")
left_ps.enable(timestep)

right_ps = robot.getDevice("ps_1")
right_ps.enable(timestep)

ps_values = [0, 0] # encoder verilerinin tutulacağı liste
#endregion #---------------------------------------------------------

#region #Encoder blgisinin lineer ilerlemeye çevrilmesi
dist_values = [0, 0] # tekerleklerin lineer ilerleme mesafelerinin kaydedileceği liste(metre)
wheel_radius = 0.025 # metre. tekerlek yarıçapı

distance_between_wheels = 0.09

wheel_circum = 2 * 3.14 * wheel_radius # tekerleğin çevresi hesaplandı.
encoder_unit = wheel_circum / 6.28 # tekerleğin çevre uzunluğu 6.28(360 derecenin radyan cinsi) değerine bölünerek 1 radyanlık doğrusal uzunluk bilgisi hesaplandı
# encoder radyan cinsinden değeri arttığı için 1 radyan değeri ile tekerleğin gittiği doğrusal mesafe bilgisi hesaplandı
#endregion

# Robot pozu
# Başlangıç konumu orjin kabul edilirse encoder ile hesaplanan başlangıçtaki konum [0, 0, 0] olur
## robot_pose_encoder = np.zeros((3, 1)) #[0, 0, 0] # x, y, theta
def get_robot_pose_from_webots():
    robot_pos = superRobot.getPosition()
    robot_rot = rotation_field.getSFRotation()

    # Robot 1 turluk dönüşü tamamladığında z eksen değeri -1 değerini alıp açı bilgisini negatif yapıyor.
    # Robotun z eksendeki dönüşünden elde edilen açının doğru olması için z eksen değeri 1(pozitif) tutulmalı.
    axis_z = robot_rot[2]
    robot_rot_z = round(robot_rot[3], 3)    
    if(axis_z < 0): 
        robot_rot_z = robot_rot_z * -1
    else:
        robot_rot_z = round(robot_rot[3], 3)
    #---------------------------------------------------------------------------------------------------------

    robot_pose = np.array([
        [round(robot_pos[0], 3)],
        [round(robot_pos[1], 3)],
        [robot_rot_z],
    ])

    return robot_pose

robot_pose_encoder = get_robot_pose_from_webots()
last_ps_values = [0, 0] # robotun pozisyon bilgisini hesaplamak için en son encoder verisinin kaydedileceği liste tanımlandı.

show_animation = True

# odometry ile hesaplanan pozisyon bilgisinde gürültü oluşuyor. Bu nedenle daha yakın sonuç elde etmek için round kullanmayan(virgülü yuvarlamadan) odometry2 fonk tanımlandı.
def odometry2(keycode):    
    ps_values[0] = left_ps.getValue() # getValues metodu ile sol encoderin ürettiği bilgi alındı.(radyan cinsinden)
    ps_values[1] = right_ps.getValue()
    
    print("----------------ODOMETRY----------------")   
    print("position sensor values(Left, Right): {} {}" .format(round(ps_values[0], 3), round(ps_values[1], 3)))
        
    for ind in range(2):
        diff = ps_values[ind] - last_ps_values[ind]      
        #if(diff < 0.001):
            #diff = 0
            #ps_values[ind] = last_ps_values[ind]
        dist_values[ind] = diff * encoder_unit # encoder bilgisi sırasıyla alınarak mesafe bilgisine çevrildi ve dist_values listesine kaydedildi.
   
    print("distance values(L, R): {} {}" .format(round(dist_values[0], 3), round(dist_values[1], 3)))
    
    # print("diff: ",diff) # timestep süresinden tekerlerin radyan cinsinden dönüş miktarı(açısal hız radyan / timestep)  
        
    # robotun lineer ve açısal hızlarının hesaplanması-----------------------
    v = (dist_values[0] + dist_values[1]) / 2.0
    w = (dist_values[0] - dist_values[1]) / distance_between_wheels
    
    print("v(m/timestep)= {}, w(rad/timestep)= {}".format(round(v, 3), round(w, 3)))        

    Vms = 1000 * v / timestep # hız metre/saniye cinsine çevrildi(simülasyondaki timestep değeri döngünün çalıştırıldığı süreyi verir. Bu süre milisaniye cinsinden olduğu için 1000 ile çarparak saniye cinsine çevrildi)
    Wrs = 1000 * w / timestep # açısal hız radyan/saniye cinsine çevrildi(simülasyondaki timestep değeri döngünün çalıştırıldığı süreyi verir. Bu süre milisaniye cinsinden olduğu için 1000 ile çarparak saniye cinsine çevrildi)
    print("v(m/s)= {}, w(rad/s)= {}".format(round(Vms, 3), round(Wrs, 3)))

    #region #Encoder bilgisi ile robot poz'unun hesaplanması    
    dt = 1 #dt # 1
    robot_pose_encoder[2] = robot_pose_encoder[2] + (w * dt) #- (w*0.1) # webots içerisinde robot dönerken oluşan hatayı azaltmak için açışal hız değeri açısal hızın 0.1 katı ile çıkartıldı. Bu sayede gerçek değer elde edildi.
    robot_pose_encoder[2] = ((robot_pose_encoder[2, 0] + math.pi) % (2 * math.pi) - math.pi)#(robot_pose_encoder[2, 0] + math.pi) % (2 * math.pi) - math.pi, 3) # Zamanla artan açı bilgisi 0 / 3.14 ve -3.14 / 0 aralığına ayarlandı 

    vx = v * math.cos(robot_pose_encoder[2])
    vy = v * math.sin(robot_pose_encoder[2])
    
    robot_pose_encoder[0] = robot_pose_encoder[0, 0] + (vx * dt)#round(robot_pose_encoder[0, 0] + (vx * dt), 3)
    robot_pose_encoder[1] = robot_pose_encoder[1, 0] + (vy * dt)#round(robot_pose_encoder[1, 0] + (vy * dt), 3)
    
    print("robot_pose_encoder(x, y, theta): {}" .format(robot_pose_encoder))    
    #endregion    
    
    last_ps_values[0] = ps_values[0] # bir önceki encoder bilgisi anlık encoder bilgisine eşitlendi
    last_ps_values[1] = ps_values[1] # bir önceki encoder bilgisi anlık encoder bilgisine eşitlendi
    
    print("----------------------------------------")
    return v, w, Vms, Wrs, robot_pose_encoder
    #--------------------------------------------------------------------------------

def get_robot_pose_from_webots():
    robot_pos = superRobot.getPosition()
    robot_rot = rotation_field.getSFRotation()

    # Robot 1 turluk dönüşü tamamladığında z eksen değeri -1 değerini alıp açı bilgisini negatif yapıyor.
    # Robotun z eksendeki dönüşünden elde edilen açının doğru olması için z eksen değeri 1(pozitif) tutulmalı.
    axis_z = robot_rot[2]
    robot_rot_z = round(robot_rot[3], 3)    
    if(axis_z < 0): 
        robot_rot_z = robot_rot_z * -1
    else:
        robot_rot_z = round(robot_rot[3], 3)
    #---------------------------------------------------------------------------------------------------------

    robot_pose_webots = np.array([
        [round(robot_pos[0], 3)],
        [round(robot_pos[1], 3)],
        [robot_rot_z],
    ])    

    return robot_pose_webots

def keyboard_control():
    keycode = keyboard.getKey() # webots içerisinde simulasyon çalışırken klavyenin tuşlarına basıldığında değer ureten fonk.
    print("key: ", keycode)
        # a & A: 65
        # s & S: 83
        # d & D: 68
        # w & W: 87
        # o & O: 79
        # m & M: 77
        # n & N: 78
        # h & H: 72
        # t & T: 84
        # y & Y: 89
    if(keycode == 87): # eğer w tuşuna basılırsa(robot ileri gidecek)
        keys["w"] = True        
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
        keys["y"] = False
    elif(keycode == 65): # eğer a tuşuna basılırsa(robot sola gidecek)
        keys["a"] = True
        keys["w"] = False        
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 83): # eğer s tuşuna basılırsa(robot geriye gidecek)
        keys["s"] = True
        keys["w"] = False
        keys["a"] = False        
        keys["d"] = False
        keys["o"] = False
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 68): # eğer d tuşuna basılırsa(robot sağa gidecek)
        keys["d"] = True
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False        
        keys["o"] = False
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 79):
        keys["o"] = True
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False        
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 77):
        keys["m"] = True
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 78):
        keys["m"] = False
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["n"] = True
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 72):
        keys["m"] = False
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["n"] = False
        keys["h"] = True
        keys["t"] = False
        keys["y"] = False
    elif(keycode == 84):
        keys["m"] = False
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = True
        keys["y"] = False
    elif(keycode == 89):
        keys["m"] = False
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = True
    else:
        keys["w"] = False
        keys["a"] = False
        keys["s"] = False
        keys["d"] = False
        keys["o"] = False
        keys["m"] = False
        keys["n"] = False
        keys["h"] = False
        keys["t"] = False
        keys["y"] = False

    return keycode

def robotControl(vL, vR, isGoal, d, path, current_time):
    # Manuel veya otonom sürüş için | manuelControl Değişkeninin active değeri True ise mauel sürüş değilse otonom sürüş moduna geçer
    #klavyenin o tuşuna bir kez basıldığında manuel kontrol aktif, ikinci kez basıldığında otonom sürüş aktif oluyor.
    if (keys["o"]): # o tuşuna basıldığında robotun manuel kontrolü açılıp kapatılması sağlanıyor.
        manualControl["active"] = True
        manualControl["count"] = manualControl["count"] + 1
        if (manualControl["count"] % 2 == 0):
            manualControl["active"] = False
            manualControl["count"] = 0

    # Klavye tuşlarından(w a s d tuşları) robotun hareket ettirilmesi için********** 
    if (keys["a"] and manualControl["active"]):
        left_motor.setVelocity(1)
        right_motor.setVelocity(-1)        
    elif (keys["d"] and manualControl["active"]):
        left_motor.setVelocity(-1)
        right_motor.setVelocity(1)        
    elif (keys["w"] and manualControl["active"]):
        left_motor.setVelocity(5)
        right_motor.setVelocity(5)        
    else:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
    #*******************************************************************************
    
    #Dynamic Window Aproac için otonom sürüş sistemi
    if(manualControl["active"] == False):
        
        if(isGoal == True or current_time < 3): # FAST SLAM Başlangıçta 3 saniye beklesin diye "current_time < 3" eklendi.
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
        else:
            # print("das1")
            left_motor.setVelocity(vL)
            right_motor.setVelocity(vR)

        hedef_path = [[-0.75, 0.25, 0.01],
                        [0.25, 0.25, 0.01],
                        [0.25, 0.75, 0.01],
                        [-0.75, 0.75, 0.01],
                        # [-0.50, 0.75, 0.01],
                        [0.25, 0.70, 0.01],
                        [0.25, 0.25, 0.01],
                        [-0.75, 0.25, 0.01],
                        [-0.75, -0.75, 0.01],
                        [-0.25, -0.75, 0.01],
                        [-0.25, -0.25, 0.01],
                        [0.75, -0.25, 0.01],
                        [0.75, 0.75, 0.01],
                        [0.75, -0.25, 0.01],
                        [-0.25, -0.25, 0.01],
                        [-0.25, -0.75, 0.01],
                        [0.75, -0.75, 0.01],
                        ]

        print("d: ", d)
        print("path: ", path["path"])
        if((keys["h"] or d <= 0.15) and isGoal):
            hedef_trans = hedef.getField("translation")
            hedef_trans.setSFVec3f(hedef_path[path["path"]]) # [x, y, z]
        
            if(path["path"] < (len(hedef_path) - 1)):
                path["path"] += 1

    return path["path"], d
        
def calculate_wheel_speed(v, w):
    vL = round(((2 * v) + (w * distance_between_wheels)) / (2 ), 3) # sol tekerin m/s cinsinden dönüşü
    print("Angular Velocity(L): ", vL / wheel_radius) # sol tekerin rad/s cinsinden dönüşü
    vL = vL / wheel_radius
    l = left_motor.getVelocity() # sol tekerleğin gerçek rad/s cinsinden açısal hız değeri
    # print("getVelocity(L): ", l)

    vR = round(((2 * v) - (w * distance_between_wheels)) / (2 ), 3)
    print("Angular Velocity(R): ", vR / wheel_radius)
    vR = vR / wheel_radius

    r = right_motor.getVelocity() # sağ tekerleğin gerçek rad/s cinsinden açısal hız değeri
    # print("getVelocity(L): ", r)
    return vL, vR

def get_LidarPoints_from_webots():
    pointCloud = lidar.getRangeImage() # lidar sensöründen veri alındı ve pointCloud içerisine kaydedildi
    # print("pointCloud: ", pointCloud[:5])
    # anlık pointCloud verisini almak için z döngü içerisinde sıfırlanmalı
    z = np.zeros((0, 2)) 
    angle_i = np.zeros((0, 1))

    for i in range(len(pointCloud)):
        angle = ((len(pointCloud) - i) * 0.703125 * math.pi / 180) + robot_pose_webots[2, 0] + w*0.1 # w(döngü süresi içerisindeki robotun açısal hız değeri) ile toplanarak robotun dönmesi sırasında lidar noktalarının kayması azaltıldı. *0.1 değeri ile robotun dönme sırasında lidar noktalarının kayma miktarı azaltıldı.
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        
        vx = v * math.cos(robot_pose_webots[2]) # robot hareket ettikçe alınan lidar noktaları da hareket ediyor. Bu nedenle robotun x ve y hızları hesaplanarak lidar noktalarına eklenmeli.
        vy = v * math.sin(robot_pose_webots[2])
        # print("vx: ", vx)
        ox = round((math.cos(angle) * pointCloud[i] + robot_pose_webots[0, 0] + vx*0.1), 3)
        oy = round((math.sin(angle) * pointCloud[i] + robot_pose_webots[1, 0] + vy*0.1), 3)
        
        zi = np.array([ox, oy]) # tek bir noktanın x ve y değeri
        
        # Sadece anlık pointcloud verisini almak için
        z = np.vstack((z, zi)) # bütün anlık lidar noktalarının x ve y değerleri z dizisi içerisine kaydedildi.

        angle_i = np.vstack((angle_i, angle)) # tek bir noktanın açı değeri
   
    z_points = np.zeros((0, 2))
    # AŞAĞIDAKİ for döngüsü ile pointcloud verisi içerisindeki inf(infinity) değerleri temizlendi.(sadece engelleri belirten noktalar alındı)
    for i in z:
        # print("İ: ", i[0])
        if(i[0] == np.inf or i[0] == -np.inf):            
            continue
        z_points = np.vstack((z_points, i)) # sadece engelleri algılayan lidar noktalarının x ve y değerleri z_points dizisi içerisine eklendi
    # print("z_points: ", z_points)

    return z, z_points, angle_i, pointCloud

if __name__ == '__main__':
    print("BAŞLATILIYOR...")
    
    keys = {
        "w": False,
        "a": False,
        "s": False,
        "d": False,
        "o": False,
        "m": False,
        "h": False,
        "t": False,
        "y": False,
    }    
    robot_pose_webots = get_robot_pose_from_webots()
        
    # harita dizileri
    map = np.zeros((0, 2)) #np.zeros((0, 3)) # pointCloud verisi ile harita oluşturmak için z döngüden önce tanımlanmalı
    
    dongu = 0
    ort = 0
    # Robotun otonom olarak gitmesini sağlayan konumların anlık sıra değeri path dict list içerisinde kaydediliyor.
    path = {
        "path": 0
    }

    previous_time = 0 # geçen sürenin hesaplanması için kullanılıyor.
    time = 4 # harita kaydetmek gibi işlemlerin yapılması için beklenen süre
    
    hTruePos = np.zeros((3, 0)) # robotun webots içerisindeki gerçek konumların kaydedildiği dizi
    hEncoPos = np.zeros((3, 0)) # encoder ile hesaplanan robot konumlarının kaydedileceği dizi

    isGoal = False

    while robot.step(timestep) != -1:
        current_time = robot.getTime() # simülasyonda anlık zaman bilgisi alındı
        print("Time: ",current_time)
        print("TimePrev: ",previous_time) # bir önceki simülasyon döngü zamanı
       
        keycode = keyboard_control()
        print("manualControl: ",manualControl)

        v, w, Vms, Wrs, robot_pose_encoder = odometry2(keycode)        
        hEncoPos = np.hstack((hEncoPos, robot_pose_encoder)) # encoder ile hesaplanan robot konumu robot hareket ettikçe hEncoPos dizisine kaydedilir.
        print("hEncoPos kaydedildi")

        # lidar verisinin alınması
        robot_pose_webots = get_robot_pose_from_webots()
        z, z_points, angle, pointDistance = get_LidarPoints_from_webots()

        #HARİTA oluşturulması ve güncellenmesi ve Kaydedilmesi
        if((time < previous_time or len(z_points) == 0)): 
            # belirtilen zaman aralıklarında veya oluşturulan harita henüz yoksa ise anlık lidar verisi harita oluşturmak için kaydedilir.
            map = np.vstack((map, z_points))                        
            previous_time = 0
        else: 
            previous_time += 0.064
            print("previous_time: ", round(previous_time, 3))

        print("----------Harita----------")
        print("map: ", map)
        print("map size: ",len(map))
        print("--------------------------")
        

        # hedefin pozisyonunun belirlenmesi ve robot ile arasındaki mesafenin hesaplanması
        hedef_pos = hedef.getPosition()
        hedef_pos = np.array([
            round(hedef_pos[0], 3),
            round(hedef_pos[1], 3)
        ])
        # print("hedef: ", hedef_pos)
        
        robot_pose_webots = get_robot_pose_from_webots()
        
        xd = hedef_pos[0] - robot_pose_webots[0]
        yd = hedef_pos[1] - robot_pose_webots[1]
        distance_path = math.hypot(xd, yd)
        print("Hedef mesafesi: ", distance_path)

        if distance_path <= 0.15: # eğer robot hedef konuma yakınsa
            print("Hedef konuma ulaşıldı")
            isGoal = True
                
        vL, vR = calculate_wheel_speed(v, w)
        nPath, dist = robotControl(vL, vR, isGoal, distance_path, path, current_time)

        hTruePos = np.hstack((hTruePos, robot_pose_webots)) # robotun webots içerisinde hareket ederken gittiği bütün konumlar hTruePos a  kaydedildi
        print("Gerçek robot konumları kaydedildi \n")

        if show_animation:      
            plt.clf()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            # Haritanın çizilmesi
            plt.plot(map[:, 0],
                    map[:, 1], ".b", label="Harita")
            
            # robotun gerçek konumunun harita içerisindeki ilerlemesi
            plt.plot(hTruePos[0, :],
                    hTruePos[1, :], "-b")#label="Gerçek Konum"
            
            # robotun enkoder ile hesaplanan konumunun harita içerisindeki ilerlemesi
            plt.plot(hEncoPos[0, :],
                    hEncoPos[1, :], "-k")#label="Encoder Konum"
            
            # Anlık poitCloud verisi
            plt.plot(z[:, 0],
                    z[:, 1], ".m", label="Anlık Lidar")
            
            # robotun ileri yönünü gösteren ok çizildi
            plt.arrow(robot_pose_webots[0, 0], 
                    robot_pose_webots[1, 0], 0.05 * math.cos(robot_pose_webots[2]), 
                    0.05 * math.sin(robot_pose_webots[2]),
                    head_length=0.07, 
                    head_width=0.07
            )
           
            # Encoder ile hesaplanan açı bilgisini görmek için arrow kullanıldı
            plt.arrow(robot_pose_encoder[0, 0], 
                    robot_pose_encoder[1, 0], 0.05 * math.cos(robot_pose_encoder[2]), 
                    0.05 * math.sin(robot_pose_encoder[2]),
                    head_length=0.07, 
                    head_width=0.07,
                    color= "k"
            )
            
            plt.plot(hedef_pos[0], hedef_pos[1], "xg")
           
            plt.title("Webots_Lidar_Mapping")
            plt.legend()
            plt.axis("equal")
            plt.grid(True)            
            plt.pause(0.001)
        
        previous_time = current_time

