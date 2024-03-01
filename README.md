## Dokumentasi PKL BRIN

    ---

    Dokumentasi ini merupakan laporan yang memuat rangkuman dari berbagai kegiatan yang telah dilakukan dalam pengembangan JetRacer 

### Flash JetRacer Pro AI Kit Image

    Menggunakan prebuilt image bawaan dari JetRacer yang sudah dilengkapi Ubuntu 18.04 dan Script Jupyter Notebook untuk navigasi sederhana. 

    Tahapan instalasi mengikuti link berikut: [https://github.com/laitathei/JetRacer_Pro](https://github.com/laitathei/JetRacer_Pro).

    Tools: 

    * SD Card
    * Balena Etcher
    * JetRacer image

### Setup VNC

    Tahapan instalasi mengikuti link berikut: [https://www.youtube.com/watch?v=a3G6r8x_Bbc.](https://www.youtube.com/watch?v=a3G6r8x_Bbc.)

### Menjalankan Script Bawaan di Jupyter Notebook

    Berikut tahapannya: 

    * Buka laman `http://<jetracer_ip_address>:8888` pada browser.
    * Login ke Jupyter Lab menggunakan password `jetson`.
    * Pilih notebooks yang tersedia di laman tersebut, program dapat dimodifikasi sesuai keinginan.
    * Jalankan cells.

### Set Up ROS Melodic dan Catkin Ws
    
    Menggunakan Script untuk menginstall keseluruhan dependensi dari ROS sehingga hanya perlu menjalankan script tersebut. 
    
    Tahapan instalasi mengikuti link berikut: [https://github.com/jetsonhacks/installROS?tab=readme-ov-file](https://github.com/jetsonhacks/installROS?tab=readme-ov-file)
    
    Setelah instalasi selesai, diperlukan memeriksa file `**~/.bashrc**` untuk memastikan bahwa `**ROS_MASTER_URI**` dan `**ROS_IP**` telah diatur dengan benar dan sesuai.
    
### Install RPLidar-A1
    
    Menggunakan prebuilt image bawaan dari JetRacer yang sudah dilengkapi Ubuntu 18.04 dan Script Jupyter Notebook untuk navigasi sederhana. 
    
    * Clone repository dari Slamtec : [https://github.com/Slamtec/rplidar_ros.git](https://github.com/Slamtec/rplidar_ros.git)
    * Jalankan `catkin_make` 
    * Gunakan perintah `roslaunch rplidar_ros rplidar_a1.launch` untuk menjalankan node rplidar
    * Gunakan perintah `roslaunch rplidar_ros view_rplidar_a1.launch` untuk visualisasi di rviz

### Install Hector Slam
    
    Prerequisites: `Install Qt4`
    
    Tahapan instalasi mengikuti link berikut:
    
    * [https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/](https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/).
    * [https://github.com/NickL77/RPLidar_Hector_SLAM](https://github.com/NickL77/RPLidar_Hector_SLAM)
    
### Install Arduino Uno
    
    Tahapan Install:
    
    * Clone repositories: [https://github.com/JetsonHacksNano/installArduinoIDE.git](https://github.com/JetsonHacksNano/installArduinoIDE.git)
    * Jalankan `catkin_make`
    * Kemudian jalankan script `$ ./installArduinoIDE.sh`
    
### Setup Rosserial
    
    Referensi yang digunakan ialah dari [https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup). 
    
    Tahapan instalasi sebagai berikut :
    
    * jalankan script `sudo apt-get install ros-melodic-rosserial-arduino sudo apt-get install ros-melodic-rosserial` 
    * Buka catkin\_ws dan jalankan `git clone` [`https://github.com/ros-drivers/rosserial.git`](https://github.com/ros-drivers/rosserial.git)
    * Jalankan `catkin_make` dan `catkin_make install`
    * Kemudian install library **rosserial** pada Arduino IDE
    
    
### Program Teleoprasi Keyboard, Widget, dan Joystick.
    
    Menambahkan script teleoprasi IPYNB pada folder JetRacer. Kemudian dijalankan melalui laman jupyter notebook.
    
### Membuat Package ROS 
    
    Tahapan membuat package ROS sebagai berikut:
    
    * `cd ~/catkin_ws/src`
    * `catkin_create_pkg my_racecar_control rospy roscpp std_msgs`
    * `cd ~/catkin_ws/`lalu jalankan `catkin_make`
    * Masukan library bawaan jetracer ke folder my\_racecar\_control/src
    * Sesuaikan `CMakelists.txt` dan `package.xml`
    * Install library traitlets, adafruit servokit, dan upgrade versi ke `python 3`
    
    Note: Masih terdapat error setiap melakukan catkin\_make, disebabkan package jetracer yang tidak ditemukan, kemungkinan ada kesalahan di konfigurasi CMakelists atau package.xml
    
### Mengolah Data RPLIDAR A1 (Segmentasi 4 Area) 
    
    Data lidar yang didapatkan kemudian di oleh menjadi 4 area, yaitu _front, back, left and right_
    
    Berikut programnya : 
    
    ```python
    #!/usr/bin/env python3
    
    import rospy
    from sensor_msgs.msg import LaserScan
    
    class LidarData:
    def __init__(self):
    rospy.init_node('lidar_node', anonymous=True)
    self.front, self.right, self.left, self.back = None, None, None, None
    
    rospy.Subscriber("/scan", LaserScan, self.scan_callback)
    rospy.spin()
    
    def calculate_mean_distance(self, angles_degrees, distances, angle_range):
    area_distances = [dist for angle, dist in zip(angles_degrees, distances) if angle_range[0] <= angle <=angle_range[1]] return sum(area_distances) / len(area_distances) if area_distances else None def scan_callback(self, scan_msg): angles=[scan_msg.angle_min + scan_msg.angle_increment * i for i in range(len(scan_msg.ranges))] angles_degrees=[angle * (180.0 / 3.14159) for angle in angles] distances=scan_msg.ranges # Define angle ranges for each area f, r, l, b=(175, 185), (67.5, 112.5), (-112.5, -67.5), (-5, 5) # Calculate mean distance for each area self.front, self.right, self.left, self.back=(self.calculate_mean_distance(angles_degrees, distances, a) for a in [f, r, l, b]) # Print the mean distances self.print_mean_distances() def print_mean_distances(self): rospy.loginfo(f"Mean Distance in Area Front: {self.front}") rospy.loginfo(f"Mean Distance in Area Right: {self.right}") rospy.loginfo(f"Mean Distance in Area Left: {self.left}") rospy.loginfo(f"Mean Distance in Area Back: {self.back}") if __name__=='__main__' : LidarData() ``` ### Merancang Program Wall Following dengan PID Program Wall following dijalankan menggunakan script bernama wall\_following.py. Program ini menginisiasi node ROS bernama racecar\_control\_node yang berfungsi untuk mengendalikan JetRacer.  Berikut Programnya: ```python #!/usr/bin/env python3 import rospy import math from jetracer.nvidia_racecar import NvidiaRacecar from sensor_msgs.msg import LaserScan class RacecarNode: def __init__(self): rospy.init_node('racecar_control_node', anonymous=True) rospy.Subscriber("/scan", LaserScan, self.scan_callback) self.front, self.right, self.left, self.back=None, None, None, None self.car=NvidiaRacecar() # Register the callback to stop the throttle when shutting down rospy.on_shutdown(self.shutdown_callback) rospy.spin() def calculate_mean_distance(self, angles_degrees, distances, angle_range, min_threshold): area_distances=[dist for angle, dist in zip(angles_degrees, distances) if angle_range[0] <=angle <=angle_range[1] and not math.isnan(dist) and dist < float('inf') and dist> min_threshold]
        if area_distances:
        return sum(area_distances) / len(area_distances)
        else:
        return None
        
        def scan_callback(self, scan_msg):
        angles = [scan_msg.angle_min + scan_msg.angle_increment * i for i in range(len(scan_msg.ranges))]
        angles_degrees = [angle * (180.0 / 3.14159) for angle in angles]
        distances = scan_msg.ranges
        # Define angle ranges for each area
        f, r, l, b = (175, 185), (133, 136), (-136, -133), (-5, 5)
        self.a_angle = 160
        self.b_angle = 90
        
        self.a_range = self.calculate_mean_distance(angles_degrees, distances, (self.a_angle - 1, self.a_angle + 1), 0.1)
        self.b_range = self.calculate_mean_distance(angles_degrees, distances, (self.b_angle - 1, self.b_angle + 1), 0.1)
        # Assigning values to self.a_range and self.b_range
        # Ensure self.a_range is not None
        self.a_range = self.a_range if self.a_range is not None else 100.0 # Assign default value 100.0 if self.a_range is None
        # Ensure self.b_range is not None
        self.b_range = self.b_range if self.b_range is not None else 100.0 # Assign default value 100.0 if self.b_range is None
        
        # Set minimum threshold distance
        min_threshold = 0.1 # Adjust this value based on your laser scanner's specifications
        # Calculate mean distance for each area
        self.front, self.right, self.left, self.back = (self.calculate_mean_distance(angles_degrees, distances, a, min_threshold) for a in [f, r, l, b])
        self.motorDrive()
        
        def motorDrive(self):
        if hasattr(self, 'car') and self.front is not None and self.left is not None and self.right is not None:
        # Adjust throttle based on the distance in front
        # if abs(self.car.steering) > 0.3:
        # self.car.throttle = 0.15
        # elif abs(self.car.steering) > 0.2:
        # self.car.throttle = 0.15
        # else:
        # self.car.throttle = 0.15
        # if self.front > 0.8:
        # self.car.throttle = 0.17
        # else:
        # self.car.throttle = 0.16
        if self.front < 0.2: # If there is a wall in front, prioritize avoiding collision # self.car.steering=1.0 self.car.throttle=0.01 else: # If there is no wall in front, continue following the left wall if self.left < 0.2: # If left distance is less than 0.4 and right is greater than left, steer right self.car.steering=0.4 else: # Otherwise, continue following the left wall distRef=0.3 rospy.loginfo("Left Distance: %.4f", self.left) alpha=math.atan((self.a_range * math.cos(self.b_angle - self.a_angle) - self.b_range) / (self.a_range * math.sin(self.b_angle - self.a_angle))) d_t=self.b_range * math.cos(alpha) d_t1=d_t + 0.3 * math.sin(alpha) error=distRef - d_t1 # error=distRef - self.left rospy.loginfo("Err: %.4f", error) rospy.loginfo("Dt %.4f", d_t) rospy.loginfo("Dt1: %.4f", d_t1) KP=5.0 KI=0.0 KD=1.0 if 'integral' not in self.__dict__: self.integral=0 self.integral +=error if 'previous_error' not in self.__dict__: self.previous_error=0 derivative=error - self.previous_error self.previous_error=error PIDvalue=KP * error + KI * self.integral + KD * derivative PIDvalue=min(max(PIDvalue, -1.0), 1.0) rospy.loginfo("PID value: %.4f", PIDvalue) # self.car.steering=-PIDvalue # else: # self.car.throttle=0.0 def shutdown_callback(self): # Stop the throttle when the ROS node is shut down if hasattr(self, 'car' ): self.car.throttle=0.0 if __name__=='__main__' : racecar_node=RacecarNode() ``` Penjelasan Struktur Program: #### Mendefinisikan Class ‘RacecarNode’  * Init node: `racecar_control_node` * Subscribe: `/scan` * Mendefinisikan object car * scan\_callback: Memproses data pemindaian lasescan. Menghitung jarak rata-rata pada range sudut tertentu di area sekitar Jetracer. * Memanggil fungsi motorDrive untuk mengontrol Jetracer berdasarkan pembacaan sensor. * shutdown\_callback: Menghentikan throttle Jetracer ketika node ROS dimatikan. #### Pengolahan Data Sensor * Data pemindaian laser yang diterima dari topik `/scan` diproses dalam fungsi `scan_callback`. * Jarak rata-rata dihitung untuk empat area berbeda di sekitar JetRacer: depan, kanan, kiri, dan belakang. * Rentang sudut ditentukan untuk setiap area untuk mengekstrak data yang relevan dari pemindaian laser. #### Logika Pengendalian * Fungsi `motorDrive` mengimplementasikan logika pengendalian untuk JetRacer. * Ini menyesuaikan throttle dan kemudi berdasarkan pembacaan sensor. * Jika dinding terdeteksi di depan (`self.front < 0.2`), JetRacer memprioritaskan menghindari tabrakan dengan mengurangi throttle (`self.car.throttle=0.01`). * Jika tidak, JetRacer mengikuti dinding kiri: * Jika jarak ke kiri kurang dari ambang batas tertentu (`self.left < 0.2`), Jetracer bermanuver ke kanan untuk menghindari tabrakan. * Jika tidak, dihitung kesalahan antara jarak yang diinginkan dari dinding kiri dan jarak aktual (`error=distRef - d_t1`) dan diterapkan kontrol PID untuk menyesuaikan sudut kemudi sesuai. #### Kontrol PID * Kontrol Proportional-Integral-Derivative (PID) digunakan untuk menyesuaikan sudut kemudi berdasarkan kesalahan antara jarak yang diinginkan dan aktual dari dinding kiri. * Parameter PID (KP, KI, KD) dapat dikonfigurasi untuk menyetel perilaku pengendalian. * Istilah integral dan derivatif dihitung untuk meningkatkan kinerja dan stabilitas pengontrol. ### Things to Improve: * Menambahkan TTC Braking system. * Menggunakan Reactive Methods untuk wall following, kemudian kembangkan ke algortima safety bubble. * Mapping. * Planning menggunakan algoritma Pure Pursuit. * VIsion. ---
