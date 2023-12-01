int32 sender_id
int32 receiver_id
int32 msg_size

# текущий режим
uint8 mode_real      

# флаги замыкания контуров 
uint8 yaw_contour           
uint8 pitch_contour 
uint8 roll_contour 
uint8 march_contour 
uint8 depth_contour 
uint8 lag_contour 

# текущий выбор модель/реальный НПА
uint8 mode_auv_real  

# текущие курс, дифферент, крен, значения с имушки по сути
float32 yaw   
float32 pitch
float32 roll  
float32 march 
float32 depth 
float32 lag 

# данные, которые идут на каждый ВМА
float32 vma1    
float32 vma2    
float32 vma3    
float32 vma4    
float32 vma5    
float32 vma6  

# данные с БСО
float32 yaw_imu       #курс градусы +/- 180
float32 pitch_imu     
float32 roll_imu    
float32 x_accel_imu  
float32 y_accel_imu 
float32 z_accel_imu  
float32 x_rate_imu   
float32 y_rate_imu   
float32 z_rate_imu   
float32 x_magn_imu   
float32 y_magn_imu   
float32 z_magn_imu   
float32[4] quat

# данных с датчика давления
float32 temperature 
float32 depth_sensor       
float32 pressure  

# данные с UWB
float32 location_x # координата аппарата по оси X
float32 location_y # координата аппарата по оси Y
float32[4] distance_to_beacon # расстоние до i-го маяка
float32[10] distance_to_agent # расстояние до i-го агента

uint8 start_calibration
uint8 end_calibration

# состояние миссии
uint8 id_mission 
uint8 mission_status

uint32 checksum