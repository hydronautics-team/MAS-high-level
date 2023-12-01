# данные, которые идут с пульта, с джойстика
float32 yaw_joy   
float32 pitch_joy 
float32 roll_joy  
float32 march_joy
float32 depth_joy 
float32 lag_joy   

# режим работы
uint8 cs_mode #ручной, автоматизир-й, автоматич-й, групповой

float32[3] beacon_x
float32[3] beacon_y

# флаги замыкания контуров (если больше 0, то замкнуты)
uint8 yaw_closed_real 
uint8 pitch_closed_real 
uint8 roll_closed_real 
uint8 march_closed_real 
uint8 depth_closed_real 
uint8 lag_closed_real 

#текущий выбор модель/реальный НПА
uint8 mode_auv_selection

#режим работы системы питания
uint8 power_mode 

#флаги для калибровки в корпусе
uint8 init_calibration
uint8 save_calibration

# команды управления миссией
uint8 id_mission_auv
uint8 mission_command

uint8 checksum_to_bort