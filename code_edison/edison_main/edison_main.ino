  #include <ransac.h>
  #include <gyro_pid.h>
  #include <motor_define.h>
  
  #include <sys/mman.h>
  #include <stdio.h>
  #include <stdint.h>
  #include <fcntl.h>
  
  #include <Arduino.h>
  #include <Wire.h>
  #include <I2Cdev.h>
  #include <MPU6050Wrapper.h>
  #include <Lidar.h>
  #include <motor_controller_comms.h>
  #include <dist_pid.h>
  
  
  
  /*Motor Controlling*/
  //-----------------------------------------------------
  float vel_l = 5.0;
  float vel_r = 5.0;
  
  float vel_l_nom = 8.0;
  float vel_r_nom = 8.0;
  
  long last_time_motors = 0;
  long curTime = 0;
  
  unsigned char mc_buf[PACKET_SIZE];
  //-----------------------------------------------------
  
  
  /*Reading imu*/
  //-----------------------------------------------------
  boolean saveData = false;
  long lastTime_imu = 0;
  long curTime_imu = 0;
  
  int save_count = 0;
  
  boolean file_open = false;
  int imuTimer = 10000000;
  int delayTime = 10000;
  int code = 1;
  int save_time_start;
  //-----------------------------------------------------
  
  
  /* Angle correction */
  //-----------------------------------------------------
  long lastTime_gyro_pid;
  
  boolean useGyro = false;
  
  boolean started = false;
  long lastTime_angle = 0;
  //Backwards engineering stuff
  #define SLIP_CONST 180.0/155.0
  #define COUNTS_PER_DEGREE_PER_WHEEL 15.04
  #define COUNTS_PER_RAD 509.296 // (3200 counts/rev) / (2pi rads / rev)
  
  int startTDelay = 3;
  int startT = 0;
  double s = 3;
  double d = 180;
  double slipConst = 1;
  double yawi = 0;
  boolean yawi_set = false;
  double rad_per_sec_diff = (SLIP_CONST * COUNTS_PER_DEGREE_PER_WHEEL * d)/(COUNTS_PER_RAD*s);
  
  /* Robot state */
  #define NUM_DISTS 5
  double dist_buf_left[NUM_DISTS], dist_buf_right[NUM_DISTS], dist_buf_front[NUM_DISTS];
  double dist_average_left, dist_average_right, dist_average_front;
  double dist_sum_left = 0, dist_sum_right = 0, dist_sum_front = 0;
  int num_dists_left = 0, num_dists_right = 0, num_dists_front = 0;
  int dist_buf_ind_left = 0, dist_buf_ind_right = 0, dist_buf_ind_front = 0;
  double dist_cur_left, dist_cur_right, dist_cur_front;
  
  int robot_state; // 0 = brake, 1 = drive, 2 = position correct
  int robot_warning; // 0 = fine, 1 = close left, 2 = close right;
  int robot_open_area, robot_open_area_count = 0;
  long time_since_turn = 0;
  
  double angle_left, angle_right;
  
  int priority_warning = 0;
  int secondary_warning = 0;
  //-----------------------------------------------------
  
  int scans_received = 0;
  
   long start_time_scan;
  long cur_time_scan;
  
  void update_robot_state() {
    
    /* Left state */
    equation_parameters test = calculate_linear_regression(52,150,500);
    dist_cur_left = test.b;
    //Dist buffer
   /* if(num_dists_left < NUM_DISTS) {
      num_dists_left++;
      dist_cur_left = test.b;
      dist_buf_left[num_dists_left-1] = test.b;
      dist_sum_left += test.b;
      dist_average_left = dist_sum_left / num_dists_left;
    }
    else {
      dist_cur_left = test.b;
      dist_sum_left -= dist_buf_left[dist_buf_ind_left];
      dist_buf_left[dist_buf_ind_left] = test.b;
      dist_sum_left += test.b;
      dist_average_left = dist_sum_left / num_dists_left;
      dist_buf_ind_left = (dist_buf_ind_left+1) % NUM_DISTS;
    }*/
    angle_left = test.m;
    
    /* Left state */
    test = calculate_linear_regression(230,315,500);
     dist_cur_right = test.b;
    /*if(num_dists_right < NUM_DISTS) {
      num_dists_right++;
      dist_cur_right = test.b;
      dist_buf_right[num_dists_right-1] = test.b;
      dist_sum_right += test.b;
      dist_average_right = dist_sum_right / num_dists_right;
    }
    
    else {
      dist_cur_right = test.b;
      dist_sum_right -= dist_buf_right[dist_buf_ind_right];
      dist_buf_right[dist_buf_ind_right] = test.b;
      dist_sum_right += test.b;
      dist_average_right = dist_sum_right / num_dists_right;
      dist_buf_ind_right = (dist_buf_ind_right+1) % NUM_DISTS;
    }*/
    angle_right = test.m;
    
    /* Front state */
    test = calculate_linear_regression(10,40,500);
    dist_cur_front = test.b;
    if(num_dists_front < NUM_DISTS) {
      num_dists_right++;
      dist_cur_front = test.b;
      dist_buf_front[num_dists_front-1] = test.b;
      dist_sum_front += test.b;
      dist_average_front = dist_sum_front / num_dists_front;
    }
    
    else {
      dist_cur_front = test.b;
      dist_sum_front -= dist_buf_front[dist_buf_ind_front];
      dist_buf_front[dist_buf_ind_front] = test.b;
      dist_sum_front += test.b;
      dist_average_front = dist_sum_front / num_dists_front;
      dist_buf_ind_front = (dist_buf_ind_front+1) % NUM_DISTS;
    }
    
   float dist_abs_left = abs(dist_cur_left);
   float dist_abs_right = abs(dist_cur_right);
   robot_warning = 0;
   if(abs(dist_average_front) < 400){
      robot_warning = 3;
   }
   else if(dist_abs_left < dist_abs_right){
      if(dist_abs_left < 2500 && dist_abs_left > 10) {
          robot_warning = 1;
      }else if(dist_abs_right < 2500 && dist_abs_right > 10) {
            robot_warning = 2;
      }
    }else if (dist_abs_left > dist_abs_right){
       if(dist_abs_right < 2500 && dist_abs_right > 10) {
          robot_warning = 2;
      }else if(dist_abs_left < 2500    && dist_abs_left > 10) {
            robot_warning = 1;
      }
    }
    
   robot_open_area = 0;
   if(dist_abs_left > 1650 || dist_abs_right > 1650) {
     robot_open_area = 1; 
     robot_open_area_count++;}
   else {
     robot_open_area_count=0; }
    
    /*
    }else if(abs(dist_average_left) < 750) {
      robot_warning = 1;
    }
    else if(abs(dist_average_right) < 750) {
      robot_warning = 2;
    }
    //else if(abs(dist_average_front) < 500){
      //vel_l += rad_per_sec_diff;
       //vel_r += -rad_per_sec_diff;
    //}
    else {
      robot_warning = 0;
    }
*/
  }
  
  void robot_take_action() {
    if(robot_warning == 3){
      stop_pls();
    }else if(robot_warning == 1 && angle_left < 0) {
       Serial.println("E LEFT");
       dist_pid_set_target_angle(.2); 
    }
     else if(robot_warning == 2 && angle_right > 0) {
      Serial.println("E RIGHT");
       dist_pid_set_target_angle(-.2); }
     else {
       dist_pid_set_target_angle(.05); }
     
     dist_pid_set_gains(2, 0, 0);
     if(robot_open_area && robot_open_area_count < 3) {
       dist_pid_set_gains(.25, 0, 0);
     }
     
     if(robot_open_area_count < 2) {
       Serial.print("OK");
        dist_pid_update(angle_left, &vel_l, &vel_r, vel_l_nom, vel_r_nom);
        update_motors();
     }
     else {
        int current_time_to_turn = millis(); 
        if(current_time_to_turn - time_since_turn < 10000) {}
        else{
         Serial.print("TURN");
         if(dist_cur_left < dist_cur_right ) {
           vel_l = -7;
           vel_r = 7;
         }
         else {
           vel_l = 7;
           vel_r = -7;
         }
         int start_time = millis();
         int cur_time;
         while((cur_time=millis()) - start_time < 150) {
           update_motors();
           //delay(50);
         }
        // init_lidar();
         robot_open_area_count = 0;
         time_since_turn = millis();
       }
     }
    
     init_lidar();
     /*Serial.print(robot_warning);
     Serial.print('\t');
     Serial.print(angle_left);
     Serial.print('\t');
     Serial.print(dist_cur_left);
     Serial.print('\t');
     Serial.print(angle_right);
     Serial.print('\t');
     Serial.print(dist_cur_right);
     Serial.print('\t');
     Serial.print(vel_l);
     Serial.print('\t');
     Serial.print(vel_r);
     Serial.print('\t');
     Serial.print(robot_open_area_count);
     Serial.println();
     */
      reset_scan_buf();
     
  }
  
  void setup() {
   Serial.begin(115200);
   Serial1.begin(115200); 
  
   init_imu();
   delay(1000);
   loop_imu();	
   //imu_open_save_file(save_count);
   //file_open = true;
   
   pinMode(2, OUTPUT);
   pinMode(4, OUTPUT);
   pinMode(3, INPUT);
   pinMode(7, INPUT);
   
   
   delay(2000);
   save_time_start = millis();
   digitalWrite(4,HIGH);
   
   
   //initialize gyro angle correction
   gyro_pid_set_gain_enable(1, 0, 0);
   gyro_pid_set_timestep(10);
   gyro_pid_set_gains(.2, 0, 0);
   gyro_pid_set_target_angle(30);
   
   dist_pid_set_gain_enable(1, 0, 0);
   dist_pid_set_timestep(450);
   dist_pid_set_gains(2, 0, 0);
   dist_pid_set_target_angle(.15); 
   init_lidar();
  }
  
  void loop() {
  
    curTime = millis();
    int in_pin = digitalRead(3) || digitalRead(7);
    //init_lidar();
    //loop_imu
    loop_lidar();
   
     if(in_pin && curTime - last_time_motors >= 10) {
       update_motors();
       last_time_motors = curTime;
     }
  /*
  if(curTime - lastTime_imu > 1){
  int in_pin = digitalRead(3);
   
  if(!in_pin && file_open){
  imu_close_save_file();
  stop_lidar_save_data();
  save_count++;
  file_open = false;
  digitalWrite(2,LOW);
  } else if(!file_open && in_pin){
  imu_open_save_file(save_count);
  
  init_lidar();
  init_lidar_data_save(save_count);
  file_open = true;
  long start_time = millis();
  
  char c;
  while(millis() - start_time < 5000) {
  while(Serial1.available()) {c = Serial1.read();}
  }
  
  }else if(in_pin && file_open){
  imu_save_data();
  digitalWrite(2,HIGH);
  }
  else {
  digitalWrite(2, LOW);
  }
  
  
   //Serial.println(file_open);
  if(curTime_imu - save_time_start > delayTime && file_open){
  file_open = false;
  imu_close_save_file();
  digitalWrite(2,LOW);
  }else if(file_open){
  digitalWrite(2,HIGH);
  imu_save_data();
  }
  
  
  }
  */
  if(in_pin){
    digitalWrite(2,HIGH);
    
  }else{
    stop_pls();
    digitalWrite(2,LOW);
  }
  
  if(flag_scan_complete) { 
    
    scans_received++;
    if(in_pin){
      update_robot_state();
      robot_take_action();
    }
    
    while(Serial1.available() > 100) Serial1.read();
    reset_scan_buf();
  
  }
  
  //gyro_pid_update(currentAngle, &vel_l, &vel_r);
  
  /*  if(!saveData && !useGyro){
  if(lidar_buf_count > 90) {
  int i;
  for(i = 0; i < 90; i++) {
    lidar_packet p = lidar_packet_buf[lidar_buf_out_ind];
    if((p.id-160) * 4 == 240) {
     // Serial.println(p.id);
      if(p.dist[2] < 900 && p.dist[2] != 53) {
       //Serial.println('b');
         digitalWrite(2, HIGH);
         //digitalWrite(13, HIGH);
      }
    else {
      digitalWrite(2, LOW);
      //digitalWrite(13, LOW);
    }
  
   // Serial.println();
  }
  lidar_buf_out_ind = (lidar_buf_out_ind+1)%LIDAR_BUF_SIZE;
  lidar_buf_count--;
  }
  }
   }*/
  
  
  /*
  if(useGyro && (started|| (!saveData && in_pin))){
  if(!started){
  startT = startTDelay * 1000 + millis();
  started = true;
  digitalWrite(2,HIGH);
  }
  
  double angleTraversed = 0;
  }
  */
  //Serial.print(currentAngle);
  //Serial.println(vel_l-rad_per_sec_diff);
  /*
  if(curTime > 30000) {
  vel_l = 0;
  vel_r = 0;
  }
  else if(curTime > (startT+(2*s*1000)))  {
  vel_l = 0;
  vel_r = 0;
  }
  else if(curTime > (startT+(s*1000))) {
  vel_l = 6;
  vel_r = 6;
  started = false;
  yawi_set = false;
  digitalWrite(2,LOW);
  }
  else if(curTime > startT) {
  if(!yawi_set){
  yawi = currentAngle;
  yawi_set = true;
  }
  double time = (curTime - startT);
  
  angleTraversed = rad_per_sec_diff*(time/1000.0) * COUNTS_PER_RAD/COUNTS_PER_DEGREE_PER_WHEEL;
  double ca = currentAngle - yawi;
  if(ca<0) ca += 360;
  //slipConst = abs((angleTraversed - d)/(abs(ca) - d));
  
  Serial.println(ca);    
  Serial.print("\t");
  Serial.print(angleTraversed);
  Serial.print("\t");
  Serial.print(slipConst);
  vel_l = slipConst * rad_per_sec_diff;
  vel_r = slipConst *  -rad_per_sec_diff;
  }
  else {
  vel_l = 6;
  vel_r = 6;
  }
  gyro_pid_set_target_angle(angleTraversed + yawi);
  gyro_pid_update(currentAngle, &vel_l, &vel_r);
  }else{
  vel_l = 0;
  vel_r = 0;
  }
  */
  }
  

  void update_motors(){
    make_packet_vels(mc_buf,vel_r,vel_l);
    send_packet_serial(mc_buf);
  }
  
  void stop_pls(){
    vel_l = 0;
    vel_r = 0;
    
  }
  

