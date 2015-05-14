#include <sys/mman.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>

#include <Arduino.h>
#include <Lidar.h>

void init_edison() {
	
	
	init_lidar();
	
	init_lidar_data_save();
}

void loop_edison() {
	loop_lidar();
}

