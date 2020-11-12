#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor_Calibration.h>

#define NUMBER_OF_CALIBRATION_SECONDS (60)
#define UPDATE_RATE_HZ 400

Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);
Adafruit_Sensor *gyroscope;

float sum_x = 0;
float sum_y = 0;
float sum_z = 0;

int count = 0;

uint32_t timestamp;
void setup() {
  Serial.begin(115200);
  while (!Serial) yield();  
  
  fxas.begin(GYRO_RANGE_500DPS);
  gyroscope = &fxas;
  
  gyroscope->printSensorDetails();
    
  timestamp = micros();  

  Wire.setClock(400000); // 400KHz  

  Serial.println(F("Place gyro on flat, stable surface!"));

  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");

  for (uint32_t sample = 0; sample < NUMBER_OF_CALIBRATION_SECONDS*UPDATE_RATE_HZ; sample++) {

    uint32_t curr_time = micros();    
    while ((curr_time - timestamp) < (1000000 / UPDATE_RATE_HZ)) {        
        curr_time = micros();    
        yield();
    }
    
    timestamp = curr_time;

    sensors_event_t event;
    gyroscope->getEvent(&event);
    sum_x += event.gyro.x;
    sum_y += event.gyro.y;
    sum_z += event.gyro.z;

    count += 1;
    
    if(sample % UPDATE_RATE_HZ == 0){
        Serial.print("Progress: ");
        Serial.print(((100.0*sample/UPDATE_RATE_HZ)/NUMBER_OF_CALIBRATION_SECONDS));
        Serial.println("%...");
    }
    
  }
  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(sum_x/count, 10); Serial.print(", ");
  Serial.print(sum_y/count, 10); Serial.print(", ");
  Serial.println(sum_z/count, 10);
  
}


void loop() {
  delay(10); 
}