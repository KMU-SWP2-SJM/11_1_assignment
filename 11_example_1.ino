#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED 핀
#define PIN_TRIG  12  // TRIGGER 핀
#define PIN_ECHO  13  // ECHO 핀
#define PIN_SERVO 10  // SERVO 핀

// configurable parameters for sonar
#define SND_VEL 346.0     // 24도 기준 음속 (unit: m/sec)
#define INTERVAL 25      // 샘플링 주기 (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // 최소 거리 (unit: mm)
#define _DIST_MAX 360.0   // 최대 거리 (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // 최대 에코 주기 (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // 길이 변환용

#define _EMA_ALPHA 0.5    // EMA 가중치 (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

// 목표 거리
#define _TARGET_LOW  190.0
#define _TARGET_HIGH 350.0

// duty duration for myservo.writeMicroseconds()
// NEEDS TUNING (servo by servo)
 
#define _DUTY_MIN 500 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1500 // servo neutral position (90 degree)
#define _DUTY_MAX 2500 // servo full counterclockwise position (180 degree)

// global variables
float  dist_ema, dist_prev = _DIST_MAX; // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // 핀 초기화
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);  // LOW로 지정

  // servo 초기화
  myservo.attach(PIN_SERVO); 
  // 중간각도
  myservo.writeMicroseconds(_DUTY_NEU);

  // 필터링 이전값 최소로 초기화
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float  dist_raw;
  
  // 샘플링 주기 함수 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;           // Cut higher than maximum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;           // cut lower than minimum
    digitalWrite(PIN_LED, 1);       // LED OFF
  } else {    // In desired Range
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, 0);       // LED ON      
  }

  // EMA 식  = a * d(k) + [ 1 - a ] * d(k-1) = 알파 * 거리 + ( 1 - 알파 ) * 직전거리
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema;

  // adjust servo position according to the USS read value
  // add your code here!
  if (dist_ema < _TARGET_LOW){
    myservo.writeMicroseconds(_DUTY_MIN);
  } else if ( dist_ema > _TARGET_HIGH){
    myservo.writeMicroseconds(_DUTY_MAX);
  } else {
    // ema 값을 duty 사이클 값으로 변환
    float servo_pos = map(dist_ema, _TARGET_LOW, _TARGET_HIGH, _DUTY_MIN, _DUTY_MAX); // TARGET값에 따라 DUTY 값으로 변경됨
    servo_pos = constrain(servo_pos, _DUTY_MIN, _DUTY_MAX); // pos 값이 duty 범위 내에 있도록 지정
    myservo.writeMicroseconds(servo_pos); // duty 범위로 각도 조절
  }

  // output the distance to the serial port
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(",dist:");  Serial.print(dist_raw);
  Serial.print(",ema:");  Serial.print(dist_ema);
  Serial.print(",Servo:"); Serial.print(myservo.read());  
  Serial.print(",Max:");   Serial.print(_DIST_MAX);
  Serial.println("");
 
  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
