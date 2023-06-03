#include <Stepper.h>
#include <math.h>
#include <ArduinoJson.h>
#include <Servo.h>

// Step motor parametreleri
#define MOTOR_STEPS_PER_REV 200 // Tur başına adım sayısı
#define MOTOR_ANGLE_PER_STEP 1.8 // Her adım için açı (derece)
#define MOTOR_SPEED 300

// Motor 1 pin tanımları PORT_B
#define MOTOR_1_DIR_PIN 8
#define MOTOR_1_STEP_PIN 9
#define MOTOR_1_MS1_PIN 10
#define MOTOR_1_MS2_PIN 11

// Motor 2 pin tanımları PORT_C
#define MOTOR_2_DIR_PIN 14
#define MOTOR_2_STEP_PIN 15
#define MOTOR_2_MS1_PIN 16
#define MOTOR_2_MS2_PIN 17

// Motor 3 pin tanımları PORT_D
#define MOTOR_3_DIR_PIN 2
#define MOTOR_3_STEP_PIN 3
#define MOTOR_3_MS1_PIN 4
#define MOTOR_3_MS2_PIN 5
#define SERVO_PIN 6

// Robot kol parametreleri
#define L1 105.0 // İlk kol uzunluğu
#define L2 98.85 // İkinci kol uzunluğu -gripper merkezine olan uzaklık.
#define mm_per_step // Yukseklik motorunun bir adım başına katettiği yol. 
#define MOTOR_MAX_Z // düşey eksende maksimum derinlik.
#define tekil_gorev_suresi 2000
#define motor3_guvenlik_suresi 2000
#define gripper_delay 2000
#define release_safety_mm 1.5
#define ev_boyut 13
//Kutu yerleri
#define RED_BOX_1_X
#define RED_BOX_1_Y

#define RED_BOX_2_X
#define RED_BOX_2_Y

#define RED_BOX_3_X
#define RED_BOX_3_Y


#define GREEN_BOX_1_X
#define GREEN_BOX_1_Y

#define GREEN_BOX_2_X
#define GREEN_BOX_2_Y

#define GREEN_BOX_3_X
#define GREEN_BOX_3_Y


#define BLUE_BOX_1_X
#define BLUE_BOX_1_Y

#define BLUE_BOX_2_X
#define BLUE_BOX_2_Y

#define BLUE_BOX_3_X
#define BLUE_BOX_3_Y


#define BLACK_BOX_1_X
#define BLACK_BOX_1_Y

#define BLACK_BOX_2_X
#define BLACK_BOX_2_Y

#define BLACK_BOX_3_X
#define BLACK_BOX_3_Y

class Ev {
public:
  double x;
  double y;
  const char* colorName;
  int floor;

  // Yapıcı (constructor) fonksiyon
  Ev(double _x, double _y, const char* _colorName, int _floor) {
    x = _x;
    y = _y;
    colorName = _colorName;
    floor = _floor;
  }
};

double piksel_mm_donusum_katsayisi = (L1 + L2) / 1100 ;
double a, b, x, y, z;
int colorVal, floor;
int numHouses;
double kutu_alma_yuksekligi = MOTOR_MAX_Z - (ev_boyut/2);
bool isBoxUsed[3][2] = {false};  // Kutu kullanım durumu flag'leri
Ev evler[7];

// motor nesneleri
Stepper motor1(MOTOR_STEPS_PER_REV, MOTOR_1_DIR_PIN, MOTOR_1_STEP_PIN, MOTOR_1_MS1_PIN, MOTOR_1_MS2_PIN);
Stepper motor2(MOTOR_STEPS_PER_REV, MOTOR_2_DIR_PIN, MOTOR_2_STEP_PIN, MOTOR_2_MS1_PIN, MOTOR_2_MS2_PIN);
Stepper motor3(MOTOR_STEPS_PER_REV, MOTOR_3_DIR_PIN, MOTOR_3_STEP_PIN, MOTOR_3_MS1_PIN, MOTOR_3_MS2_PIN);
Servo servoMotor;


void rotateMotor(Stepper& motor, int steps) {
    motor.step(steps);
}

double inverse_kinematics(double x, double y) {
    // İlk açıyı hesapla (theta1)
  double theta1;
  double theta2;
  
  double k1 = L1 + L2 * cos(theta2);
  double k2 = L2 * sin(theta2);
  theta1 = atan2(y, x) - atan2(k2, k1);

  // İkinci açıyı hesapla (theta2)
  double num = pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2);
  double den = 2 * L1 * L2;
  theta2 = acos(num / den);
  return theta1, theta2;
}

int dusey_adim(double z) {
  int dusey_adim = -1 * (z / mm_per_step);
  return dusey_adim;
}

double acidan_adim(double aci) {
  int adim = aci * (180.0 / PI) / MOTOR_ANGLE_PER_STEP;
  return adim;
}

void gripper_kapa() {
  servoMotor.write(180);
}
void gripper_ac() {  
    servoMotor.write(0); // 0 dereceye ayarlayın (geri dönüş)
}

int parseJSON(const char* json) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, json);
  
  if (error) {
    // Ayrıştırma hatası var, hata işleme yapılabilir
    return;
  }
  
  int numHouses = doc.size(); // Ev sayısını al

  if (numHouses > 7) {
    // Hata: Beklenenden fazla ev var
    return;
  }

  for (int i = 0; i < numHouses; i++) {
    JsonObject house = doc[i]; // Sıradaki evi al

    double x = house["x"]* piksel_mm_donusum_katsayisi;
    double y = house["y"]* piksel_mm_donusum_katsayisi;
    const char* colorName = house["colorName"];
    int floor = house["floor"];

    evler[i] = Ev(x, y, colorName, floor);
  }
  return numHouses;
}

void Task(double a, double b, double x, double y, double z) {
  double aci1, aci2 = inverse_kinematics(a,b);
  int adim1 = acidan_adim(aci1);
  int adim2 = acidan_adim(aci2);
  int adim3 = dusey_adim(kutu_alma_yuksekligi);
  rotateMotor(motor1, adim1);
  rotateMotor(motor2, adim2);
  delay(motor3_guvenlik_suresi);
  rotateMotor(motor3, adim3);
  delay(tekil_gorev_suresi);
  gripper_kapa();
  delay(gripper_delay);
  //kutu alındı
  rotateMotor(motor1, -1*adim1);
  rotateMotor(motor2, -1*adim2);
  delay(motor3_guvenlik_suresi);
  rotateMotor(motor3, -1*adim3);
  delay(tekil_gorev_suresi);
  //kutu orijinde
  double aci1, aci2 = inverse_kinematics(x,y);
  int adim1 = acidan_adim(aci1);
  int adim2 = acidan_adim(aci2);
  int adim3 = dusey_adim(z);
  rotateMotor(motor1, adim1);
  rotateMotor(motor2, adim2);
  delay(motor3_guvenlik_suresi);
  rotateMotor(motor3, adim3);
  delay(tekil_gorev_suresi);
  gripper_ac();
  delay(gripper_delay);
  //kutu bırakıldı
  rotateMotor(motor1, -1*adim1);
  rotateMotor(motor2, -1*adim2);
  delay(motor3_guvenlik_suresi);
  rotateMotor(motor3, -1*adim3);
  delay(tekil_gorev_suresi);
}

void setup() {
  // Step motor hızını ayarla (isteğe bağlı)
  motor1.setSpeed(MOTOR_SPEED); // Adım/saniye
  motor2.setSpeed(MOTOR_SPEED); // Adım/saniye
  motor3.setSpeed(MOTOR_SPEED); // Adım/saniye
  servoMotor.attach(SERVO_PIN);

  // Serial Monitor başlat
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n');
    int numHouses = parseJSON(message.c_str());
  }
  for (int currentFloor = 1; currentFloor <= 4; currentFloor++) {
    for (int ev_indexi = 0; ev_indexi < numHouses; ev_indexi++) {
      if (evler[ev_indexi].floor >= currentFloor) {
        // Ev hesaba dahil edilecek
        // İlgili işlemler burada yapılır
        // Örneğin, evin floor değerini kullanarak işlemler yapabilirsiniz
        // Switch-case yapısı kullanarak colorName'e göre işlemler yapabilirsiniz                         
        double x = evler[ev_indexi].x;
        double y = evler[ev_indexi].y;
        double z = kutu_alma_yuksekligi + release_safety_mm + (currentFloor - 1) * (ev_boyut);
        switch (evler[ev_indexi].colorName) {
          case "Black":
            if (!isBoxUsed[0][0]) {
              // Rengin ilk kutusu henüz kullanılmadı
              double a = BLACK_BOX_1_X;
              double b = BLACK_BOX_1_Y;
              isBoxUsed[0][0] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][1]) {
              double a = BLACK_BOX_2_X;
              double b = BLACK_BOX_2_Y;
              isBoxUsed[0][1] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][2]) {
              double a = BLACK_BOX_3_X;
              double b = BLACK_BOX_3_Y;
              isBoxUsed[0][2] = true;  // Kullanıldı olarak işaretleyin
            }
            break; 
          case "Red":
            if (!isBoxUsed[0][0]) {
              // Rengin ilk kutusu henüz kullanılmadı
              double a = RED_BOX_1_X;
              double b = RED_BOX_1_Y;
              isBoxUsed[0][0] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][1]) {
              double a = RED_BOX_2_X;
              double b = RED_BOX_2_Y;
              isBoxUsed[0][1] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][2]) {
              double a = RED_BOX_3_X;
              double b = RED_BOX_3_Y;
              isBoxUsed[0][2] = true;  // Kullanıldı olarak işaretleyin
            } 
            break;
          case "Green":
            if (!isBoxUsed[0][0]) {
              // Rengin ilk kutusu henüz kullanılmadı
              double a = GREEN_BOX_1_X;
              double b = GREEN_BOX_1_Y;
              isBoxUsed[0][0] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][1]) {
              double a = GREEN_BOX_2_X;
              double b = GREEN_BOX_2_Y;
              isBoxUsed[0][1] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][2]) {
              double a = GREEN_BOX_3_X;
              double b = GREEN_BOX_3_Y;
              isBoxUsed[0][2] = true;  // Kullanıldı olarak işaretleyin
            } 
            break;
          case "Blue":
            if (!isBoxUsed[0][0]) {
              // Rengin ilk kutusu henüz kullanılmadı
              double a = BLUE_BOX_1_X;
              double b = BLUE_BOX_1_Y;
              isBoxUsed[0][0] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][1]) {
              double a = BLUE_BOX_2_X;
              double b = BLUE_BOX_2_Y;
              isBoxUsed[0][1] = true;  // Kullanıldı olarak işaretleyin
            } else if (!isBoxUsed[0][2]) {
              double a = BLUE_BOX_3_X;
              double b = BLUE_BOX_3_Y;
              isBoxUsed[0][2] = true;  // Kullanıldı olarak işaretleyin
            } 
            break;
        }
        Task(a,b,x,y,z);
      }
    }
  }
}
