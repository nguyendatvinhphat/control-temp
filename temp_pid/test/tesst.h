// #include <PID_v1.h>

// // Cấu hình các chân kết nối với mạch mô-đun PID và cảm biến nhiệt độ
// const int pinNhiemVu = 9;     // Chân kết nối với mạch mô-đun PID
// const int pinNguonNgu = 8;    // Chân kết nối với mạch mô-đun PID
// const int pinCamBienNhiet = A0; // Chân kết nối với cảm biến nhiệt độ

// // Biến PID
// double setpoint, input, output;
// double Kp = 2, Ki = 5, Kd = 1; // Tham số PID ban đầu
// PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// void setup() {
//   pinMode(pinNhiemVu, OUTPUT);
//   pinMode(pinNguonNgu, OUTPUT);
//   pinMode(pinCamBienNhiet, INPUT);

//   // Khởi tạo giao tiếp Serial để theo dõi kết quả điều khiển nhiệt độ
//   Serial.begin(9600);

//   // Đặt nhiệt độ mục tiêu
//   setpoint = 250; // Thay đổi nhiệt độ mục tiêu theo nhu cầu của bạn
  
//   // Tự động điều chỉnh PID
//   AutoTunePID();

//   // Bắt đầu điều khiển PID
//   myPID.SetMode(AUTOMATIC);
// }

// void loop() {
//   // Đọc giá trị nhiệt độ từ cảm biến
//   input = readTemperature();

//   // Tính toán điều khiển PID
//   myPID.Compute();

//   // Điều khiển nhiệt độ mỏ hàn bằng cách điều chỉnh nhiệt độ nguồn ngu
//   analogWrite(pinNguonNgu, output);

//   // Hiển thị các giá trị trong Serial Monitor
//   Serial.print("Setpoint: ");
//   Serial.print(setpoint);
//   Serial.print(" | Input: ");
//   Serial.print(input);
//   Serial.print(" | Output: ");
//   Serial.println(output);

//   // Đợi một khoảng thời gian để thu thập dữ liệu
//   delay(1000);
// }

// double readTemperature() {
//   // Đọc giá trị ADC từ cảm biến nhiệt độ và chuyển đổi sang độ C
//   int rawValue = analogRead(pinCamBienNhiet);
//   double voltage = (rawValue / 1023.0) * 5.0;
//   double temperature = (voltage - 0.5) * 100.0;
//   return temperature;
// }

// void AutoTunePID() {
//   // Áp dụng thuật toán Ziegler-Nichols để tự động điều chỉnh các tham số PID
//   double Ku, Tu;
//   double tempKp, tempKi, tempKd;
//   double tempInput, tempOutput;

//   // Tắt điều khiển PID
//   myPID.SetMode(MANUAL);
//   analogWrite(pinNguonNgu, 0);

//   // Tìm hệ số tự động điều chỉnh
//   Ku = 0.2; // Thay đổi giá trị này nếu cần thiết

//   // Tìm khoảng thời gian tự động điều chỉnh
//   int tempTime = millis();
//   while (true) {
//     tempOutput = Ku * 255.0;
//     analogWrite(pinNguonNgu, tempOutput);
//     delay(200);
//     tempInput = readTemperature();
//     if (tempInput > setpoint) {
//       Tu = millis() - tempTime;
//       break;
//     }
//   }

//   // Tính toán các tham số PID theo phương pháp Ziegler-Nichols
//   tempKp = 0.6 * Ku;
//   tempKi = 2.0 * tempKp / Tu;
//   tempKd = tempKp * Tu / 8.0;

//   // Áp dụng các tham số PID đã tính được
//   myPID.SetTunings(tempKp, tempKi, tempKd);
//   myPID.SetMode(AUTOMATIC);
// }