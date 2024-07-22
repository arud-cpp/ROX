#include <Arduino.h> //for PlatformI/O https://www.media.lab.uec.ac.jp/?page_id=1414
#include <ps5Controller.h> //https://github.com/rodneybakiskan/ps5-esp32
#include <DDT_Motor_M15M06.h> //https://github.com/takex5g/M5_DDTMotor_M15M06
#include <M5Unified.h>
#include <FastLED.h>
#include <Preferences.h>

int16_t targetSpeed[4];
int16_t actualSpeed[4];
uint8_t Brake_Disable = 0;
uint8_t Brake_Enable = 0xFF;
uint8_t Acce = 1;
uint8_t Brake_P = 0;
uint8_t ID = 1;
uint8_t brake = Brake_Disable;
const int16_t POS_MAX = 32767;
Receiver Receiv;
auto motor_handler = MotorHandler(27, 19); //M5Tough用RS485ポート設定

#define NUM_LEDS 200
CRGB leds[NUM_LEDS];
#define LED_DATA_PIN 25 //M5Tough用LEDポート設定

Preferences preferences; //NVS(不揮発性ストレージ)を使用して設定情報を記憶する
bool ps5_connected = false;
#define DEFAULT_DS_MAC "4C:B9:9B:64:76:1A" //自分のDualSenseに合わせて変更する（または起動時のUIで変更する）
String ps5_mac_address = DEFAULT_DS_MAC;

int currentColorIndex = 0;
const CRGB colorOptions[] = {CRGB::White, CRGB::Red, CRGB::Green, CRGB::Blue, CRGB::Cyan, CRGB::Magenta, CRGB::Yellow};
const int colorOptionsCount = sizeof(colorOptions) / sizeof(colorOptions[0]);

void LEDArray(CRGB color, int brightness) {
  if(brightness > 20) brightness = 20;
  if(brightness < 0) brightness = 0;
  FastLED.setBrightness(brightness);
  for(int i=0; i<NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void setup() {
  FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  LEDArray(CRGB::White, 20);
  
  M5.begin();

  // NVSを初期化し、保FV存されたMACアドレスを読み込む
  preferences.begin("myApp", false);
  currentColorIndex = preferences.getInt("colorIndex", 0);
  ps5_mac_address = preferences.getString("mac", DEFAULT_DS_MAC);
  preferences.end();

  //モーターを停止しておく
  for(int i=1; i<=4; i++) {
    motor_handler.Control_Motor(0, i, Acce, Brake_P, &Receiv);
    delay(5);
  }

  connect_dualsense(false);
  LEDArray(colorOptions[currentColorIndex], 20);
  display_motor_speeds(true);  // 初回表示
}

void loop() { //メインループ　必要な処理に応じて改造し使用してください
  M5.update();
  LEDArray(colorOptions[currentColorIndex], 20);
  ps5_connected = ps5.isConnected();

  if (ps5_connected) {  
    handle_dualsense_controller();
  }

  display_motor_speeds(false);
  check_buttons();
  delay(1);
}

void handle_dualsense_controller() {
  if (ps5.Cross()) {
    motor_brake();
  }  
  else if(ps5.L2()) {
    int speed_limit = 115;
    int rotation_limit = (int)(speed_limit * 0.3);
    int val_LStickX = ps5.LStickX();
    int val_LStickY = ps5.LStickY();
    if(val_LStickX > -16 && val_LStickX < 16) val_LStickX = 0;
    if(val_LStickY > -20 && val_LStickY < 20) val_LStickY = 0;

    int left_vel = (int)(speed_limit*val_LStickY/128)+(int)(rotation_limit*val_LStickX/128);
    int right_vel = (int)(speed_limit*val_LStickY/128)-(int)(rotation_limit*val_LStickX/128);
    vehicle_steer(right_vel, left_vel);
  } 
  else if(ps5.R2()) {
    int speed_limit = 115;
    int val_RStickX = ps5.RStickX();
    int val_RStickY = ps5.RStickY();

    int rf_vel = -(int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
    int rb_vel = (int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
    int lf_vel = (int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
    int lb_vel = -(int)(speed_limit*val_RStickX/128)+(int)(speed_limit*val_RStickY/128);
    vehicle_omni(rf_vel, rb_vel, lf_vel, lb_vel);
  } 
  else {
    motor_stop();
  }
}

void motor_exec() {
  for(int i=0; i<4; i++) {
    motor_handler.Control_Motor(targetSpeed[i], i+1, Acce, brake, &Receiv);
    actualSpeed[i]=Receiv.BSpeed;
    delay(5);
  }
}

void motor_stop() {
  targetSpeed[0]=targetSpeed[2]=0;
  targetSpeed[1]=targetSpeed[3]=0;
  brake = Brake_Disable;
  motor_exec();
}

void motor_brake() {
  targetSpeed[0]=targetSpeed[2]=0;
  targetSpeed[1]=targetSpeed[3]=0;
  brake = Brake_Enable;
  motor_exec();
}

void vehicle_steer(int right, int left) {
  targetSpeed[0]=targetSpeed[2]=-right;
  targetSpeed[1]=targetSpeed[3]=left;
  brake = Brake_Disable;
  motor_exec();
} 

void vehicle_omni(int rf_vel, int rb_vel, int lf_vel, int lb_vel) {
  targetSpeed[0]=-rf_vel;
  targetSpeed[1]=lf_vel;
  targetSpeed[2]=-rb_vel;
  targetSpeed[3]=lb_vel;
  brake = Brake_Disable;
  motor_exec();
} 

void set_motor_id(uint8_t id) {
  for (int i = 0; i < 5; i++) {
    motor_handler.Set_MotorID(id);
    delay(5);
  }
}


bool confirm_id_setup() {
  M5.Display.clear();
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Set Motor ID?");

  // YESボタンの描画
  M5.Display.drawRect(40, 100, 100, 50, WHITE);
  M5.Display.setCursor(73, 118);
  M5.Display.print("YES");

  // NOボタンの描画
  M5.Display.drawRect(180, 100, 100, 50, WHITE);
  M5.Display.setCursor(220, 118);
  M5.Display.print("NO");

  while (true) {
    M5.update();

    if (M5.Touch.getCount()) {
      auto touch = M5.Touch.getDetail();
      if (touch.wasPressed()) {
        if (touch.x >= 40 && touch.x < 140 && touch.y >= 100 && touch.y < 150) {
          return true;  // YESが押された
        }
        if (touch.x >= 180 && touch.x < 280 && touch.y >= 100 && touch.y < 150) {
          return false;  // NOが押された
        }
      }
    }

    delay(100);
  }
}

void select_motor_id() {
  M5.Display.clear();
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Select Motor ID:");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0, 20);
  M5.Display.println("* Please connect only one motor before setting ID.");
  M5.Display.setTextSize(2);

  int selected_id = 0;
  bool id_selected = false;

  // ボタンの定義
  const int btnWidth = 66;
  const int btnHeight = 40;
  const int btnMargin = 10;
  const int startX = 10;
  const int startY = 40;

  // IDボタンの描画
  for (int i = 0; i < 4; i++) {
    int x = startX + (btnWidth + btnMargin) * i;
    int y = startY;
    M5.Display.drawRect(x, y, btnWidth, btnHeight, WHITE);
    M5.Display.setCursor(x + 28, y + 12);
    M5.Display.printf("%d", i + 1);
  }

  // OKボタンの描画
  M5.Display.drawRect(startX, startY + (btnHeight + btnMargin) * 2, btnWidth * 2 + btnMargin, btnHeight, WHITE);
  M5.Display.setCursor(startX + 38, startY + (btnHeight + btnMargin) * 2 + 12);
  M5.Display.print("Set ID");

  // キャンセルボタンの描画
  M5.Display.drawRect(startX + btnWidth * 2 + btnMargin * 2, startY + (btnHeight + btnMargin) * 2, btnWidth * 2 + btnMargin, btnHeight, WHITE);
  M5.Display.setCursor(startX + btnWidth * 2 + btnMargin * 2 + 38, startY + (btnHeight + btnMargin) * 2 + 12);
  M5.Display.print("Cancel");

  while (!id_selected) {
    M5.update();

    if (M5.Touch.getCount()) {
      auto touch = M5.Touch.getDetail();
      if (touch.wasPressed()) {
        // IDボタンの判定
        for (int i = 0; i < 4; i++) {
          int x = startX + (btnWidth + btnMargin) * i;
          int y = startY;
          if (touch.x >= x && touch.x < x + btnWidth && touch.y >= y && touch.y < y + btnHeight) {
            selected_id = i + 1;
            break;
          }
        }

        // OKボタンの判定
        if (touch.x >= startX && touch.x < startX + btnWidth * 2 + btnMargin &&
            touch.y >= startY + (btnHeight + btnMargin) * 2 && touch.y < startY + (btnHeight + btnMargin) * 2 + btnHeight) {
          if (selected_id != 0) {
            id_selected = true;
          }
        }

        // キャンセルボタンの判定
        if (touch.x >= startX + btnWidth * 2 + btnMargin * 2 && touch.x < startX + btnWidth * 4 + btnMargin * 3 &&
            touch.y >= startY + (btnHeight + btnMargin) * 2 && touch.y < startY + (btnHeight + btnMargin) * 2 + btnHeight) {
          return;  // キャンセルして関数を終了
        }

        // 選択されたIDの表示を更新
        M5.Display.fillRect(10, 100, 310, 30, BLACK);  // 前の表示を消去
        M5.Display.setCursor(10, 100);
        M5.Display.printf("Selected ID: %d", selected_id);
      }
    }

    delay(100);
  }


  ID = selected_id;
  set_motor_id(ID);

  M5.Display.fillRect(0, 0, 320, 240, BLACK);  // 画面をクリア
  M5.Display.setCursor(0, 0);
  M5.Display.printf("Motor ID %d set\n", ID);
  delay(2000);
}
void connect_dualsense(bool showMacAddressChangeDialog) {
  const char* connecting_states[] = {"Connecting", "Connecting.", "Connecting..", "Connecting..."};
  int state_index = 0;
  unsigned long last_update = 0;
  const unsigned long update_interval = 1000; // 1秒ごとに更新
  if(showMacAddressChangeDialog){
    if (confirm_change_mac()) {
      String new_mac = input_mac_address();
      if (new_mac != "") {
        if (new_mac.length() == 17) {  // MACアドレスが正しい長さ（17文字）であることを確認
          ps5_mac_address = new_mac;
          // 新しいMACアドレスをNVSに保存
          preferences.begin("myApp", false);
          preferences.putString("mac", ps5_mac_address);
          preferences.end();
        } else {
          // エラーメッセージを表示
          M5.Display.fillRect(0, 0, 320, 240, BLACK);
          M5.Display.setTextSize(1);
          M5.Display.setTextColor(RED);
          M5.Display.setCursor(10, 100);
          M5.Display.println("Error: Invalid MAC address");
          M5.Display.setTextColor(WHITE);
          delay(2000);  // 2秒間エラーメッセージを表示
          // NVRAMの値を使用
          preferences.begin("myApp", false);
          ps5_mac_address = preferences.getString("mac", DEFAULT_DS_MAC);
          preferences.end();
        }
      } else {
        // キャンセルされた場合、接続を開始せずに関数を終了
        return;
      }
    }
  }

  M5.Display.fillRect(220, 0, 100, 30, BLACK);
  M5.Display.drawRect(220, 0, 100, 30, CYAN);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(CYAN);  // テキスト色をCYANに設定
  M5.Display.setCursor(225, 10);
  M5.Display.print("Connecting...");

  ps5.begin(ps5_mac_address.c_str());
  esp_log_level_set("ps5_L2CAP", ESP_LOG_VERBOSE);
  esp_log_level_set("ps5_SPP", ESP_LOG_VERBOSE);  

  // 接続を待つ（最大5秒）
  int attempts = 0;
  while (!ps5.isConnected() && attempts < 50) {
    unsigned long current_time = millis();
    if (current_time - last_update >= update_interval) {
      M5.Display.fillRect(220, 0, 100, 30, BLACK);
      int anim = (4-state_index);
      M5.Display.drawRect(220+anim, anim, 100-anim*2, 30-anim*2, CYAN);
      M5.Display.setCursor(225, 10);
      M5.Display.print(connecting_states[state_index]);
      state_index = (state_index + 1) % 4;  // 0から3まで繰り返す
      last_update = current_time;
    }
    delay(100);
    attempts++;
  }
  M5.Display.setTextColor(WHITE);  // テキスト色を元に戻す
  ps5_connected = ps5.isConnected();
  display_motor_speeds(true);  // 接続状態を反映するために再描画
}

bool confirm_change_mac() {
  M5.Display.clear();
  M5.Display.setTextSize(2);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Change MAC Address?");
  M5.Display.println("Current: " + ps5_mac_address);

  // YESボタンの描画
  M5.Display.drawRect(40, 100, 100, 50, WHITE);
  M5.Display.setCursor(70, 115);
  M5.Display.print("YES");

  // NOボタンの描画
  M5.Display.drawRect(180, 100, 100, 50, WHITE);
  M5.Display.setCursor(220, 115);
  M5.Display.print("NO");

  while (true) {
    M5.update();

    if (M5.Touch.getCount()) {
      auto touch = M5.Touch.getDetail();
      if (touch.wasPressed()) {
        if (touch.x >= 40 && touch.x < 140 && touch.y >= 100 && touch.y < 150) {
          return true;  // YESが押された
        }
        if (touch.x >= 180 && touch.x < 280 && touch.y >= 100 && touch.y < 150) {
          return false;  // NOが押された
        }
      }
    }

    delay(10);
  }
}

String input_mac_address() {
  String mac = ps5_mac_address;
  const char* hex_chars = "0123456789ABCDEF<C";  // <- はバックスペース、C はClear
  int cursor_pos = mac.length();

  M5.Display.clear();
  M5.Display.setTextSize(1.5);
  M5.Display.setCursor(0, 0);
  M5.Display.println("Enter DualSense MAC Address:");
  M5.Display.println(mac + "_");

  // 16進数キーボードの描画（5x4レイアウト）
  const int btnWidth = 60;
  const int btnHeight = 40;
  const int startX = 10;
  const int startY = 40;

  auto drawKeyboard = [&]() {
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 5; j++) {
        int index = i * 5 + j;
        M5.Display.drawRect(startX + j * btnWidth, startY + i * btnHeight, btnWidth, btnHeight, WHITE);
        M5.Display.setTextSize(1.5);
        M5.Display.setCursor(startX + j * btnWidth + 25, startY + i * btnHeight + 14);
        if (index < 16) {
          M5.Display.print(hex_chars[index]);
        } else if (index == 16) {
          M5.Display.print("<");
        } else if (index == 17) {
          M5.Display.print("C");
        }
      }
    }

    // CancelとOKボタンの描画
    M5.Display.drawRect(10, 210, 100, 30, WHITE);
    M5.Display.setCursor(40, 220);
    M5.Display.print("Cancel");

    M5.Display.drawRect(210, 210, 100, 30, WHITE);
    M5.Display.setCursor(250, 220);
    M5.Display.print("OK");
  };

  drawKeyboard();

  while (true) {
    M5.update();

    if (M5.Touch.getCount()) {
      auto touch = M5.Touch.getDetail();
      if (touch.wasPressed()) {
        int col = (touch.x - startX) / btnWidth;
        int row = (touch.y - startY) / btnHeight;
        
        if (row >= 0 && row < 4 && col >= 0 && col < 5) {
          int index = row * 5 + col;
          if (index < 16) {
            if (cursor_pos < 17) {
              if (cursor_pos == 2 || cursor_pos == 5 || cursor_pos == 8 || cursor_pos == 11 || cursor_pos == 14) {
                if (cursor_pos == mac.length()) mac += ":";
                cursor_pos++;
              }
              if (cursor_pos == mac.length()) mac += hex_chars[index];
              else mac.setCharAt(cursor_pos, hex_chars[index]);
              cursor_pos++;
            }
          } else if (index == 16) {  // Backspace
            if (!mac.isEmpty() && cursor_pos > 0) {
              if (cursor_pos == 3 || cursor_pos == 6 || cursor_pos == 9 || cursor_pos == 12 || cursor_pos == 15) {
                mac = mac.substring(0, mac.length() - 2);
                cursor_pos -= 2;
              } else {
                mac = mac.substring(0, mac.length() - 1);
                cursor_pos--;
              }
            }
          } else if (index == 17) {  // Clear
            mac = "";
            cursor_pos = 0;
          }
        } else if (touch.y >= 210 && touch.y < 260) {  // CancelとOKボタンの判定
          if (touch.x < 150) {  // Cancel
            return "";
          } else if (touch.x > 170) {  // OK
            if (mac.length() == 17) {
              return mac;
            }
          }
        }

        M5.Display.fillRect(0, 11, 320, 25, BLACK);
        M5.Display.setTextSize(1.5);
        M5.Display.setCursor(0, 0);
        M5.Display.println();
        M5.Display.println(mac + "_");
      }
    }

    delay(10);
  }
}

void display_motor_speeds(bool force_redraw) {
  static int16_t last_speed[4] = {0, 0, 0, 0};
  static bool last_ps5_state = false;

  if (force_redraw) {
    M5.Display.clear();
    M5.Display.setTextSize(2);
    M5.Display.setCursor(0, 0);
    M5.Display.println("Motor Speed:");
    
   // モーターIDセットボタンの描画
    M5.Display.drawRect(0, 190, 130, 40, WHITE);
    M5.Display.setTextSize(1.5);
    M5.Display.setCursor(10, 205);
    M5.Display.print("Set Motor ID");

    // リセットボタンの描画
    M5.Display.drawRect(250, 190, 70, 40, WHITE);
    M5.Display.setCursor(265, 205);
    M5.Display.print("Reset");

    M5.Display.setTextSize(2);
  }

  // PS5接続状態ボタンの描画
  M5.Display.drawRect(220, 0, 100, 30, WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(225, 10);
  if (ps5_connected != last_ps5_state || force_redraw) {
    M5.Display.fillRect(221, 1, 98, 28, BLACK);
    if (ps5_connected) {
      M5.Display.setTextColor(CYAN);
      M5.Display.print("DS Connected");
    } else {
      M5.Display.setTextColor(RED);
      M5.Display.print("DS Disconnected");
    }
    last_ps5_state = ps5_connected;
  }
  M5.Display.setCursor(218, 35);
  M5.Display.print(ps5_mac_address);

  // Color Changeボタンの描画
  M5.Display.drawRect(220, 60, 100, 30, WHITE);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(225, 70);
  M5.Display.setTextColor(WHITE);
  M5.Display.print("LED ColorChange");

  for (int i = 0; i < 4; i++) {
    if (actualSpeed[i] != last_speed[i] || force_redraw) {
      // Clear the previous speed display
      M5.Display.fillRect(0, 30 + i * 30, 219, 30, BLACK);
      
      // Display the new speed
      M5.Display.setTextSize(2);
      M5.Display.setCursor(0, 30 + i * 30);
      M5.Display.printf("Motor %d: %d RPM", i + 1, actualSpeed[i]);
      
      last_speed[i] = actualSpeed[i];
    }
  }
}

void check_buttons() {
  if (M5.Touch.getCount()) {
    auto touch = M5.Touch.getDetail();
    if (touch.wasPressed()) {
      if (touch.x >= 0 && touch.x < 130 && touch.y >= 190 && touch.y < 230) {
        // モーターIDセットボタンが押された
        select_motor_id();
        display_motor_speeds(true);  // モーターIDセット後に画面を再描画
      }
      if (touch.x >= 250 && touch.x < 310 && touch.y >= 190 && touch.y < 230) {
        // リセットボタンが押された
        ESP.restart();
      }
      if (touch.x >= 220 && touch.x < 320 && touch.y >= 0 && touch.y < 30) {
        // PS5接続状態ボタンが押された
        if (!ps5_connected) {
          connect_dualsense(true);
        }
      }
      if (touch.x >= 220 && touch.x < 320 && touch.y >= 60 && touch.y < 90) {
        // Color Changeボタンが押された
        currentColorIndex = (currentColorIndex + 1) % colorOptionsCount;
        LEDArray(colorOptions[currentColorIndex], 20);
        
        // 新しい色のインデックスをNVRAMに保存
        preferences.begin("myApp", false);
        preferences.putInt("colorIndex", currentColorIndex);
        preferences.end();
      }
    }
  }
}