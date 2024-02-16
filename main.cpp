#include "ACAN2517FD.h"
#include "CANSerialBridge.hpp"
#include "MDCClient.hpp"
#include "MbedHardwareSPI.h"
#include "cstdio"
#include "mbed.h"
#include "wheel.hpp"

#include "Controller.hpp"
#include <MbedHardwareSerial.hpp>
#include <SerialBridge.hpp>

#define HINATA_SCLK PB_13
#define HINATA_MISO PB_14
#define HINATA_MOSI PB_15
#define HINATA_CS PB_1
#define HINATA_InterruptIn PB_2
#define HINATA_LED PC_4
#define SERIAL_TX PA_15
#define SERIAL_RX PB_7
/*SPI通信のピン配置*/

#define Aircylinder1 PB_10
#define Aircylinder2 PA_8
#define DIR PA_9
#define PWM1 PC_7
#define PWM2 PB_6
/*可動部のピン配置*/

#define LED1 PA_5
#define LED2 PA_6
#define LED3 PA_7
/*デバック用LEDのピン配置*/

#define PI 3.1415

SerialDev *dev = new MbedHardwareSerial(new BufferedSerial(USBTX, USBRX, 9600));
SerialBridge serial(dev);

SPI spi(HINATA_MOSI, HINATA_MISO, HINATA_SCLK);
DigitalOut SLED(HINATA_LED);
DigitalIn sii(HINATA_InterruptIn);

DigitalOut Air(Aircylinder1);
DigitalOut dir(DIR);
PwmOut pwm1(PWM1), pwm2(PWM2);

DigitalOut led1(LED1), led2(LED2), led3(LED3);

Timer timer;

uint32_t getMillisecond() {
  return (uint32_t)duration_cast<std::chrono::milliseconds>(
             timer.elapsed_time())
      .count();
}

MbedHardwareSPI dev_spi(spi, HINATA_CS);
ACAN2517FD dev_can(dev_spi, getMillisecond);
CANSerialBridge can_serial(&dev_can);

MDCClient mdc_client0(&can_serial, 0);
MDCClient mdc_client1(&can_serial, 1);

Control msg;

Wheel wheel1(0.396, PI * 0.138, PI / 2, PI / 4),
    wheel2(0.396, PI * 0.862, PI / 2, PI * 3 / 4),
    wheel3(0.396, PI * 1.138, PI / 2, PI * 5 / 4),
    wheel4(0.396, PI * 1.862, PI / 2, PI * 7 / 4);

int main() {
    float joystick_x_raw, joystick_yraw, joystick_turn_raw;
  float joystick_x, joystick_y, joystick_turn;
  float moter_speed_raw;
  float moter_speed;
  bool roller_status;
  bool shoot_bottom;
  bool arm_up, arm_down;
  bool hand_status;

  serial.add_frame(0, &msg);

  timer.start();

  //  set up
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_4MHz, 500UL * 1000UL,
                              DataBitRateFactor::x2);

  settings.mRequestedMode = ACAN2517FDSettings::NormalFD;

  settings.mDriverTransmitFIFOSize = 4;
  settings.mDriverReceiveFIFOSize = 3;

  const uint32_t errorCode0 = dev_can.begin(settings);
  if (errorCode0 == 0) {
    printf("Initializing process completed.\n\r");
  } else {
    printf("Configuration error 0x%x\n\r", errorCode0);
  }

  setting_struct_t mdc_setting = {
      OperatorMode::MD_OPERATOR, EncoderType::VELOCITY,
      //  分解能
      1,
      //  反転するか(回転方向が合わない場合にトグルしてください。)
      false,
      //  Pゲイン
      0,
      //  Iゲイン
      0,
      //  Dゲイン
      0,
      //  フォワードゲイン(1 / 最大速度)
      //  設定しなくても大丈夫です。応答性が遅い場合に設定すると効果的
      0,
      //  電流制御用ゲイン
      0, 0, 0

  };

  //  0番目(回路側では1番)のモータ動作を設定します。
  mdc_client0.update_setting(0, mdc_setting);
  mdc_client0.update_setting(1, mdc_setting);
  mdc_client0.update_setting(2, mdc_setting);
  mdc_client0.update_setting(3, mdc_setting);
  while (true) {
    //  update and read

    mdc_client1.update_setting(0, mdc_setting);
    mdc_client1.update_setting(1, mdc_setting);
    mdc_client1.update_setting(2, mdc_setting);
    mdc_client1.update_setting(3, mdc_setting);
    serial.update();

    if (msg.was_updated()) {

      joystick_x = msg.data.joystick_x;
      joystick_x = static_cast<float>(joystick_x_raw / 100);
      joystick_y = msg.data.joystick_y;
      joystick_y = static_cast<float>(joystick_y / 100);
      joystick_turn = static_cast<float>(joystick_turn / 100);
      joystick_turn = msg.data.joystick_turn;
      moter_speed = msg.data.moter_speed;
      moter_speed = static_cast<float>(moter_speed / 100);
      roller_status = msg.data.roller_status;
      shoot_bottom = msg.data.shoot_bottom;
      arm_up = msg.data.arm_up;
      arm_down = msg.data.arm_down;
      hand_status = msg.data.hand_status;

      printf("%d\n\r", msg.data.joystick_x);

      mdc_client0.set_target(
          0, wheel1.get_velocity(joystick_x, joystick_y, joystick_turn));
      mdc_client0.set_target(
          1, wheel2.get_velocity(joystick_x, joystick_y, joystick_turn));
      mdc_client0.set_target(
          2, wheel3.get_velocity(joystick_x, joystick_y, joystick_turn));
      mdc_client0.set_target(
          3, wheel4.get_velocity(joystick_x, joystick_y, joystick_turn));
      mdc_client0.send_target();

      if (roller_status == true) {
        mdc_client1.set_target(0, fabs(moter_speed));
        mdc_client1.set_target(1, fabs(moter_speed));
      }

      if (shoot_bottom == true)
        mdc_client1.set_target(2, 0.5);
      else
        mdc_client1.set_target(2, 0);

      if (arm_up == true && arm_down || false)
        mdc_client1.set_target(3, 0.5);
      else if (arm_up == true && arm_down || false)
        mdc_client1.set_target(3, -0.5);
      else
        mdc_client1.set_target(3, 0);

      if (hand_status == true)
        Air = 1;
      else
        Air = 0;
    }

    dev_can.poll();
    can_serial.update();
    wait_us(100000);
  }
}