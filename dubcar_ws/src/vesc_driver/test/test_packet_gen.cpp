#include <gtest/gtest.h>
#include "bldc_interface.h"
#include <cstdio>
#include "vesc_driver/vesc_packet.h"

#define PRINT_BUF(buf, len) {\
  for (int i = 0; i < len; i++)\
    printf("%02x", (buf)[i]);\
  printf("\n");\
}

#define EXPECT_EQ_BUF(buf_a, buf_b, len) {\
for (int i = 0; i < len; i++)\
  EXPECT_EQ((buf_a)[i], (buf_b)[i]);\
}



TEST(SendsCorrectPackets, DutyCycle) {

  unsigned char *truth1 = bldc_interface_set_duty_cycle(0);
  Buffer res1 = vesc_driver::VescPacket::createDutyCycleCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_duty_cycle(12);
  Buffer res2 = vesc_driver::VescPacket::createDutyCycleCmd(12).getBuffer();

  unsigned char *truth3 = bldc_interface_set_duty_cycle(100.0);
  Buffer res3 = vesc_driver::VescPacket::createDutyCycleCmd(100.0).getBuffer();
 

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Current) {

  unsigned char *truth1 = bldc_interface_set_current(0.0);
  Buffer res1 = vesc_driver::VescPacket::createCurrentCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_current(12.1);
  Buffer res2 = vesc_driver::VescPacket::createCurrentCmd(12.1).getBuffer();

  unsigned char *truth3 = bldc_interface_set_current(100.0);
  Buffer res3 = vesc_driver::VescPacket::createCurrentCmd(100.0).getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, CurrentBrake) {

  unsigned char *truth1 = bldc_interface_set_current_brake(0);
  Buffer res1 = vesc_driver::VescPacket::createCurrentBrakeCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_current_brake(12);
  Buffer res2 = vesc_driver::VescPacket::createCurrentBrakeCmd(12).getBuffer();

  unsigned char *truth3 = bldc_interface_set_current_brake(100.0);
  Buffer res3 = vesc_driver::VescPacket::createCurrentBrakeCmd(100.0).getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, RPM) {

  unsigned char *truth1 = bldc_interface_set_rpm(0);
  Buffer res1 = vesc_driver::VescPacket::createRpmCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_rpm(12);
  Buffer res2 = vesc_driver::VescPacket::createRpmCmd(12).getBuffer();

  unsigned char *truth3 = bldc_interface_set_rpm(100);
  Buffer res3 = vesc_driver::VescPacket::createRpmCmd(100).getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Pos) {
  unsigned char *truth1 = bldc_interface_set_pos(0);
  Buffer res1 = vesc_driver::VescPacket::createPositionCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_pos(12);
  Buffer res2 = vesc_driver::VescPacket::createPositionCmd(12).getBuffer();

  unsigned char *truth3 = bldc_interface_set_pos(100.0);
  Buffer res3 = vesc_driver::VescPacket::createPositionCmd(100.0).getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, ServoPos) {

  unsigned char *truth1 = bldc_interface_set_servo_pos(0);
  Buffer res1 = vesc_driver::VescPacket::createServoPositionCmd(0).getBuffer();

  unsigned char *truth2 = bldc_interface_set_servo_pos(12);
  Buffer res2 = vesc_driver::VescPacket::createServoPositionCmd(12).getBuffer();

  unsigned char *truth3 = bldc_interface_set_servo_pos(100.0);
  Buffer res3 = vesc_driver::VescPacket::createServoPositionCmd(100.0).getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());
  EXPECT_EQ_BUF(res2, truth2, res2.size());
  EXPECT_EQ_BUF(res3, truth3, res3.size());

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Reboot) {

  unsigned char *truth1 = bldc_interface_reboot();
  Buffer res1 = vesc_driver::VescPacket::createRebootCmd().getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());

  free(truth1);

}

TEST(SendsCorrectPackets, SendAlive) {

  unsigned char *truth1 = bldc_interface_send_alive();
  Buffer res1 = vesc_driver::VescPacket::createSendAliveCmd().getBuffer();

  EXPECT_EQ_BUF(res1, truth1, res1.size());

  free(truth1);
}







int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
