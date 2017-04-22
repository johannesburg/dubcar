#include <gtest/gtest.h>
#include "bldc_interface.h"
#include <cstdio>

#define PRINT_BUF(buf, len) {\
  for (int i = 0; i < len; i++)\
    printf("%02x", (buf)[i]);\
  printf("\n");\
}


TEST(SendsCorrectPackets, DutyCycle) {

  unsigned char *truth1 = bldc_interface_set_duty_cycle(0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_duty_cycle(12);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,10)

  unsigned char *truth3 = bldc_interface_set_duty_cycle(100.0);
  unsigned char *res3 = truth3; // our_solution();
 

  EXPECT_EQ(0,1);
  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Current) {

  unsigned char *truth1 = bldc_interface_set_current(0.0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_current(12.1);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,10)

  unsigned char *truth3 = bldc_interface_set_current(100.0);
  unsigned char *res3 = truth3; // our_solution();

  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, CurrentBrake) {

  unsigned char *truth1 = bldc_interface_set_current_brake(0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_current_brake(12);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,10)

  unsigned char *truth3 = bldc_interface_set_current_brake(100.0);
  unsigned char *res3 = truth3; // our_solution();

  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, RPM) {

  unsigned char *truth1 = bldc_interface_set_rpm(0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_rpm(12);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,10)

  unsigned char *truth3 = bldc_interface_set_rpm(100.0);
  unsigned char *res3 = truth3; // our_solution();

  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Pos) {

  unsigned char *truth1 = bldc_interface_set_pos(0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_pos(12);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,10)

  unsigned char *truth3 = bldc_interface_set_pos(100.0);
  unsigned char *res3 = truth3; // our_solution();

  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, ServoPos) {

  unsigned char *truth1 = bldc_interface_set_servo_pos(0);
  unsigned char *res1 = truth1; // our_solution();

  unsigned char *truth2 = bldc_interface_set_servo_pos(12);
  unsigned char *res2 = truth2; // our_solution();

  PRINT_BUF(truth2,8)

  unsigned char *truth3 = bldc_interface_set_servo_pos(100.0);
  unsigned char *res3 = truth3; // our_solution();

  EXPECT_EQ(res1, truth1);
  EXPECT_EQ(res2, truth2);
  EXPECT_EQ(res3, truth3);

  free(truth1);
  free(truth2);
  free(truth3);
}

TEST(SendsCorrectPackets, Reboot) {

  unsigned char *truth1 = bldc_interface_reboot();
  unsigned char *res1 = truth1; // our_solution();

  PRINT_BUF(truth1,6)

  EXPECT_EQ(res1, truth1);


  free(truth1);

}

TEST(SendsCorrectPackets, SendAlive) {

  unsigned char *truth1 = bldc_interface_send_alive();
  unsigned char *res1 = truth1; // our_solution();

  PRINT_BUF(truth1,6)

  EXPECT_EQ(res1, truth1);

  free(truth1);
}







int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
