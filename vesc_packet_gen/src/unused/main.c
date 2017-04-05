
#include "bldc_interface.h"
#include <stdio.h>

int main(int argc, char **argv)
{
  printf("hello world\n");
  unsigned char * test = bldc_interface_set_current(5);
  bldc_interface_set_duty_cycle(4.0);
  printf("end\n");

  print_buffer(test, 512);
}
