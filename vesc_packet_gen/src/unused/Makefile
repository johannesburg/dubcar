CPPFLAGS += -MD -MP -fno-stack-protector 

SRC = $(wildcard *.c)

hello_world: $(SRC:%.c=%.o)
	gcc -o $@ $^

clean:
	rm *.o

-include $(SRC:%.c=%.d)
