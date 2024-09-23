CC = gcc
CFLAGS = -I/media/daten/Anwendungen/casadi-3.6.5-linux64-matlab2018b/include
LDFLAGS = -L/media/daten/Anwendungen/casadi-3.6.5-linux64-matlab2018b
LDLIBS = -lcasadi -lm

TARGET = main_mpc_udp

all: $(TARGET)

$(TARGET): $(TARGET).c $(shell find ../s_functions/ur5e_6dof/mpc_c_sourcefiles -name '*.c')
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)