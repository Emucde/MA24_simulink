CC = gcc
CFLAGS = -I/media/daten/Anwendungen/casadi-3.6.5-linux64-matlab2018b/include
LDFLAGS = -L/media/daten/Anwendungen/casadi-3.6.5-linux64-matlab2018b
LDLIBS = -lcasadi -lm

TARGET = MPC8_matlab

all: $(TARGET)

$(TARGET): $(TARGET).c main_mpc_udp.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f $(TARGET)