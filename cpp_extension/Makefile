CC := g++
CFLAGS := -std=c++11 -std=gnu++11

main: main.cpp communication.o
	$(CC) $(CFLAGS) main.cpp communication.cpp -o libpycallclass.so -shared -fPIC

hc: hc_sensor.cpp
	$(CC) hc_sensor.cpp jetsonGPIO.c -o hc_sensor.so -shared -fPIC
	

clean:
	rm *.o libpycallclass.so hc_sensor.so
