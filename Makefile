CFLAGS = -O3  -Wall
main.exe: main.o Manager.o layer.o 
	g++ -o main.exe main.o Manager.o layer.o

main.o: main.cpp Manager.h
	g++ -c $(CFLAGS) main.cpp

layer.o: layer.cpp layer.h
	g++ -c $(CFLAGS) layer.cpp

Manager.o: Manager.cpp Manager.h layer.h
	g++ -c $(CFLAGS) Manager.cpp



clean:
	rm -f *.o 
