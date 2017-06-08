CFLAGS = -std=c++11 -g  -Wall
main.exe: main.o Manager.o layer.o cluster.o
	g++ -o main.exe main.o Manager.o layer.o cluster.o

main.o: main.cpp Manager.h
	g++ -c $(CFLAGS) main.cpp

cluster.o: cluster.cpp cluster.h fiboheap.h fiboqueue.h
	g++ -c $(CFLAGS) cluster.cpp

layer.o: layer.cpp layer.h cluster.h
	g++ -c $(CFLAGS) layer.cpp

Manager.o: Manager.cpp Manager.h layer.h
	g++ -c $(CFLAGS) Manager.cpp



clean:
	rm -f *.o 
