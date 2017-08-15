CFLAGS = -g -Wall 
net_open_finder: main.o Manager.o layer.o cluster.o sgc.o
	g++ -o net_open_finder main.o Manager.o layer.o cluster.o sgc.o

main.o: main.cpp Manager.h
	g++ -c $(CFLAGS) main.cpp

cluster.o: cluster.cpp cluster.h fiboheap.h
	g++ -c $(CFLAGS) cluster.cpp

layer.o: layer.cpp layer.h cluster.h fiboheap.h
	g++ -c $(CFLAGS) layer.cpp

Manager.o: Manager.cpp Manager.h layer.h
	g++ -c $(CFLAGS) Manager.cpp

sgc.o:sgc.cpp Manager.h
	g++ -c $(CFLAGS) sgc.cpp

clean:
	rm -f *.o 
