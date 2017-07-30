CFLAGS = -O3 -Wall 
net_open_finder: main.o Manager.o layer.o layer2.o cluster.o sgc.o sgc2.o
	g++ -o net_open_finder main.o Manager.o layer.o layer2.o cluster.o sgc.o sgc2.o

main.o: main.cpp Manager.h
	g++ -c $(CFLAGS) main.cpp

cluster.o: cluster.cpp cluster.h fiboheap.h
	g++ -c $(CFLAGS) cluster.cpp

layer.o: layer.cpp layer.h cluster.h fiboheap.h
	g++ -c $(CFLAGS) layer.cpp

layer2.o: layer2.cpp layer.h
	g++ -c $(CFLAGS) layer2.cpp

Manager.o: Manager.cpp Manager.h layer.h
	g++ -c $(CFLAGS) Manager.cpp

sgc.o:sgc.cpp Manager.h
	g++ -c $(CFLAGS) sgc.cpp

sgc2.o:sgc2.cpp Manager.h 
	g++ -c $(CFLAGS) sgc2.cpp

clean:
	rm -f *.o 
