CFLAGS = -O3 -Wall -fopenmp 
net_open_finder: main.o Manager.o layer.o cluster.o sgc.o layer_dff.o
	g++ -o net_open_finder -fopenmp main.o Manager.o layer.o cluster.o sgc.o layer_dff.o

main.o: main.cpp Manager.h
	g++ -c $(CFLAGS) main.cpp

cluster.o: cluster.cpp cluster.h fiboheap.h
	g++ -c $(CFLAGS) cluster.cpp

layer.o: layer.cpp layer.h cluster.h fiboheap.h
	g++ -c $(CFLAGS) layer.cpp

layer_dff.o: layer_dff.cpp layer.h cluster.h fiboheap.h
	g++ -c $(CFLAGS) layer_dff.cpp

Manager.o: Manager.cpp Manager.h layer.h
	g++ -c $(CFLAGS) Manager.cpp

sgc.o:sgc.cpp Manager.h
	g++ -c $(CFLAGS) sgc.cpp

clean:
	rm -f *.o 
