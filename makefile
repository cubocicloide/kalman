MAIN = main

CMD = sudo ./$(MAIN)

CC = g++

CFLAGS = -std=c++11 -Wall -pthread -lrt -lm -I

SOURCE = $(MAIN).cpp 

LINK1 = /usr/include/eigen3/
LINK2 = `pkg-config --cflags --libs allegro`

$(MAIN): 
	$(CC) $(CFLAGS) $(LINK1) $(SOURCE) -o $(MAIN) $(LINK2)

clean:
	rm -rf *o $(MAIN)

run:
	$(CMD)