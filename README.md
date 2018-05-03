# kalman
Implementation of a discrete Kalman filter

da scaricare:
sudo apt-get install g++
sudo apt-get install libeigen3-dev
sudo apt-get install liballegro4.2‐dev
---
compilazione:
g++ -std=c++11 -Wall -pthread -lrt -lm -I /usr/include/eigen3/ main.cpp  -o main `pkg-config --cflags --libs allegro`
