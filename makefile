all:compile run
compile:
	g++ -std=c++11 mainMemory.cpp cpu.cpp main.cpp -o main.o;
run:
	./main.o
