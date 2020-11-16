# Path to the solver source code
SRC = ../../src/
# Path to the problem source code
PROB = ./

# Header and souce for the model
MODELHDR =	$(PROB)px4Model.h

MODELSRCS =	$(PROB)px4Model.cc

# Main file of solver
SOLVERMAIN = $(PROB)px4Solver.cc

# Main file of simulator
SIMULATORMAIN = $(PROB)px4Simulator.cc

# Set compiler options for debugging etc.
#CXXFLAGS = -Wall -ggdb -O0
#CXXFLAGS = -Wall -ggdb -O0 -fopenmp
CXXFLAGS = -Wall -O3 -fopenmp

include $(SRC)common.mk
