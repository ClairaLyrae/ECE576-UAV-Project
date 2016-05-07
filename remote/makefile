COMP            = g++ 
FLAGS           = -g -Wall -std=c++11

# Use the following for ece3.ece.arizona.edu (compute.engr.arizona.edu)   
LIBS            = -L/usr/local/systemc/lib-linux64 -lsystemc
INCS            = -I/usr/local/systemc/include -I../gmtl-0.6.1 

OBJ_LIST        = Main.o
TARGET_NAME     = Main

all: $(TARGET_NAME)

$(TARGET_NAME): $(OBJ_LIST)
	$(COMP) $(FLAGS) -o $(TARGET_NAME) $(OBJ_LIST) $(LIBS)

%.o: %.cc %.h
	$(COMP) $(FLAGS) $(INCS) -c $<

%.o: %.cpp %.h
	$(COMP) $(FLAGS) $(INCS) -c $<

%.o: %.cc
	$(COMP) $(FLAGS) $(INCS) -c $<

%.o: %.cpp
	$(COMP) $(FLAGS) $(INCS) -c $<

clean:
	rm -f *.o *~ core $(TARGET_NAME)
