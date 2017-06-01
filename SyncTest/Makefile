CC	=	gcc
EXE = synctest
SRC = src/synctest.c src/keyb.c src/userparams.c
COPTS	=	-DLINUX -Wall
FLAGS	=	
LIBS	=	-lCAENDigitizer -lm
INCLUDEDIR =    -I/usr/include # adapt to your include path
OBJS = *.o $(EXE)

#########################################################################

all	:	$(EXE)

clean	:
		/bin/rm -f $(OBJS) 

$(EXE)	:	$(SRC)
		/bin/rm -f $(BINLIB)
		$(CC) $(SRC) $(COPTS) $(INCLUDEDIR) $(FLAGS) -o $(EXE) $(LIBS) 

%.o	:	%.c
		$(CC) $(COPTS) $(INCLUDEDIR) -c -o $@ $<

