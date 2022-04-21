PROG = raytrasdf
CFLAGS = -Wall -O2
LDFLAGS = -lm -lpthread -lSDL2
OBJS = raytrasdf.o loadstl.o tpool.o sdlout.o

$(PROG) : $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(CFLAGS) $(LDFLAGS)

raytrasdf.o: raytrasdf.c raytrasdf.h
	$(CC) $(CFLAGS) -c raytrasdf.c

loadstl.o: loadstl.c raytrasdf.h
	$(CC) $(CFLAGS) -c loadstl.c

tpool.o: tpool.c raytrasdf.h
	$(CC) $(CFLAGS) -c tpool.c

sdlout.o: sdlout.c raytrasdf.h
	$(CC) $(CFLAGS) -c sdlout.c

pic.ppm: $(PROG)
	./raytrasdf

clean:
	rm -f $(PROG) $(OBJS) pic.ppm

.PHONY: clean
