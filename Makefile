maverick : maverick.c
	gcc $(CFLAGS) -lwiringPi maverick.c -o maverick

clean:
	rm maverick
