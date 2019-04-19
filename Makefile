src = $(wildcard *.c)
obj = $(src:.c=.o)
CFLAGS = -O2 -Wall

ifneq ($(shell grep -m 1 -o BCM2835 /proc/cpuinfo),)
	DFLAGS = -lbcm2835 -D ENBCM
endif

LDFLAGS = -lpthread -lm

.PHONY: all
tcp_srv: $(obj)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(DFLAGS)
	rm -f $(obj)

.PHONY: clean
clean:
	rm -f $(obj) tcp_srv
