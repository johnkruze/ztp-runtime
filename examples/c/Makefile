# from examples/c, after: cd ../.. && cargo build --release
# macOS dylib rpath → ../../target/release
# Hz is the body's. Grasp is not hypha is not tesseract.
CC ?= cc
CFLAGS ?= -O2 -Wall
UNAME := $(shell uname)
INCDIR := ../../include
LIBDIR := ../../target/release
BINS := hold tissue hypha tesseract ocean peek

ifeq ($(UNAME),Darwin)
  LIB := $(LIBDIR)/libztp_runtime.dylib
  RPATH := -Wl,-rpath,@loader_path/$(LIBDIR)
else
  LIB := $(LIBDIR)/libztp_runtime.so
  RPATH := -Wl,-rpath,'$$ORIGIN/$(LIBDIR)'
endif

.PHONY: all check clean
all: $(BINS)

hold: hold.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o hold hold.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)

tissue: tissue.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o tissue tissue.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)

hypha: hypha.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o hypha hypha.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)

tesseract: tesseract.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o tesseract tesseract.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH) -lm

ocean: ocean.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o ocean ocean.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)

peek: peek.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o peek peek.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)

check: $(BINS)
	mkdir -p ../../soma
	./hold
	./tissue
	./hypha
	./tesseract
	./ocean
	./peek

clean:
	rm -f $(BINS)
