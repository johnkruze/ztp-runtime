# from examples/c, after: cd ../.. && cargo build --release
# macOS dylib rpath → ../../target/release
CC ?= cc
CFLAGS ?= -O2 -Wall
UNAME := $(shell uname)
INCDIR := ../../include
LIBDIR := ../../target/release

ifeq ($(UNAME),Darwin)
  LIB := $(LIBDIR)/libztp_runtime.dylib
  RPATH := -Wl,-rpath,@loader_path/$(LIBDIR)
else
  LIB := $(LIBDIR)/libztp_runtime.so
  RPATH := -Wl,-rpath,'$$ORIGIN/$(LIBDIR)'
endif

.PHONY: all check clean
all: hold
hold: hold.c $(INCDIR)/ztp.h
	$(CC) $(CFLAGS) -o hold hold.c -I$(INCDIR) -L$(LIBDIR) -lztp_runtime $(RPATH)
check: hold
	./hold
clean:
	rm -f hold
