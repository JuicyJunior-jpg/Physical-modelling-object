# Makefile for juicy_physical_synth~ (Pd external)
PD_SRC_MAC=/Applications/Pd-0.56-1.app/Contents/Resources/src
PD_SRC_LIN=/usr/include/pd

OBJ=juicy_physical_synth_tilde.c

all: mac linux

mac:
	cc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \
		-I"$(PD_SRC_MAC)" -arch arm64 -arch x86_64 -mmacosx-version-min=10.13 \
		-bundle -undefined dynamic_lookup \
		-o juicy_physical_synth~.pd_darwin $(OBJ)

linux:
	cc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \
		-I"$(PD_SRC_LIN)" -shared -fPIC -Wl,-export-dynamic -lm \
		-o juicy_physical_synth~.pd_linux $(OBJ)

clean:
	rm -f juicy_physical_synth~.pd_darwin juicy_physical_synth~.pd_linux
