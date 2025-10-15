PD_SRC_MAC ?= /Applications/Pd-0.56-1.app/Contents/Resources/src
PD_SRC_LIN ?= /usr/include/pd

SRC = juicy_physical_synth_tilde.c
OUT_MAC = juicy_physical_synth~.pd_darwin
OUT_LIN = juicy_physical_synth~.pd_linux

mac_x86:
	cc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \
		-I"$(PD_SRC_MAC)" -arch x86_64 -mmacosx-version-min=10.13 \
		-bundle -undefined dynamic_lookup \
		-o $(OUT_MAC) $(SRC)

mac_arm:
	cc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \
		-I"$(PD_SRC_MAC)" -arch arm64 -mmacosx-version-min=10.13 \
		-bundle -undefined dynamic_lookup \
		-o $(OUT_MAC) $(SRC)

linux:
	cc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \
		-I"$(PD_SRC_LIN)" -shared -fPIC -Wl,-export-dynamic -lm \
		-o $(OUT_LIN) $(SRC)

clean:
	rm -f $(OUT_MAC) $(OUT_LIN)
