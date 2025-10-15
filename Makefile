# Makefile for juicy_physical_synth~ (Pd external)
# Targets:
#   make mac_x86     -> x86_64 (Intel) .pd_darwin  [use this for PlugData needing x86_64]
#   make mac_arm     -> arm64 (Apple Silicon) .pd_darwin
#   make mac_universal -> fat (arm64 + x86_64)
#   make linux       -> .pd_linux
#
# Override PD_SRC_MAC/PD_SRC_LIN if your Pd headers live elsewhere:
#   make mac_x86 PD_SRC_MAC=/path/to/Pd.app/Contents/Resources/src

PD_SRC_MAC=/Applications/Pd-0.56-1.app/Contents/Resources/src
PD_SRC_LIN=/usr/include/pd

SRC=juicy_physical_synth_tilde.c

mac_x86:
\tcc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \\
\t\t-I"$(PD_SRC_MAC)" -arch x86_64 -mmacosx-version-min=10.13 \\
\t\t-bundle -undefined dynamic_lookup \\
\t\t-o juicy_physical_synth~.pd_darwin $(SRC)

mac_arm:
\tcc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \\
\t\t-I"$(PD_SRC_MAC)" -arch arm64 -mmacosx-version-min=10.13 \\
\t\t-bundle -undefined dynamic_lookup \\
\t\t-o juicy_physical_synth~.pd_darwin $(SRC)

mac_universal:
\tcc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \\
\t\t-I"$(PD_SRC_MAC)" -arch arm64 -arch x86_64 -mmacosx-version-min=10.13 \\
\t\t-bundle -undefined dynamic_lookup \\
\t\t-o juicy_physical_synth~.pd_darwin $(SRC)

linux:
\tcc -O3 -fPIC -DPD -Wall -Wextra -Wno-unused-parameter -Wno-cast-function-type \\
\t\t-I"$(PD_SRC_LIN)" -shared -fPIC -Wl,-export-dynamic -lm \\
\t\t-o juicy_physical_synth~.pd_linux $(SRC)

clean:
\trm -f juicy_physical_synth~.pd_darwin juicy_physical_synth~.pd_linux
