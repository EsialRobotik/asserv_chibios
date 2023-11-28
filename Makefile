# Check if the specified ROBOT exist
AVAILABLE_ROBOTS=$(shell find src/Robots -mindepth 1 -maxdepth 1 -type d -exec basename {} \;)
ifeq (,$(wildcard src/Robots/$(ROBOT)/main.cpp))
$(error Unknown ROBOT specified! Knowns are : $(AVAILABLE_ROBOTS))
endif

MAKE_PID := $(shell echo $$PPID)
JOBS := $(shell ps T | sed -n 's%.*$(MAKE_PID).*$(MAKE).* \(-j\|--jobs=\) *\([0-9][0-9]*\).*%\2%p')

ifeq ($(JOBS),)
JOBS := 1
endif

%:
	cd src/Robots/$(ROBOT) && make $@ -j $(JOBS)