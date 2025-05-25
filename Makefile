# Check if the specified ROBOT exist
AVAILABLE_ROBOTS=$(shell find src/Robots -mindepth 1 -maxdepth 1 -type d -exec basename {} \;)
ifeq (,$(wildcard src/Robots/$(ROBOT)/main.cpp))
$(error Unknown ROBOT specified! Knowns are : $(AVAILABLE_ROBOTS))
endif

%:
	cd src/Robots/$(ROBOT) && make $@ -j 8