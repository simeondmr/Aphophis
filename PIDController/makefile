ifdef OS
#Windows
EXEEXT  :=.exe
else
#Unix
EXEEXT  :=
endif
MKDIR   :=mkdir
RMDIR   :=rm -rf
CC      :=g++
BIN     :=bin
OBJ     :=obj
DEP     :=dep
INCLUDE :=include
SRC     :=src
GEN     :=gen
OBJGEN  :=$(OBJ)/$(GEN)
DEPGEN  :=$(DEP)/$(GEN)
SRCGEN  :=$(SRC)/$(GEN)
DEPFLAGS:= -MT $@ -MMD -MP -MF $(OBJ)/$*.d
SRCS    :=$(shell bash -c "/usr/bin/find $(SRC)/ -type f -name '*.cpp'")
OBJS    :=$(addprefix $(OBJ)/, $(SRCS:.cpp=.o))
DEPS    :=$(addprefix $(DEP)/, $(SRCS:.cpp=.d))
EXE     :=$(BIN)/a$(EXEEXT)
LOG     :=log.txt
CFLAGS  :=-Wall -Wno-comment -Wno-sign-compare -lpthread -g -std=c++17 -Wno-narrowing

.PHONY: test build debug debuglog run log err tee cleanobj cleanbin cleandep cleangen cleannotbin clean all alldebug alldebuglog allrun alllog allerr alltee

test:
	@echo $(SRCS)
	@echo
	@echo $(OBJS)
	@echo
	@echo $(DEPS)
	@echo

build: $(EXE)

$(EXE): $(OBJS) | $(BIN)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(OBJS): $(OBJ)/%.o: %.cpp | $(OBJ) $(DEP) $(SRCGEN) $(OBJGEN) $(DEPGEN)
	@echo $(DEP)/$*.d
	@echo $(@D)
	$(MKDIR) -p $(dir $(DEP)/$*.d)
	$(MKDIR) -p $(@D)
	$(CC) $(CFLAGS) -MT $@ -MMD -MP -MF $(DEP)/$*.d -c $< -o $@

$(BIN) $(OBJ) $(DEP) $(SRCGEN) $(OBJGEN) $(DEPGEN):
	$(MKDIR) $@

debug: $(EXE)
	gdb -ex r -ex bt -batch $<

debuglog: $(EXE)
	gdb -ex r -ex bt -batch $< 2>&1 > $(LOG)

run: $(EXE)
	$<

log: $(EXE)
	$< 2>&1 > $(LOG)

err: $(EXE)
	$< 2> $(LOG)

tee: $(EXE)
	$< 2>&1 | tee $(LOG)

cleangen:
	$(RMDIR) $(SRCGEN)

cleanobj: cleangen
	$(RMDIR) $(OBJ)

cleanbin:
	$(RMDIR) $(BIN)
	$(RMDIR) a$(EXEEXT).stackdump

cleandep:
	$(RMDIR) $(DEP)

cleannotbin: cleanobj cleandep

clean: cleannotbin cleanbin

all: clean build cleanobj

alldebug: clean build cleanobj debug

alldebuglog: clean build cleanobj debuglog

allrun: clean build cleanobj run

alllog: clean build cleanobj log

allerr: clean build cleanobj err

alltee: clean build cleanobj tee

include $(wildcard $(DEPS))