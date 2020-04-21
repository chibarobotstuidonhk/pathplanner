include ../Makefile_linux.inc

TRAJ = traj   $(SNOPT_WRAPPER)

TRAJ_O = $(TRAJ:%=$(EXAMPLESDIR)/%.o)

traj: $(TRAJ_O) $(PSOPT_LIBS) $(DMATRIX_LIBS) $(SPARSE_LIBS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -L$(LIBDIR) $(ALL_LIBRARIES) $(LDFLAGS) -lglfw -lGLEW -lGL -I./include
	rm -f *.o

