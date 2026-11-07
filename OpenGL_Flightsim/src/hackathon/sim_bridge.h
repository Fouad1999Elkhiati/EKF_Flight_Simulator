#pragma once
// Named shared-memory block shared with controller.py.
//   Windows : CreateFileMapping, name "flightsim_bridge"   (Python: mmap(-1, size, tagname=...))
//   Linux   : shm_open("/flightsim_bridge") -> /dev/shm/flightsim_bridge

#include "sim_types.h"

class SimBridge
{
 public:
  SimBridge() = default;
  SimBridge(const SimBridge&) = delete;
  SimBridge& operator=(const SimBridge&) = delete;
  ~SimBridge();

  bool create();                 // creates/maps the block and writes the header
  SharedBlock* data() { return data_; }
  bool ok() const { return data_ != nullptr; }

 private:
  SharedBlock* data_ = nullptr;
#ifdef _WIN32
  void* mapping_ = nullptr;      // HANDLE
#else
  int fd_ = -1;
#endif
};
