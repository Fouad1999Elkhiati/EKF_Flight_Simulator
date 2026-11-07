#include "sim_bridge.h"

#include <cstdio>
#include <cstring>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

static void init_header(SharedBlock* d)
{
  std::memset(d, 0, sizeof(SharedBlock));
  d->camera.width = CAM_WIDTH;
  d->camera.height = CAM_HEIGHT;
  d->camera.channels = CAM_CHANNELS;
  d->camera.bottom_up = 1;
  d->camera.rgb = 1;
  d->header.version = BRIDGE_VERSION;
  d->header.total_size = static_cast<uint32_t>(sizeof(SharedBlock));
  d->header.sim_running = 1;
  d->header.magic = BRIDGE_MAGIC;  // written last: tells the controller the block is ready
}

#ifdef _WIN32

bool SimBridge::create()
{
  const DWORD size = static_cast<DWORD>(sizeof(SharedBlock));
  HANDLE h = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, size, BRIDGE_NAME);
  if (!h) {
    std::fprintf(stderr, "[bridge] CreateFileMapping failed (%lu)\n", GetLastError());
    return false;
  }
  void* p = MapViewOfFile(h, FILE_MAP_ALL_ACCESS, 0, 0, size);
  if (!p) {
    std::fprintf(stderr, "[bridge] MapViewOfFile failed (%lu)\n", GetLastError());
    CloseHandle(h);
    return false;
  }
  mapping_ = h;
  data_ = static_cast<SharedBlock*>(p);
  init_header(data_);
  std::printf("[bridge] shared memory '%s' ready (%u bytes)\n", BRIDGE_NAME, static_cast<unsigned>(size));
  return true;
}

SimBridge::~SimBridge()
{
  if (data_) {
    data_->header.sim_running = 0;
    UnmapViewOfFile(data_);
  }
  if (mapping_) CloseHandle(static_cast<HANDLE>(mapping_));
}

#else

bool SimBridge::create()
{
  const char* name = "/" BRIDGE_NAME;
  fd_ = shm_open(name, O_CREAT | O_RDWR, 0666);
  if (fd_ < 0) {
    std::perror("[bridge] shm_open");
    return false;
  }
  if (ftruncate(fd_, sizeof(SharedBlock)) < 0) {
    std::perror("[bridge] ftruncate");
    return false;
  }
  void* p = mmap(nullptr, sizeof(SharedBlock), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (p == MAP_FAILED) {
    std::perror("[bridge] mmap");
    return false;
  }
  data_ = static_cast<SharedBlock*>(p);
  init_header(data_);
  std::printf("[bridge] shared memory /dev/shm/%s ready (%zu bytes)\n", BRIDGE_NAME, sizeof(SharedBlock));
  return true;
}

SimBridge::~SimBridge()
{
  if (data_) {
    data_->header.sim_running = 0;
    munmap(data_, sizeof(SharedBlock));
  }
  if (fd_ >= 0) close(fd_);
}

#endif
