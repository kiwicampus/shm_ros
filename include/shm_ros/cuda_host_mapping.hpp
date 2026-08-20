/*! @package shm_ros
    Code Information:
        Maintainer: Advanced Robotics Team
        Copyright robot.com / Advanced Robotics Team

    Maps an existing host memory range (a shm_ros segment mapping) for direct
    GPU access via cudaHostRegister, instead of exporting/importing a CUDA IPC
    memory handle. On Jetson, CPU and GPU share the same physical DRAM, so this
    is enough to get a valid device pointer with no copy -- and unlike
    cudaIpcMemHandle, it works on Jetson's integrated GPU. On a discrete GPU it
    still works, just with every GPU access to the pointer crossing PCIe
    instead of hitting real VRAM.
*/

#ifndef SHM_ROS__CUDA_HOST_MAPPING_HPP_
#define SHM_ROS__CUDA_HOST_MAPPING_HPP_

#include <cuda_runtime_api.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace shm_ros {

class CudaHostMapping {
 public:
  CudaHostMapping() = default;
  ~CudaHostMapping() { unmap(); }
  CudaHostMapping(const CudaHostMapping &) = delete;
  CudaHostMapping &operator=(const CudaHostMapping &) = delete;

  // Maps [host_ptr, host_ptr + bytes). Safe to call again (e.g. after a
  // segment resize/recreate) -- unmaps any prior mapping first.
  void map(void *host_ptr, size_t bytes) {
    unmap();
    const cudaError_t err = cudaHostRegister(host_ptr, bytes, cudaHostRegisterMapped);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaHostRegister failed: " + std::string(cudaGetErrorString(err)));
    }
    host_ptr_ = host_ptr;
    mapped_ = true;
  }

  bool is_mapped() const { return mapped_; }

  // The host pointer last passed to map(), or nullptr. Lets a caller detect a
  // segment remap (new address) and know to call map() again.
  void *host_ptr() const { return host_ptr_; }

  // Device pointer for host_ptr() + offset, valid in this process's current
  // CUDA context. Must call map() first.
  void *device_ptr(size_t offset) const {
    if (!mapped_) {
      throw std::runtime_error("device_ptr() called before map()");
    }
    void *dev_ptr = nullptr;
    const cudaError_t err = cudaHostGetDevicePointer(&dev_ptr, host_ptr_, 0);
    if (err != cudaSuccess) {
      throw std::runtime_error("cudaHostGetDevicePointer failed: " + std::string(cudaGetErrorString(err)));
    }
    return static_cast<uint8_t *>(dev_ptr) + offset;
  }

  void unmap() {
    if (mapped_) {
      cudaHostUnregister(host_ptr_);  // best-effort: nothing useful to do if this fails
      mapped_ = false;
      host_ptr_ = nullptr;
    }
  }

 private:
  void *host_ptr_ = nullptr;
  bool mapped_ = false;
};

}  // namespace shm_ros

#endif  // SHM_ROS__CUDA_HOST_MAPPING_HPP_
