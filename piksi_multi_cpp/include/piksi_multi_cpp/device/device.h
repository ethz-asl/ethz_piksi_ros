#ifndef PIKSI_MULTI_CPP_DEVICE_DEVICE_H_
#define PIKSI_MULTI_CPP_DEVICE_DEVICE_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace piksi_multi_cpp {

typedef std::string Identifier;
typedef std::set<Identifier> Identifiers;
inline bool identifierEqual(const Identifier& a, const Identifier& b) {
  return a.compare(b) == 0;
}

class Device {
 public:
  typedef std::shared_ptr<Device> Ptr;

  Device(const Identifier& id);
  bool open();
  int32_t read(uint8_t* buff, uint32_t n);
  int32_t write(std::vector<uint8_t> buff);
  void close();
  inline std::string getID() const { return id_; }

  // This function will be passed to sbp_process.
  // libsbp is a C interface and does not allow binding a non-static member
  // function using std::bind. Thus we bind this static member function that
  // points on the context's read function.
  // WARNING: Requires sbp_state_set_io_context to be called first.
  static int32_t read_redirect(uint8_t* buff, uint32_t n, void* context);
  static int32_t write_redirect(uint8_t* buff, uint32_t n, void* context);

 protected:
  virtual bool openImpl() = 0;
  virtual int32_t readImpl(uint8_t* buff, uint32_t n) const = 0;
  virtual int32_t writeImpl(std::vector<uint8_t> buff) const = 0;
  virtual void closeImpl() = 0;
  Identifier id_;

 private:
  std::mutex mtx_;
};

}  // namespace piksi_multi_cpp

#endif  // PIKSI_MULTI_CPP_DEVICE_DEVICE_H_
