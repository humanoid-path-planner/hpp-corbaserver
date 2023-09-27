// Copyright (C) 2023 by Joseph Mirabel
//
// This code was taken from https://stackoverflow.com/a/28121513

#include <condition_variable>
#include <mutex>

namespace hpp {
namespace corbaServer {

/// Synchronization class that ensures the following.
/// - A Write block Weads and Writes.
/// - Reads are only blocked by Writes, not by other Reads.
/// - Write is favored over Reads.
///
/// This code was taken from https://stackoverflow.com/a/28121513
class ReadWriteLock {
 public:
  ReadWriteLock()
      : shared_(),
        readerQ_(),
        writerQ_(),
        activeReaders_(0),
        waitingWriters_(0),
        activeWriters_(0) {}

  void readLock() {
    std::unique_lock<std::mutex> lk(shared_);
    while (waitingWriters_ != 0) readerQ_.wait(lk);
    ++activeReaders_;
    lk.unlock();
  }

  void readUnlock() {
    std::unique_lock<std::mutex> lk(shared_);
    --activeReaders_;
    lk.unlock();
    writerQ_.notify_one();
  }

  void writeLock() {
    std::unique_lock<std::mutex> lk(shared_);
    ++waitingWriters_;
    while (activeReaders_ != 0 || activeWriters_ != 0) writerQ_.wait(lk);
    ++activeWriters_;
    lk.unlock();
  }

  void writeUnlock() {
    std::unique_lock<std::mutex> lk(shared_);
    --waitingWriters_;
    --activeWriters_;
    if (waitingWriters_ > 0)
      writerQ_.notify_one();
    else
      readerQ_.notify_all();
    lk.unlock();
  }

 private:
  std::mutex shared_;
  std::condition_variable readerQ_;
  std::condition_variable writerQ_;
  int activeReaders_;
  int waitingWriters_;
  int activeWriters_;
};

}  // end of namespace corbaServer.
}  // end of namespace hpp.
