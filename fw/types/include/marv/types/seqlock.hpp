#pragma once

// A single-writer seqlock (quad spec: cores and contexts exchange data through seqlocks in C11 atomics; decision 0053 Q4, the
// tick's telemetry snapshot). The writer never waits: write is a fixed sequence of stores with no loop and no lock, so an
// interrupt that writes while the code it interrupted is reading completes, and that read sees the change and is retried.
// A reader never returns a torn value: it copies the words between two reads of the sequence number and accepts the copy
// only if the number was even and unchanged.
//
// The value lives in std::atomic<std::uint32_t> words read and written relaxed, ordered by the sequence number's release
// and acquire and by fences (H.-J. Boehm, "Can seqlocks get along with programming language memory models?", MSPC 2012,
// section 3, the fence-based reader): no plain data is shared, so the two sides never race in the C++ memory model.
// One writer only; any number of readers. No allocation.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace marv {

template <class T>
class Seqlock {
  static_assert(std::is_trivially_copyable_v<T>, "a seqlock carries a trivially copyable value");
  using Word = std::uint32_t;
  static constexpr std::size_t kWords = (sizeof(T) + sizeof(Word) - 1) / sizeof(Word);

 public:
  enum class Read : std::uint8_t {
    Ok,     // `out` holds a published value, not torn
    Empty,  // nothing was published yet; `out` unchanged
    Torn,   // a write overlapped the copy; `out` unchanged
  };

  // Back to "never written". Only while no writer and no reader runs (an init).
  void reset() noexcept { seq_.store(0, std::memory_order_relaxed); }

  // The one writer. Never waits.
  void write(const T& value) noexcept {
    std::array<Word, kWords> w{};
    std::memcpy(w.data(), &value, sizeof(T));
    const Word s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (std::size_t i = 0; i < kWords; ++i) {
      words_[i].store(w[i], std::memory_order_relaxed);
    }
    const Word next = s + 2;
    seq_.store(next == 0 ? 2 : next, std::memory_order_release);  // 0 stays "never written" when the number wraps
  }

  // One attempt. `between` runs after the copy and before the check (a test puts a write there, as an interrupt would).
  template <class Between>
  [[nodiscard]] Read try_read(T& out, Between&& between) const noexcept {
    const Word s = seq_.load(std::memory_order_acquire);
    if (s == 0) {
      return Read::Empty;
    }
    if ((s & 1U) != 0) {
      return Read::Torn;
    }
    std::array<Word, kWords> w{};
    for (std::size_t i = 0; i < kWords; ++i) {
      w[i] = words_[i].load(std::memory_order_relaxed);
    }
    between();
    std::atomic_thread_fence(std::memory_order_acquire);
    if (seq_.load(std::memory_order_relaxed) != s) {
      return Read::Torn;
    }
    std::memcpy(static_cast<void*>(&out), w.data(), sizeof(T));  // T is trivially copyable (asserted above)
    return Read::Ok;
  }

  [[nodiscard]] Read try_read(T& out) const noexcept {
    return try_read(out, [] {});
  }

  // Retries a torn read until one is whole: true with `out` the latest value, false if nothing was published yet. A reader
  // preempted by the writer retries; the writer is never held up by it.
  [[nodiscard]] bool read(T& out) const noexcept {
    for (;;) {
      switch (try_read(out)) {
        case Read::Ok:
          return true;
        case Read::Empty:
          return false;
        case Read::Torn:
          break;
      }
    }
  }

 private:
  std::atomic<Word> seq_{0};  // odd while a write is in progress; 0 only until the first write
  std::array<std::atomic<Word>, kWords> words_{};
};

}  // namespace marv
