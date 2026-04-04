#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstddef>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace mcl {

/**
 * Background thread drains a queue of strings to a FILE* (stdout / serial) at a max bytes/sec rate.
 * Enqueue is non-blocking for the producer (short mutex hold). No filesystem I/O.
 */
class ThrottledWriter {
public:
    explicit ThrottledWriter(
        size_t max_bytes_per_sec,
        FILE* stream = stdout,
        size_t max_queue_depth = 200)
        : max_bytes_per_sec_(max_bytes_per_sec),
          stream_(stream),
          max_queue_depth_(max_queue_depth),
          worker_([this] { run(); }) {}

    ThrottledWriter(const ThrottledWriter&) = delete;
    ThrottledWriter& operator=(const ThrottledWriter&) = delete;

    ~ThrottledWriter() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    void enqueue(std::string msg) {
        if (msg.empty()) return;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (max_queue_depth_ > 0 && queue_.size() >= max_queue_depth_) {
                queue_.pop_front();
            }
            queue_.push_back(std::move(msg));
        }
        cv_.notify_one();
    }

    void set_max_bytes_per_sec(size_t bytes) {
        max_bytes_per_sec_.store(bytes, std::memory_order_relaxed);
    }

    size_t max_bytes_per_sec() const {
        return max_bytes_per_sec_.load(std::memory_order_relaxed);
    }

private:
    void run() {
        for (;;) {
            std::string msg;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
                if (stop_ && queue_.empty()) {
                    return;
                }
                if (!queue_.empty()) {
                    msg = std::move(queue_.front());
                    queue_.pop_front();
                }
            }
            if (msg.empty()) {
                continue;
            }
            const size_t n = msg.size();
            if (stream_) {
                std::fwrite(msg.data(), 1, n, stream_);
                std::fflush(stream_);
            }
            const size_t budget = max_bytes_per_sec_.load(std::memory_order_relaxed);
            if (budget > 0 && n > 0) {
                const double sec = static_cast<double>(n) / static_cast<double>(budget);
                const auto delay = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::duration<double>(sec));
                if (delay.count() > 0) {
                    std::this_thread::sleep_for(delay);
                }
            }
        }
    }

    std::atomic<size_t> max_bytes_per_sec_;
    FILE* stream_;
    size_t max_queue_depth_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::string> queue_;
    bool stop_ = false;
    std::thread worker_;
};

} // namespace mcl
