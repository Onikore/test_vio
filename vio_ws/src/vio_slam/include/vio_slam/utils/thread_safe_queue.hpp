#ifndef VIO_SLAM_THREAD_SAFE_QUEUE_HPP
#define VIO_SLAM_THREAD_SAFE_QUEUE_HPP

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <optional>

namespace vio_slam {
namespace utils {

/**
 * @brief Thread-safe queue for inter-thread communication
 * 
 * Supports single-producer, single-consumer pattern with optional blocking
 */
template<typename T>
class ThreadSafeQueue {
public:
    using value_type = T;
    
    explicit ThreadSafeQueue(size_t max_size = 100) : max_size_(max_size) {}
    
    /**
     * @brief Push element to queue (blocks if full)
     */
    void push(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [this]() { return queue_.size() < max_size_; });
        queue_.push(std::move(item));
        lock.unlock();
        not_empty_.notify_one();
    }
    
    /**
     * @brief Try push without blocking
     * @return true if successful, false if queue is full
     */
    bool tryPush(T item) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            return false;
        }
        queue_.push(std::move(item));
        lock.unlock();
        not_empty_.notify_one();
        return true;
    }
    
    /**
     * @brief Pop element from queue (blocks if empty)
     */
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [this]() { return !queue_.empty(); });
        T item = std::move(queue_.front());
        queue_.pop();
        lock.unlock();
        not_full_.notify_one();
        return item;
    }
    
    /**
     * @brief Try pop without blocking
     * @return Optional containing value if available, empty otherwise
     */
    std::optional<T> tryPop() {
        std::unique_lock<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        T item = std::move(queue_.front());
        queue_.pop();
        lock.unlock();
        not_full_.notify_one();
        return item;
    }
    
    /**
     * @brief Check if queue is empty
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    /**
     * @brief Get current size
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    /**
     * @brief Clear the queue
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::queue<T> queue_;
    size_t max_size_;
};

} // namespace utils
} // namespace vio_slam

#endif // VIO_SLAM_THREAD_SAFE_QUEUE_HPP
