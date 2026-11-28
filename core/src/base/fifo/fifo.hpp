/**
 * @file fifo.hpp
 * @brief Bounded, thread-safe software FIFO (header-only).
 * @brief 有界、线程安全的软件 FIFO（header-only）。
 *
 * This file is part of RDK Stereo Camera SDK (https://github.com/D-Robotics/RDK-Stereo-Camera-SDK).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef DSC_BASE_FIFO_HPP
#define DSC_BASE_FIFO_HPP

#include <cstddef>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include "types.hpp"

namespace dsc {
namespace base {

/**
 * Bounded FIFO of copyable items, safe for one or more producers and consumers.
 * 存放可拷贝元素的有界 FIFO，可安全用于一个或多个生产者与消费者。
 *
 * The buffer is allocated once, at construction, so pushing never allocates.  Items
 * are stored by value: T must be default-constructible and copy-assignable, one copy
 * is made on push and one on pop.
 * 缓冲区在构造时一次性分配，因此 push 过程不再分配内存。元素按值存储：
 * T 必须可默认构造且可拷贝赋值；push 时做一次拷贝，pop 时再做一次。
 *
 * Validity: the SDK deliberately requires a depth of at least 2. A shallower FIFO
 * is rejected silently and remains inert: operator bool() is false, push() fails,
 * and the accessors report the empty state.
 * 有效性：SDK 有意要求深度至少为 2。更浅的 FIFO 会被静默拒绝并保持惰性状态：
 * operator bool() 为 false，push() 失败，各访问器报告空状态。
 *
 * Thread safety: push(), pop(), size(), empty(), and full() serialize on the
 * per-object mutex. Individual calls are atomic, but sequences of calls are not.
 * Moving or destroying the object while
 * another thread uses it is not safe.
 * 线程安全：push()、pop()、size()、empty() 和 full() 在对象级互斥锁上串行化。
 * 单次调用是原子的，但多次调用的序列不是。在另一个线程使用该对象时移动或销毁它是不安全的。
 *
 * Ownership: the FIFO owns the buffered copies.  The optional Disposer hook is
 * called with a mutable reference to an item this FIFO discards -- on overwrite in
 * FifoMode::DropOld and for every item still queued at destruction -- which lets the
 * owner release a resource the item refers to.  The hook runs while the FIFO mutex
 * is held, so it must not call back into this FIFO; the caller must have joined the
 * producer and consumer threads before the destructor runs.
 * 所有权：FIFO 持有缓冲区中的拷贝。可选的 Disposer 钩子会以本 FIFO 丢弃的元素的可变引用
 * 被调用——在 FifoMode::DropOld 模式下发生覆盖时，以及析构时仍在排队的每个元素——
 * 从而让持有者释放该元素所引用的资源。该钩子在持有 FIFO 互斥锁期间运行，
 * 因此不得回调本 FIFO；调用方必须在析构函数运行前 join 生产者与消费者线程。
 */
template <typename T>
class Fifo {
public:
    // Optional hook to release an item the FIFO drops or still holds when it dies
    // 可选钩子：释放 FIFO 丢弃的元素，或它在销毁时仍持有的元素。
    using Disposer = void (*)(T &);

    /**
     * @param depth    Buffer capacity; must be at least 2, see the class note on validity.
     * @param depth    缓冲区容量；必须至少为 2，参见类注释中关于有效性的说明。
     * @param mode     Policy applied once the buffer is full.
     * @param mode     缓冲区满后采用的策略。
     * @param disposer Optional drop hook, or nullptr to dispose of queued items as plain copies.
     * @param disposer 可选的丢弃钩子；为 nullptr 时按普通拷贝的方式丢弃排队元素。
     */
    Fifo(std::size_t depth, FifoMode mode, Disposer disposer = nullptr): 
        mode_(mode), buf_(depth), valid_(depth >= 2),
        mtx_(std::make_unique<std::mutex>()), disposer_(disposer)
    {    
    }

    // Disposes of the queued items through the hook; not safe to run concurrently with
    // any other member (the user must have joined producer and consumer threads first).
    // 通过该钩子丢弃排队中的元素；不得与任何其他成员函数并发执行
    // （使用者必须已先 join 生产者与消费者线程）。
    ~Fifo()
    {
        if(!valid_ || disposer_ == nullptr)return;
        // the producer and the consumer are joined before the FIFO is destroyed
        // 生产者与消费者在 FIFO 被销毁前已 join。
        for(std::size_t i = 0; i < count_; i++)disposer_(buf_[(tail_ + i) % buf_.size()]);
    }

    // copying disabled
    // 拷贝被禁用。
    Fifo(const Fifo &) = delete;
    Fifo &operator=(const Fifo &) = delete;

    // define move behavior
    // 定义移动行为。
    // Not thread-safe: the moved-from object ends up inert (empty buffer, false
    // operator bool()), and the items it handed over are disposed of by the new owner.
    // 非线程安全：被移动后的对象最终处于惰性状态（空缓冲区、operator bool() 为 false），
    // 它交出的元素由新的持有者负责丢弃。
    Fifo(Fifo &&other) noexcept: 
        mode_(other.mode_), buf_(std::move(other.buf_)),
        head_(other.head_), tail_(other.tail_), count_(other.count_),
        valid_(other.valid_), mtx_(std::move(other.mtx_)), disposer_(other.disposer_)
    {
        other.valid_ = false;
    }
    // Move-assignment overwrites this object's contents without running the disposer on
    // them; the source is left inert, exactly as after move construction.
    // 移动赋值会覆盖本对象的内容，但不会对它们运行 disposer；
    // 源对象被置为惰性状态，与移动构造之后完全相同。
    Fifo &operator=(Fifo &&other) noexcept
    {
        if(this != &other){
            mode_  = other.mode_;
            buf_   = std::move(other.buf_);
            head_  = other.head_;
            tail_  = other.tail_;
            count_ = other.count_;
            valid_ = other.valid_;
            mtx_   = std::move(other.mtx_);
            disposer_ = other.disposer_;
            other.valid_ = false;
        }
        return *this;
    }

    /** @return true when the object is usable (depth >= 2). */
    /** @return 当对象可用（深度 >= 2）时为 true。 */
    explicit operator bool() const {return valid_;}

    /**
     * Append a copy of @p item.
     * 追加 @p item 的一个拷贝。
     *
     * When the queue is full the configured FifoMode decides: DropNew refuses the item
     * and returns false without touching the queued contents, DropOld overwrites the
     * oldest item (after handing it to the disposer) and still reports success.
     * 队列已满时由配置的 FifoMode 决定：DropNew 拒绝该元素并在不触碰队列内容的情况下
     * 返回 false；DropOld 覆盖最旧的元素（先把它交给 disposer），且仍报告成功。
     *
     * @return true when the item was enqueued, false when it was refused or the object is inert.
     * @return 元素入队时返回 true；元素被拒绝或对象处于惰性状态时返回 false。
     */
    bool push(const T &item)
    {
        if(!valid_) return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(full_unlocked()){
            if(mode_ == FifoMode::DropNew)return false;
            if(disposer_)disposer_(buf_[tail_]);   // release the item being overwritten 释放被覆盖的元素
            tail_ = (tail_ + 1) % buf_.size();   // overwrite the oldest 覆盖最旧的元素。
        }
        else ++count_;

        buf_[head_] = item;
        head_ = (head_ + 1) % buf_.size();
        return true;
    }

    /**
     * Remove the oldest item and copy it into @p item.
     * 移除最旧的元素并把它拷贝到 @p item。
     * @param[out] item Left untouched when nothing is popped.
     * @param[out] item 没有元素出队时保持不变。
     * @return true when an item was returned, false when the queue was empty or the object is inert.
     * @return 返回了元素时为 true；队列为空或对象处于惰性状态时为 false。
     */
    bool pop(T &item)
    {
        if(!valid_)return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(count_ == 0)
            return false;
        item = buf_[tail_];
        tail_ = (tail_ + 1) % buf_.size();
        --count_;
        return true;
    }

    /** @return Number of queued items; 0 for an inert object. */
    /** @return 排队元素的数量；惰性对象返回 0。 */
    std::size_t size() const
    {
        if(!valid_)return 0;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_;
    }

    /** @return true when nothing is queued, which is also the answer for an inert object. */
    /** @return 没有元素排队时为 true，惰性对象也返回该值。 */
    bool empty() const
    {
        if(!valid_)return true;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_ == 0;
    }

    /**
     * @return true when the queue holds capacity() items, i.e. the next push() applies the
     *         full policy; false for an inert object.
     * @return 队列已装有 capacity() 个元素，即下一次 push() 会应用满策略时为 true；
     *         惰性对象返回 false。
     */
    bool full() const
    {
        if(!valid_)return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        return full_unlocked();
    }

    /** @return Configured depth, i.e. the constructor argument; lock-free and constant. */
    /** @return 配置的深度，即构造函数参数；无锁且恒定。 */
    std::size_t capacity() const {return buf_.size();}

private:
    bool full_unlocked() const {return count_ == buf_.size();}

    FifoMode mode_;
    std::vector<T> buf_;
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t count_ = 0;
    bool valid_;
    std::unique_ptr<std::mutex> mtx_;
    Disposer disposer_;
};

} // namespace base
} // namespace dsc

#endif // DSC_BASE_FIFO_HPP
