#include "statistics.hpp"

#include <mutex>

void Statistics::onProduced()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++produced_;
}

void Statistics::onProcessed()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++processed_;
}

void Statistics::onSaved()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++saved_;
}

void Statistics::onCorrupted()
{
    std::lock_guard<std::mutex> lock(mutex_);
    ++corrupted_;
}

StatisticsSnapshot Statistics::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return {produced_, processed_, saved_, corrupted_};
}
