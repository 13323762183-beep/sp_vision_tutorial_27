#include "pipeline.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <opencv2/imgcodecs.hpp>

namespace
{
    std::mutex output_mutex;

    void logLine(std::ostream &output, const std::string &message)
    {
        std::lock_guard<std::mutex> lock(output_mutex);
        output << message << '\n';
    }
}

Pipeline::Pipeline(std::unique_ptr<FrameSource> source, PipelineConfig config)
    : source_(std::move(source)), config_(std::move(config))
{
    if (!source_)
    {
        throw std::invalid_argument("Pipeline requires a frame source");
    }
    if (config_.worker_count < 2)
    {
        throw std::invalid_argument("worker_count must be at least 2");
    }
}

Pipeline::~Pipeline()
{
    // Destruction without an explicit wait() must still join all threads.
    joinThreads();
}

void Pipeline::start()
{
    if (started_)
    {
        throw std::logic_error("Pipeline::start() may only be called once");
    }
    started_ = true;
    try
    {
        std::filesystem::create_directories(config_.output_directory);
        workers_.reserve(static_cast<std::size_t>(config_.worker_count));
        for (int i = 0; i < config_.worker_count; ++i)
        {
            workers_.emplace_back([this, i]
                                  { workerLoop(i); });
        }
        producer_ = std::thread([this]
                                { producerLoop(); });
    }
    catch (...)
    {
        // Worker threads may already be waiting for a queue item.
        queue_.close();
        joinThreads();
        throw;
    }
}

void Pipeline::joinThreads() noexcept
{
    if (producer_.joinable())
    {
        producer_.join();
    }
    // All pending queue elements remain available to workers after close().
    queue_.close();
    for (auto &worker : workers_)
    {
        if (worker.joinable())
        {
            worker.join();
        }
    }
}

void Pipeline::wait()
{
    joinThreads();
    std::exception_ptr error;
    {
        std::lock_guard<std::mutex> lock(error_mutex_);
        error = first_error_;
    }
    if (error)
    {
        std::rethrow_exception(error);
    }
}

void Pipeline::recordError(std::exception_ptr error) noexcept
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    if (!first_error_)
    {
        first_error_ = error;
    }
    queue_.close();
}

StatisticsSnapshot Pipeline::statistics() const
{
    return statistics_.snapshot();
}

void Pipeline::producerLoop()
{
    try
    {
        Frame frame;
        while (source_->next(frame))
        {
            statistics_.onProduced();
            logLine(std::cout, "[Producer] frame " + std::to_string(frame.id));
            queue_.push(std::move(frame));
            frame = Frame{};
        }
    }
    catch (...)
    {
        recordError(std::current_exception());
    }
    queue_.close();
}

void Pipeline::workerLoop(int worker_id)
{
    try
    {
        Frame frame;
        while (queue_.pop(frame))
        {
            if (config_.worker_delay_ms > 0)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(config_.worker_delay_ms));
            }

            if (checksum(frame.image) != frame.expected_checksum)
            {
                statistics_.onCorrupted();
                logLine(std::cerr,
                        "[Worker " + std::to_string(worker_id) + "] ERROR: frame " +
                            std::to_string(frame.id) + " data changed before processing");
                continue;
            }

            logLine(std::cout,
                    "[Worker " + std::to_string(worker_id) + "] processing frame " +
                        std::to_string(frame.id));
            const cv::Mat output = processor_.process(frame);
            statistics_.onProcessed();

            std::ostringstream filename;
            filename << std::setw(3) << std::setfill('0') << frame.id << ".jpg";
            const auto name = filename.str();
            if (cv::imwrite((config_.output_directory / name).string(), output))
            {
                statistics_.onSaved();
            }
        }
    }
    catch (...)
    {
        recordError(std::current_exception());
    }
}
