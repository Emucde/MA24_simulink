#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <semaphore.h>        // for sem_post, sem_t
#include <stdint.h>           // for int8_t
#include <cstddef>            // for size_t
#include <cstring>            // for strerror
#include <iostream>           // for operator<<, basic_ostream, endl, cerr
#include <memory>             // for make_unique, unique_ptr
#include <string>             // for string, basic_string, hash, operator==
#include <unordered_map>      // for unordered_map
#include <vector>             // for vector
#include "rclcpp/logger.hpp"  // for Logger
#include "rclcpp/logging.hpp" // for RCLCPP_ERROR

/*
This file defines the SharedMemory class for managing shared memory and semaphores.
It includes methods for opening, reading, writing, and closing shared memory blocks
and semaphores. The class supports both reading and writing operations,
and it can handle multiple shared memory blocks and semaphores.
It also provides a logging mechanism to report errors.
*/
class BaseLogger
{
public:
    virtual ~BaseLogger() = default;
    virtual void print_error(const std::string &message, int error) const = 0;
};

// create typedef for shared memory name and shared memory size
struct SharedMemoryInfo {
    std::string name;
    size_t block_size;
    size_t block_length;
};

class SharedMemory
{
public:
    SharedMemory() : logger(std::make_unique<StdErrLogger>()) {}
    SharedMemory(rclcpp::Logger logger) : logger(std::make_unique<ROSLogger>(logger)) {}
    ~SharedMemory();
    void open_read_shms(const std::vector<SharedMemoryInfo> shm_infos);
    void open_write_shms(const std::vector<SharedMemoryInfo> shm_infos);
    void open_readwrite_shms(const std::vector<SharedMemoryInfo> shm_infos);
    void open_readwrite_sems(const std::vector<std::string> &sem_names);

    void close_shared_memories();
    void close_semaphores();

    int read_double(const std::string &name, double *data, size_t size);
    int read_int8(const std::string &name, int8_t *data, size_t size);
    int write(const std::string &name, const void *data, size_t size, off_t byte_offset);

    int read_double(const std::string &name, double *data)
    {
        return read_double(name, data, shared_memory_block_sizes[name]);
    }
    int read_int8(const std::string &name, int8_t *data)
    {
        return read_int8(name, data, shared_memory_block_sizes[name]);
    }
    int write(const std::string &name, const void *data, off_t byte_offset)
    {
        return write(name, data, shared_memory_block_sizes[name], byte_offset);
    }
    int write(const std::string &name, const void *data)
    {
        return write(name, data, 0);
    }

    void clear_semaphore(const std::string &name)
    {
        sem_t *sem = semaphores[name];
        while (sem_trywait(sem) == 0)
        {
            // Keep decrementing until it fails
        }
    }

    int feedback_write_int8(const std::string &name, int8_t *data);

    // Method to get a shared memory file descriptor

    // Method to get a semaphore
    sem_t *get_semaphore(const std::string &name)
    {
        return semaphores[name];
    }

    // Method to get a shared memory
    char *get_shared_memory(const std::string &name)
    {
        return shared_memory_data[name];
    }

    // Method to post a semaphore
    void post_semaphore(const std::string &name)
    {
        sem_post(semaphores[name]);
    }

    // Method to wait on a semaphore
    void wait_semaphore(const std::string &name)
    {
        sem_wait(semaphores[name]);
    }

    int trywait_semaphore(const std::string &name)
    {
        return sem_trywait(semaphores[name]);
    }

private:
    std::unique_ptr<BaseLogger> logger; // Smart pointer: Memory is automatically created and cleaned up
    std::unordered_map<std::string, int> shared_memory_fds;
    std::unordered_map<std::string, char *> shared_memory_data;
    std::unordered_map<std::string, size_t> shared_memory_block_sizes;
    std::unordered_map<std::string, size_t> shared_memory_block_lengths;
    std::unordered_map<std::string, sem_t *> semaphores;
    std::vector<SharedMemoryInfo> shm_readwrite_infos;
    std::vector<SharedMemoryInfo> shm_read_infos;
    std::vector<SharedMemoryInfo> shm_write_infos;

    void open_read_shm(SharedMemoryInfo shm_info);
    void open_write_shm(SharedMemoryInfo shm_info);
    void open_readwrite_shm(SharedMemoryInfo shm_info);
    void open_readwrite_sem(const std::string &name);

    class StdErrLogger : public BaseLogger
    {
    public:
        void print_error(const std::string &message, int error) const override
        {
            std::cerr << message << ": " << strerror(error) << std::endl;
        }
    };

    class ROSLogger : public BaseLogger
    {
    public:
        explicit ROSLogger(rclcpp::Logger logger) : logger(logger) {}

        void print_error(const std::string &message, int error) const override
        {
            RCLCPP_ERROR(logger, "%s: %s", message.c_str(), strerror(error));
        }

    private:
        rclcpp::Logger logger;
    };
};

#endif // SHARED_MEMORY_HPP