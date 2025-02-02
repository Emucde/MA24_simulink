#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <semaphore.h>         // for sem_post, sem_t
#include <stdint.h>            // for int8_t
#include <cstddef>             // for size_t
#include <cstring>             // for strerror
#include <iostream>            // for operator<<, basic_ostream, endl, cerr
#include <memory>              // for make_unique, unique_ptr
#include <string>              // for string, basic_string, hash, operator==
#include <unordered_map>       // for unordered_map
#include <vector>              // for vector
#include "rclcpp/logger.hpp"   // for Logger
#include "rclcpp/logging.hpp"  // for RCLCPP_ERROR

class BaseLogger
{
public:
    virtual ~BaseLogger() = default;
    virtual void print_error(const std::string &message, int error) const = 0;
};

class SharedMemory
{
public:
    SharedMemory() : logger(std::make_unique<StdErrLogger>()) {}
    SharedMemory(rclcpp::Logger logger) : logger(std::make_unique<ROSLogger>(logger)) {}
    ~SharedMemory();
    void open_read_shms(const std::vector<std::string> &shm_names);
    void open_write_shms(const std::vector<std::string> &shm_names);
    void open_readwrite_shms(const std::vector<std::string> &shm_names);
    void open_readwrite_sems(const std::vector<std::string> &sem_names);

    void close_shared_memories();
    void close_semaphores();

    int read_double(int fd, double *data, size_t size);
    int read_double(const std::string &name, double *data, size_t size);

    int read_int8(int fd, int8_t *data, size_t size);
    int read_int8(const std::string &name, int8_t *data, size_t size);

    int write(int fd, const void *data, size_t size);
    int write(const std::string &name, const void *data, size_t size);

    // Method to get a shared memory file descriptor
    int get_shared_memory_fd(const std::string &name)
    {
        return shared_memories[name];
    }

    // Method to get a semaphore
    sem_t *get_semaphore(const std::string &name)
    {
        return semaphores[name];
    }

    // Method to post a semaphore
    void post_semaphore(const std::string &name)
    {
        sem_post(semaphores[name]);
    }

private:
    std::unique_ptr<BaseLogger> logger; // Smart pointer: Memory is automatically created and cleaned up
    std::unordered_map<std::string, int> shared_memories;
    std::unordered_map<std::string, sem_t *> semaphores;

    void open_read_shm(const std::string& name);
    void open_write_shm(const std::string& name);
    void open_readwrite_shm(const std::string& name);
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