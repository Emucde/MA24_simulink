#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h> // Include errno for error handling
#include <semaphore.h>

#include "shared_memory.hpp"

sem_t* open_write_sem(const char *name, rclcpp::Logger logger)
{
    sem_t *sem = sem_open(name, O_RDWR, 0666, 0);
    if (sem == SEM_FAILED)
    {
        RCLCPP_ERROR(logger, "write: open: Error opening semaphore: %s", strerror(errno));
        throw std::runtime_error("Error allocating semaphore");
        return nullptr;
    }
    return sem;
}

int open_read_shm(const char *name, rclcpp::Logger logger)
{
    int fd = shm_open(name, O_RDONLY, 0666);
    if (fd == -1)
    {
        RCLCPP_ERROR(logger, "read: open: Error opening shared memory: %s", strerror(errno));
        throw std::runtime_error("Error allocating shared memory");
        return -1;
    }
    return fd;
}

int open_write_shm(const char *name, rclcpp::Logger logger)
{
    int fd = shm_open(name, O_RDWR, 0666);
    if (fd == -1)
    {
        RCLCPP_ERROR(logger, "write: open: Error opening shared memory: %s", strerror(errno));
        throw std::runtime_error("Error allocating shared memory");
        return -1;
    }

    return fd;
}

// Existing read function
int read_shared_memory(int fd, double *data, size_t size, rclcpp::Logger logger)
{
    double *shared_data = (double *)mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        RCLCPP_ERROR(logger, "read: Error mapping shared memory: %s", strerror(errno));
        return -1;
    }

    memcpy(data, shared_data, size);

    if (munmap(shared_data, size) == -1)
    {
        RCLCPP_ERROR(logger, "read: Error unmapping shared memory: %s", strerror(errno));
        return -1;
    }

    return 0;
}

int read_shared_memory_flag(int fd, int8_t *data, size_t size, rclcpp::Logger logger)
{
    int8_t *shared_data = (int8_t *)mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        RCLCPP_ERROR(logger, "readf: Error mapping shared memory: %s", strerror(errno));
        return -1;
    }

    memcpy(data, shared_data, size);

    //Clean up
    if (munmap(shared_data, size) == -1)
    {
        RCLCPP_ERROR(logger, "readf: Error unmapping shared memory: %s", strerror(errno));
        return -1;
    }

    return 0;
}

// Combined write function for shared memory
int write_to_shared_memory(int fd, const void *data, size_t size, rclcpp::Logger logger)
{
    void *shared_data = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        RCLCPP_ERROR(logger, "write: Error mapping shared memory: %s", strerror(errno));
        return -1;
    }

    // Write data to shared memory
    memcpy(shared_data, data, size);

    // Clean up
    if (munmap(shared_data, size) == -1)
    {
        RCLCPP_ERROR(logger, "write: Error unmapping shared memory: %s", strerror(errno));
        return -1;
    }

    return 0;
}