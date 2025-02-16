// shared_memory.hpp
#ifndef SHARED_MEMORY_2_HPP
#define SHARED_MEMORY_2_HPP

#include <cstddef>
#include "rclcpp/rclcpp.hpp"

int open_read_shm(const char *name, rclcpp::Logger logger);
int open_write_shm(const char *name, rclcpp::Logger logger);
int read_shared_memory(int fd, double *data, size_t size, rclcpp::Logger logger);
int read_shared_memory_flag(int fd, int8_t *data, size_t size, rclcpp::Logger logger);
int write_to_shared_memory(int fd, const void *data, size_t size, rclcpp::Logger logger);
sem_t* open_write_sem(const char *name, rclcpp::Logger logger);

struct shm_flags
{
    int8_t start = 0;
    int8_t reset = 0;
    int8_t stop = 0;
    int8_t torques_valid = 0;
};

#endif // SHARED_MEMORY_2_HPP