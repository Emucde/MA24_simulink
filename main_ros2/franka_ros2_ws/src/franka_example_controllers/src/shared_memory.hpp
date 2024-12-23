// shared_memory.hpp
#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <cstddef>
#include "rclcpp/rclcpp.hpp"

int open_read_shm(const char *name, rclcpp::Logger logger);
int open_write_shm(const char *name, size_t size, rclcpp::Logger logger);
int read_shared_memory(int fd, double *data, size_t size, rclcpp::Logger logger);
int read_shared_memory_flag(int fd, int8_t *data, size_t size, rclcpp::Logger logger);
int write_to_shared_memory(int fd, const void *data, size_t size, rclcpp::Logger logger);

#endif // SHARED_MEMORY_HPP