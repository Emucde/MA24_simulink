// shared_memory.hpp
#ifndef SHARED_MEMORY_HPP
#define SHARED_MEMORY_HPP

#include <cstddef>

double *read_shared_memory(const char *name, size_t size);
void print_shared_memory_data(const char *name, size_t size);
void write_to_shared_memory(const char *name, const void *data, size_t size);

#endif // SHARED_MEMORY_HPP