#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h> // Include errno for error handling

#include "shared_memory.hpp"

// Existing read function
double *read_shared_memory(const char *name, size_t size)
{
    int fd = shm_open(name, O_RDONLY, 0666);
    if (fd == -1)
    {
        std::cerr << "Error opening shared memory: " << strerror(errno) << std::endl;
        return nullptr;
    }

    double *data = (double *)mmap(0, size, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED)
    {
        std::cerr << "Error mapping shared memory: " << strerror(errno) << std::endl;
        close(fd);
        return nullptr;
    }

    close(fd);
    return data;
}

// Existing print function
void print_shared_memory_data(const char *name, size_t size)
{
    double *data = read_shared_memory(name, size);
    if (data)
    {
        size_t num_elements = size / sizeof(double);
        std::cout << "Data from " << name << ": ";
        for (size_t i = 0; i < num_elements; ++i)
        {
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;
        munmap(data, size);
    }
}

// Combined write function for shared memory
void write_to_shared_memory(const char *name, const void *data, size_t size)
{
    int fd = shm_open(name, O_RDWR | O_CREAT, 0666);
    if (fd == -1)
    {
        std::cerr << "Error opening shared memory for writing: " << strerror(errno) << std::endl;
        return;
    }

    // Allocate the shared memory segment
    if (ftruncate(fd, size) == -1)
    {
        std::cerr << "Error allocating shared memory: " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    void *shared_data = mmap(0, size, PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        std::cerr << "Error mapping shared memory for writing: " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    // Write data to shared memory
    memcpy(shared_data, data, size);

    // Clean up
    munmap(shared_data, size);
    close(fd);
}
