#include <fcntl.h>    // for O_RDWR, O_RDONLY, O_WRONLY
#include <sys/mman.h> // for mmap, munmap, shm_open, MAP_FAILED, MAP_SHARED
#include <unistd.h>   // for close
#include <cerrno>     // for errno
#include <stdexcept>  // for runtime_error
#include "SharedMemory.hpp"

// Destructor to ensure closures
SharedMemory::~SharedMemory()
{
    close_shared_memories();
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PUBLIC METHODS ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////                ||||||   ||   ||  ||||    ||      ||    |||||                /////
/////                ||   ||  ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||||||   ||   ||  ||||||  ||      ||  ||                     /////
/////                ||       ||   ||  ||  ||  ||      ||  ||                     /////
/////                ||        |||||   ||||    ||||||  ||    |||||                /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void SharedMemory::open_read_shms(const std::vector<SharedMemoryInfo> shm_infos)
{
    // typedef std::pair<std::string, size_t> SharedMemoryInfo;
    for (const auto &shm_info : shm_infos)
    {
        open_read_shm(shm_info);
    }
}

void SharedMemory::open_write_shms(const std::vector<SharedMemoryInfo> shm_infos)
{
    for (const auto &shm_info : shm_infos)
    {
        open_write_shm(shm_info);
    }
}

void SharedMemory::open_readwrite_shms(const std::vector<SharedMemoryInfo> shm_infos)
{
    for (const auto &shm_info : shm_infos)
    {
        open_readwrite_shm(shm_info);
    }
}

void SharedMemory::open_readwrite_sems(const std::vector<std::string> &sem_names)
{
    for (const auto &name : sem_names)
    {
        open_readwrite_sem(name);
    }
}

void SharedMemory::close_shared_memories()
{
    // Unmap shared memories
    for (const auto &[name, data] : shared_memory_data)
    {
        if (data != nullptr)
            munmap(data, shared_memory_block_sizes[name]*shared_memory_block_lengths[name]);
    }

    // Close shared memory file descriptors
    for (const auto &[name, fd] : shared_memory_fds)
    {
        if (fd != 0)
        {
            close(fd);
            shared_memory_fds[name] = fd;
        }
    }
}

void SharedMemory::close_semaphores()
{
    for (const auto &[name, semaphore] : semaphores)
    {
        if (semaphore != 0)
            sem_close(semaphore);
    }
}

int SharedMemory::read_double(const std::string &name, double *data, size_t size)
{
    memcpy(data, shared_memory_data[name], size);
    return 0;
}

int SharedMemory::read_int8(const std::string &name, int8_t *data, size_t size)
{
    memcpy(data, shared_memory_data[name], size);
    return 0;
}

int SharedMemory::write(const std::string &name, const void *data, size_t size, off_t byte_offset)
{
    memcpy(shared_memory_data[name] + byte_offset*shared_memory_block_sizes[name], data, size);
    return 0;
}

// getter and setter

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// PRIVATE METHODS ///////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////                                                                             /////
/////          ||||||   ||||||   ||  ||    ||   ||||    ||||||||  ||||||          /////
/////          ||   ||  ||   ||  ||  ||    ||  ||   ||     ||     ||              /////
/////          ||||||   ||||||   ||   ||  ||   |||||||     ||     ||||||          /////
/////          ||       || ||    ||    ||||    ||   ||     ||     ||              /////
/////          ||       ||   ||  ||     ||     ||   ||     ||     ||||||          /////
/////                                                                             /////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void SharedMemory::open_read_shm(SharedMemoryInfo shm_info)
{
    const std::string &name = shm_info.name;
    size_t block_size = shm_info.block_size;
    size_t block_length = shm_info.block_length;
    int fd = shm_open(name.c_str(), O_RDONLY, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_read_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }

    void *shared_data = mmap(nullptr, block_size*block_length, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[write] Error mapping shared memory: ", errno);
        return;
    }

    shared_memory_fds[name] = fd;
    shared_memory_data[name] = static_cast<char *>(shared_data);
    shared_memory_block_sizes[name] = block_size;
    shared_memory_block_lengths[name] = block_length;
}

void SharedMemory::open_write_shm(SharedMemoryInfo shm_info)
{
    const std::string &name = shm_info.name;
    size_t block_size = shm_info.block_size;
    size_t block_length = shm_info.block_length;

    int fd = shm_open(name.c_str(), O_WRONLY, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_write_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }

    void *shared_data = mmap(nullptr, block_size*block_length, PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[write] Error mapping shared memory: ", errno);
        return;
    }

    shared_memory_fds[name] = fd;
    shared_memory_data[name] = static_cast<char *>(shared_data);
    shared_memory_block_sizes[name] = block_size;
    shared_memory_block_lengths[name] = block_length;
}

void SharedMemory::open_readwrite_shm(SharedMemoryInfo shm_info)
{
    const std::string &name = shm_info.name;
    size_t block_size = shm_info.block_size;
    size_t block_length = shm_info.block_length;

    int fd = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_readwrite_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }

    void *shared_data = mmap(nullptr, block_size*block_length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[write] Error mapping shared memory: ", errno);
        return;
    }

    shared_memory_fds[name] = fd;
    shared_memory_data[name] = static_cast<char *>(shared_data);
    shared_memory_block_sizes[name] = block_size;
    shared_memory_block_lengths[name] = block_length;
}

void SharedMemory::open_readwrite_sem(const std::string &name)
{
    sem_t *sem = sem_open(name.c_str(), O_RDWR, 0666, 0);
    if (sem == SEM_FAILED)
    {
        logger->print_error("[open_readwrite_sem] Error opening semaphore", errno);
        throw std::runtime_error("Error allocating semaphore");
    }
    semaphores[name] = sem;
}