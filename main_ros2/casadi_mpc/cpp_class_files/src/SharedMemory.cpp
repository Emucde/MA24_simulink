#include <fcntl.h>     // for O_RDWR, O_RDONLY, O_WRONLY
#include <sys/mman.h>  // for mmap, munmap, shm_open, MAP_FAILED, MAP_SHARED
#include <unistd.h>    // for close
#include <cerrno>      // for errno
#include <stdexcept>   // for runtime_error
#include "SharedMemory.hpp"

// Destructor to ensure closures
SharedMemory::~SharedMemory() {
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

void SharedMemory::open_read_shms(const std::vector<std::string> &shm_names)
{
    for (const auto &name : shm_names)
    {
        open_read_shm(name);
    }
}

void SharedMemory::open_write_shms(const std::vector<std::string> &shm_names)
{
    for (const auto &name : shm_names)
    {
        open_write_shm(name);
    }
}

void SharedMemory::open_readwrite_shms(const std::vector<std::string> &shm_names)
{
    for (const auto &name : shm_names)
    {
        open_readwrite_shm(name);
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
    for (const auto &[name, fd] : shared_memories)
    {
        if(fd != 0)
            close(fd);
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

int SharedMemory::read_double(int fd, double *data, size_t size)
{
    double *shared_data = (double *)mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[read_double] Error mapping shared memory: ", errno);
        return -1;
    }
    memcpy(data, shared_data, size);

    if (munmap(shared_data, size) == -1)
    {
        logger->print_error("[read_double] Error unmapping shared memory: ", errno);
        return -1;
    }
    return 0;
}

int SharedMemory::read_double(const std::string& name, double *data, size_t size)
{
    int fd = shared_memories[name];
    return read_double(fd, data, size);
    return 0;
}

int SharedMemory::read_int8(int fd, int8_t *data, size_t size)
{
    int8_t *shared_data = (int8_t *)mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[read_int8] Error mapping shared memory: ", errno);
        return -1;
    }
    memcpy(data, shared_data, size);

    if (munmap(shared_data, size) == -1)
    {
        logger->print_error("[read_int8] Error unmapping shared memory: ", errno);
        return -1;
    }
    return 0;
}

int SharedMemory::read_int8(const std::string& name, int8_t *data, size_t size)
{
    int fd = shared_memories[name];
    return read_int8(fd, data, size);
}

int SharedMemory::write(int fd, const void *data, size_t size)
{
    void *shared_data = mmap(nullptr, size, PROT_WRITE, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED)
    {
        logger->print_error("[write] Error mapping shared memory: ", errno);
        return -1;
    }

    memcpy(shared_data, data, size);

    if (munmap(shared_data, size) == -1)
    {
        logger->print_error("[write] Error unmapping shared memory: ", errno);
        return -1;
    }
    return 0;
}


int SharedMemory::write(const std::string& name, const void *data, size_t size)
{
    int fd = shared_memories[name];
    return write(fd, data, size);
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

void SharedMemory::open_read_shm(const std::string& name)
{
    int fd = shm_open(name.c_str(), O_RDONLY, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_read_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }
    shared_memories[name] = fd;
}

void SharedMemory::open_write_shm(const std::string& name)
{
    int fd = shm_open(name.c_str(), O_WRONLY, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_write_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }
    shared_memories[name] = fd;
}

void SharedMemory::open_readwrite_shm(const std::string& name)
{
    int fd = shm_open(name.c_str(), O_RDWR, 0666);
    if (fd == -1)
    {
        logger->print_error("[open_readwrite_shm] Error opening shared memory", errno);
        throw std::runtime_error("Error allocating shared memory");
    }
    shared_memories[name] = fd;
}

void SharedMemory::open_readwrite_sem(const std::string& name)
{
    sem_t *sem = sem_open(name.c_str(), O_RDWR, 0666, 0);
    if (sem == SEM_FAILED)
    {
        logger->print_error("[open_readwrite_sem] Error opening semaphore", errno);
        throw std::runtime_error("Error allocating semaphore");
    }
    semaphores[name] = sem;
}