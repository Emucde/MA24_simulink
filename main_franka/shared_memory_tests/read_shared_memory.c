#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define SHM_NAME "/psm_29a56b57"
#define SHM_SIZE 2000  // Größe in Bytes, anpassen nach Bedarf

int main() {
    int fd;
    double *shared_data;

    // Öffnen des Shared Memory
    fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (fd == -1) {
        perror("shm_open");
        exit(1);
    }

    // Memory mappen
    shared_data = (double*)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, fd, 0);
    if (shared_data == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }

    // Daten lesen und ausgeben
    for (int i = 0; i < 250; i++) {  // Annahme: 250 double-Werte
        printf("Data[%d] = %f\n", i, shared_data[i]);
    }

    // Aufräumen
    munmap(shared_data, SHM_SIZE);
    close(fd);

    return 0;
}
