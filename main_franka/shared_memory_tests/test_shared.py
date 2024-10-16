from multiprocessing import shared_memory
import numpy as np
import time

def create_shared_memory(name, size):
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=name)
    return shm

# Shared Memory für Lesen (250 double-Werte)
shm_read_name = "my_custom_shm_read"
shm_read = create_shared_memory(shm_read_name, 250 * 8)  # 8 bytes pro double
data_read = np.ndarray((250,), dtype=np.float64, buffer=shm_read.buf)

# Shared Memory für Schreiben (100 double-Werte)
shm_write_name = "my_custom_shm_write"
shm_write = create_shared_memory(shm_write_name, 100 * 8)  # 8 bytes pro double
data_write = np.ndarray((100,), dtype=np.float64, buffer=shm_write.buf)

shm_data_from_python_valid_name   = "data_from_python_valid"
shm_data_from_python_name         = "data_from_python"
shm_data_from_simulink_valid_name = "data_from_simulink_valid"
shm_data_from_simulink_name       = "data_from_simulink"

shm_read = create_shared_memory(shm_data_from_python_valid_name, 1000 * 8)  # 8 bytes pro double
shm_read = create_shared_memory(shm_data_from_python_name, 1)  # 1 bytes (bit possible?)
shm_read = create_shared_memory(shm_data_from_simulink_valid_name, 3 * 8 * 8)  # 8 bytes pro double
shm_read = create_shared_memory(shm_data_from_simulink_name, 1)  # 1 bytes pro double

try:
    while True:
        # Daten aus dem ersten Shared Memory lesen
        data_read[:] = np.random.rand(250)
        print("Gelesene Daten:", data_read[:5], "...")  # Zeige die ersten 5 Werte

        # Daten in den zweiten Shared Memory schreiben
        print("Geschriebene Daten:", data_write[:5], "...")  # Zeige die ersten 5 Werte

        time.sleep(1)
except KeyboardInterrupt:
    print("\nProgramm durch Benutzer beendet.")
finally:
    shm_read.close()
    shm_read.unlink()
    shm_write.close()
    shm_write.unlink()
    print("Shared Memory freigegeben.")