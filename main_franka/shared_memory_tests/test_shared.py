from multiprocessing import shared_memory
import numpy as np
import time

def create_shared_memory(name, size):
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=name)
    return shm

shm_data_from_python_name         = "data_from_python"
shm_data_from_python_valid_name   = "data_from_python_valid"

shm_data_from_simulink_name       = "data_from_simulink"
shm_data_from_simulink_valid_name = "data_from_simulink_valid"

python_buffer_bytes = 250 * 8 # 8 bytes pro double
python_flag_bytes = 1 * 8 # 1 byte
simulink_buffer_bytes = 2 * 7 * 8 # 8 bytes pro double
simulink_flag_bytes = 1 * 8# 1 byte

shm_data_from_python = create_shared_memory(shm_data_from_python_name, python_buffer_bytes)  # 8 bytes pro double
shm_data_from_python_valid = create_shared_memory(shm_data_from_python_valid_name, python_flag_bytes)  # 1 bytes (bit possible?)
shm_data_from_simulink = create_shared_memory(shm_data_from_simulink_name, simulink_buffer_bytes)  # 8 bytes pro double
shm_data_from_simulink_valid = create_shared_memory(shm_data_from_simulink_valid_name, simulink_flag_bytes)  # 1 bytes pro double

data_from_python = np.ndarray((python_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_python.buf)
data_from_python_valid = np.ndarray((python_flag_bytes//8,), dtype=np.float64, buffer=shm_data_from_python_valid.buf)

data_from_simulink = np.ndarray((simulink_buffer_bytes//8,), dtype=np.float64, buffer=shm_data_from_simulink.buf)
data_from_simulink_valid = np.ndarray((simulink_flag_bytes//8,), dtype=np.float64, buffer=shm_data_from_simulink_valid.buf)



try:
    while True:
        # Daten für Simulink schreiben
        # time.sleep(3e-3)
        if(data_from_python_valid[:] == 0):
            data_from_python_valid[:] = 1
            data_from_python[:] = np.random.rand(250)
            print("Daten von Python:", data_from_python[:5], "...")  # Zeige die ersten 5 Werte
        
        # Daten von Simulink lesen
        if(data_from_simulink_valid[:] == 1):
            print("Daten von Simulink:", data_from_simulink[:])  # Zeige die ersten 5 Werte
            data_from_simulink_valid[:] = 0
            data_from_python_valid[:] = 0
        # time.sleep(1e-9)
except KeyboardInterrupt:
    print("\nProgramm durch Benutzer beendet.")
finally:
    shm_data_from_python.close()
    shm_data_from_python.unlink()
    shm_data_from_python_valid.close()
    shm_data_from_python_valid.unlink()
    shm_data_from_simulink.close()
    shm_data_from_simulink.unlink()
    shm_data_from_simulink_valid.close()
    shm_data_from_simulink_valid.unlink()
    print("Shared Memory freigegeben.")