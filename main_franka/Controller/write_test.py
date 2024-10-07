import numpy as np
import time

def write_float_data(file_name, value):
    # Konvertiere den Float-Wert in ein NumPy-Array und schreibe es als Bytes
    float_array = np.array([value], dtype=np.float64)  # dtype für double precision
    with open(file_name, 'wb') as file:  # 'wb' für Schreiben im Binärmodus
        file.write(float_array.tobytes())
    print(f"Float-Zahl {value} wurde in die Datei '{file_name}' geschrieben.")

def main():
    filename = "data_from_crocoddyl.bin"
    
    # Startwert für die Float-Zahl
    float_value = 0.0
    
    # Initialisiere den Startzeitpunkt
    start_time = time.perf_counter()

    try:
        while True:
            # Berechne die aktuelle Zeit seit dem Start
            current_time = time.perf_counter()
            elapsed_time = current_time - start_time
            
            # Wenn 1 ms vergangen ist
            if elapsed_time >= 0.001:
                # Schreiben der Float-Zahl in die Datei
                write_float_data(filename, float_value)

                # Erhöhen des Wertes um 1 für den nächsten Durchlauf
                float_value += 1.0

                # Setze den Startzeitpunkt zurück
                start_time += 0.001  # Erhöhe den Startzeitpunkt um 1 ms
            
            # Optional: Reduziere die CPU-Auslastung durch eine kurze Pause
            time.sleep(0.000001)  # Warte kurz, um die CPU nicht zu überlasten

    except KeyboardInterrupt:
        print("Programm beendet.")

if __name__ == "__main__":
    main()
