# pip install pyserial

import serial
import time

# Konfiguracja portu szeregowego
ser = serial.Serial(
    port='/dev/ttyUSB0',  # Ustaw port zgodnie z konfiguracją twojego systemu
    baudrate=9600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def send_command(command):
    """
    Wysyła komendę do modułu przez port szeregowy.
    """
    ser.write(command.encode('ascii'))
    time.sleep(0.1)  # Małe opóźnienie dla stabilności komunikacji
    response = ser.read_all().decode('ascii')
    return response

def set_output_state(address, state):
    """
    Ustawia stan wyjścia modułu.
    """
    command = f'P{address:02d}={state}<CR>'
    return send_command(command)

def set_pwm(address, value, delay):
    """
    Ustawia współczynnik PWM i opóźnienie.
    """
    command = f'P{address:02d}={value},{delay}<CR>'
    return send_command(command)

def get_output_state(address):
    """
    Pobiera stan wyjścia modułu.
    """
    command = f'P{address:02d}=?<CR>'
    return send_command(command)

# Przykładowe użycie
if __name__ == "__main__":
    module_address = 24

    # Włączenie wyjścia
    response = set_output_state(module_address, 1)
    print(f'Włączenie wyjścia: {response}')

    # Ustawienie współczynnika PWM na 128 (50%) bez opóźnienia
    response = set_pwm(module_address, 128, 0)
    print(f'Ustawienie PWM: {response}')

    # Pobranie stanu wyjścia
    response = get_output_state(module_address)
    print(f'Stan wyjścia: {response}')

    # Wyłączenie wyjścia
    response = set_output_state(module_address, 0)
    print(f'Wyłączenie wyjścia: {response}')

# Zamknięcie portu szeregowego
ser.close()
