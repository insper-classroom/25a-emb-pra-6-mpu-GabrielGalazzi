import sys
import glob
import serial
import pyautogui
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import time

pyautogui.FAILSAFE = False

def move_mouse(dx, dy):
    """Move the mouse relative to current position."""
    pyautogui.moveRel(dx, dy, duration=0.01)

def controle(ser):
    """
    Main loop that reads serial data and controls the mouse.
    Waits for sync byte (0xFF) and then reads 4 bytes (roll, pitch).
    """
    last_click_time = time.time()
    click_cooldown = 0.5  
    deadzone = 0.5        
    sensitivity = 2       

    while True:
        sync_byte = ser.read(size=1)
        if not sync_byte:
            continue

        if sync_byte[0] == 0xFE:
            current_time = time.time()
            if current_time - last_click_time > click_cooldown:
                pyautogui.click()
                print("Click triggered")
                last_click_time = current_time

        elif sync_byte[0] == 0xFF:
            data = ser.read(size=4)
            if len(data) != 4:
                continue  # Skip incomplete packets

            roll = int.from_bytes(data[0:2], byteorder="little", signed=True) / 100.0
            pitch = int.from_bytes(data[2:4], byteorder="little", signed=True) / 100.0

            dx = round(roll * sensitivity, 2)
            dy = round(pitch * sensitivity, 2)

            if abs(roll) < deadzone:
                dx = 0
            if abs(pitch) < deadzone:
                dy = 0

            if dx != 0 or dy != 0:
                move_mouse(dx, dy)

def serial_ports():
    """Lists available serial ports."""
    if sys.platform.startswith('win'):
        ports = [f'COM{i}' for i in range(1, 256)]
    elif sys.platform.startswith('linux'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    available_ports = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            available_ports.append(port)
        except (OSError, serial.SerialException):
            pass
    return available_ports

def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    """Connects to the selected serial port and starts reading data."""
    if not port_name:
        messagebox.showwarning("Warning", "Select a serial port first.")
        return

    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        status_label.config(text=f"Connected to {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(text="Connected")
        controle(ser)  # Blocks here until KeyboardInterrupt

    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to connect: {e}")
        mudar_cor_circulo("red")
    finally:
        if 'ser' in locals():
            ser.close()
        status_label.config(text="Disconnected", foreground="red")
        mudar_cor_circulo("red")

def criar_janela():
    """Creates the GUI window."""
    root = tk.Tk()
    root.title("MPU6050 Mouse Control")
    root.geometry("400x250")
    root.resizable(False, False)

    # Dark theme
    dark_bg = "#2e2e2e"
    dark_fg = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=dark_bg, foreground=dark_fg)
    style.configure("TButton", font=("Segoe UI", 10), background="#444444")
    style.map("TButton", background=[("active", "#555555")])

    # Main frame
    frame_principal = ttk.Frame(root, padding="20")
    frame_principal.pack(expand=True, fill="both")

    # Title
    titulo_label = ttk.Label(frame_principal, text="MPU6050 Mouse Control", font=("Segoe UI", 14, "bold"))
    titulo_label.pack(pady=(0, 10))

    # Connect button
    porta_var = tk.StringVar(value="")
    botao_conectar = ttk.Button(
        frame_principal,
        text="Connect & Start",
        command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)
    )
    botao_conectar.pack(pady=10)

    # Footer (status, port selection, LED)
    footer_frame = tk.Frame(root, bg=dark_bg)
    footer_frame.pack(side="bottom", fill="x", padx=10, pady=(10, 0))

    status_label = tk.Label(footer_frame, text="Select a port...", bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    port_dropdown = ttk.Combobox(footer_frame, textvariable=porta_var, values=serial_ports(), state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle_canvas = tk.Canvas(footer_frame, width=20, height=20, bg=dark_bg, highlightthickness=0)
    circle_item = circle_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    circle_canvas.grid(row=0, column=2, sticky="e")

    def mudar_cor_circulo(color):
        circle_canvas.itemconfig(circle_item, fill=color)

    root.mainloop()

if __name__ == "__main__":
    criar_janela()