import socket
import threading
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import queue
import serial  # <-- NEW

ESP32_IP = '10.78.174.50'
PORT = 5000
WIDTH = 240
HEIGHT = 240
BYTES_PER_FRAME = WIDTH * HEIGHT * 2

SERIAL_PORT = '/dev/ttyUSB0'  # <-- Change this to your actual port, e.g., '/dev/ttyUSB0'
BAUD_RATE = 921600

def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            raise ConnectionError("Socket closed")
        data += packet
    return data

class VideoClientGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ESP32 Video Client")

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Serial connected on {SERIAL_PORT}")
        except serial.SerialException as e:
            self.serial_conn = None
            print("Serial connection failed:", e)

        # Top frame
        top_frame = tk.Frame(master)
        top_frame.pack(pady=5)

        self.connect_btn = tk.Button(top_frame, text="Connect", command=self.connect)
        self.connect_btn.pack()

        self.canvas = tk.Label(top_frame)
        self.canvas.pack()

        # Button frame
        self.button_frame = tk.Frame(master)
        self.button_frame.pack(pady=10)

        self.up_btn = tk.Button(self.button_frame, text="Up", width=10, command=lambda: self.send_command("UP"))
        self.up_btn.grid(row=0, column=1)

        self.left_btn = tk.Button(self.button_frame, text="Left", width=10, command=lambda: self.send_command("LEFT"))
        self.left_btn.grid(row=1, column=0)

        self.down_btn = tk.Button(self.button_frame, text="Down", width=10, command=lambda: self.send_command("DOWN"))
        self.down_btn.grid(row=1, column=1)

        self.right_btn = tk.Button(self.button_frame, text="Right", width=10, command=lambda: self.send_command("RIGHT"))
        self.right_btn.grid(row=1, column=2)

        self.running = False
        self.sock = None
        self.frame_queue = queue.Queue(maxsize=1)

    def send_command(self, direction):
        print(f"Sending command: {direction}")
        # Send via Serial
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write((direction + '\n').encode())
                if (direction == "UP"): 
                    self.serial_conn.write("servo move up 1\n".encode())
                elif (direction == "DOWN"): 
                    self.serial_conn.write("servo move down 1\n".encode())
                elif (direction == "RIGHT"): 
                    self.serial_conn.write("servo move right 1\n".encode())
                elif (direction == "LEFT"): 
                    self.serial_conn.write("servo move left 1\n".encode())
                else:
                    print("invalid direction")
            except Exception as e:
                print("Serial send failed:", e)

    def connect(self):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.network_thread, daemon=True).start()
        self.master.after(10, self.display_loop)

    def bgr565_to_rgb888(self, frame):
        b = ((frame & 0xF800) >> 11).astype(np.uint8)
        g = ((frame & 0x07E0) >> 5).astype(np.uint8)
        r = (frame & 0x001F).astype(np.uint8)
        r = (r << 3) | (r >> 2)
        g = (g << 2) | (g >> 4)
        b = (b << 3) | (b >> 2)
        rgb = np.dstack((r, g, b))
        return rgb

    def network_thread(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                print("Connecting...")
                s.bind(("", PORT))
                self.sock = s
                status, addr = s.recvfrom(32)
                print(addr, status.decode())
                s.sendto(b"OK", (ESP32_IP, PORT))

                while self.running:
                    magic = recv_exact(s, 4)
                    if magic != b'FRAM':
                        continue
                    frame_len = int.from_bytes(recv_exact(s, 4), 'little')
                    if frame_len != BYTES_PER_FRAME:
                        continue

                    frame = bytearray()
                    for _ in range(225):
                        data = recv_exact(s, 528)
                        frame.extend(data[16:])

                    if self.frame_queue.full():
                        self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(frame)
        except Exception as e:
            print("Network error:", e)
        finally:
            self.running = False

    def display_loop(self):
        if not self.running:
            return
        try:
            data = self.frame_queue.get_nowait()
            frame = np.frombuffer(data, dtype='<u2').reshape((HEIGHT, WIDTH))
            frame = frame.byteswap()
            rgb = self.bgr565_to_rgb888(frame)
            img = Image.fromarray(rgb, 'RGB')
            imgtk = ImageTk.PhotoImage(image=img)
            self.canvas.imgtk = imgtk
            self.canvas.configure(image=imgtk)
        except queue.Empty:
            pass
        self.master.after(1, self.display_loop)

    def on_close(self):
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = VideoClientGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
