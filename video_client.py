import socket
import time
import threading
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import queue
import serial  # <-- NEW

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
        self.frame_queue = queue.Queue(maxsize=2)

    def send_command(self, direction):
        print(f"Sending command: {direction}")
        # Send via Serial
        if self.serial_conn and self.serial_conn.is_open:
            try:
                if (direction == "UP"): 
                    self.serial_conn.write(b"U1")
                elif (direction == "DOWN"): 
                    self.serial_conn.write(b"D1")
                elif (direction == "RIGHT"): 
                    self.serial_conn.write(b"R1")
                elif (direction == "LEFT"): 
                    self.serial_conn.write(b"L1")
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
            frame = bytearray(115200)
            old_id = -1  # start with an invalid frame_id
            while self.running:
                data = self.serial_conn.read_until(expected=b'F0000', size=528)  # or read_until if there's a reliable marker

                if len(data) != 528:
                    continue  # ignore incomplete chunks

                try:
                    frame_info = data[:11].decode("utf-8")  # Assuming header is 16 bytes
                    frame_chunk = int(frame_info[-3:])
                    frame_id = int(frame_info[:-3])
                except Exception as e:
                    print(f"Parse error: {e}")
                    continue

                if frame_id != old_id:
                    # New frame: send last one to display
                    if old_id != -1 and not self.frame_queue.full():
                        self.frame_queue.put_nowait(frame)
                        print("Frame", old_id)
                    frame = bytearray(115200)
                    old_id = frame_id

                idx = frame_chunk * 512
                if idx + 512 <= len(frame):
                    frame[idx:idx + 512] = data[12:-4]
                else:
                    print("Warning: Chunk index out of range")
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
            # frame = frame.byteswap()
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
