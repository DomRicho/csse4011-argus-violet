import socket
import threading
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import queue

ESP32_IP = '192.168.1.50'
PORT = 5000
WIDTH = 240
HEIGHT = 240
BYTES_PER_FRAME = WIDTH * HEIGHT * 2

def recv_exact(sock, n):
    """Receive exactly n bytes from the socket."""
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
        self.master.geometry(f"{WIDTH+20}x{HEIGHT+60}")
        self.connect_btn = tk.Button(master, text="Connect", command=self.connect)
        self.connect_btn.pack()
        self.canvas = tk.Label(master, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.running = False
        self.sock = None
        self.frame_queue = queue.Queue(maxsize=1)  # Only keep the latest frame
        self.printed_first_frame = False
        self.printed_first_reshape = False
        self.printed_first_rgb = False

    def connect(self):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.network_thread, daemon=True).start()
        self.master.after(10, self.display_loop)

    def rgb565_to_rgb888(self, frame):
        # frame: 2D np.array of dtype uint16, shape (HEIGHT, WIDTH)
        r = ((frame & 0xF800) >> 11).astype(np.uint8)
        g = ((frame & 0x07E0) >> 5).astype(np.uint8)
        b = (frame & 0x001F).astype(np.uint8)
        # Scale to 8 bits
        r = (r << 3) | (r >> 2)
        g = (g << 2) | (g >> 4)
        b = (b << 3) | (b >> 2)
        rgb = np.dstack((r, g, b))
        return rgb

    def network_thread(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((ESP32_IP, PORT))
                self.sock = s
                while self.running:
                    # Read magic header
                    magic = recv_exact(s, 4)
                    if magic != b'FRAM':
                        print("Frame sync lost, searching for next header...")
                        continue  # Or implement resync logic

                    # Read frame length
                    frame_len_bytes = recv_exact(s, 4)
                    frame_len = int.from_bytes(frame_len_bytes, 'little')
                    if frame_len != BYTES_PER_FRAME:
                        print(f"Unexpected frame size: {frame_len}")
                        continue

                    # Read frame data
                    data = recv_exact(s, frame_len)
                    if not self.printed_first_frame:
                        self.printed_first_frame = True
                        print("Client first 16 bytes:", ' '.join(f"{b:02X}" for b in data[:16]))
                        print("Client last 16 bytes:", ' '.join(f"{b:02X}" for b in data[-16:]))
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.frame_queue.put_nowait(data)
        except Exception as e:
            print("Connection Error:", e)
        finally:
            self.running = False

    def display_loop(self):
        if not self.running:
            return
        try:
            data = self.frame_queue.get_nowait()
            frame = np.frombuffer(data, dtype='<u2').reshape((HEIGHT, WIDTH))
            if not self.printed_first_reshape:
                self.printed_first_reshape = True
                print("After reshape, shape:", frame.shape)
                flat = frame.flatten()
                print("After reshape, first 8 uint16:", ' '.join(f"{v:04X}" for v in flat[:8]))
                print("After reshape, last 8 uint16:", ' '.join(f"{v:04X}" for v in flat[-8:]))
            rgb = self.rgb565_to_rgb888(frame)
            if not self.printed_first_rgb:
                self.printed_first_rgb = True
                print("After RGB conversion, shape:", rgb.shape)
                print("After RGB conversion, first 2 pixels:", rgb[0,0], rgb[0,1])
                print("After RGB conversion, last 2 pixels:", rgb[-1,-2], rgb[-1,-1])
            img = Image.fromarray(rgb, 'RGB')
            imgtk = ImageTk.PhotoImage(image=img)
            self.canvas.imgtk = imgtk
            self.canvas.configure(image=imgtk)
        except queue.Empty:
            pass
        self.master.after(1, self.display_loop)

    def on_close(self):
        self.running = False
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = VideoClientGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()