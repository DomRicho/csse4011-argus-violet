import socket
import threading
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import numpy as np

ESP32_IP = '192.168.1.50'
PORT = 5000
WIDTH = 240   # Set to your ESP32's frame width
HEIGHT = 240  # Set to your ESP32's frame height
BYTES_PER_FRAME = WIDTH * HEIGHT * 2  # 2 bytes per pixel for RGB565

class VideoClientGUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ESP32 Video Client")
        # Set window size to fit the frame plus some space for the button
        self.master.geometry(f"{WIDTH+20}x{HEIGHT+60}")
        self.connect_btn = tk.Button(master, text="Connect", command=self.connect)
        self.connect_btn.pack()
        # Set the label size to the frame size
        self.canvas = tk.Label(master, width=WIDTH, height=HEIGHT)
        self.canvas.pack()
        self.running = False
        self.sock = None

    def connect(self):
        if self.running:
            return
        self.running = True
        threading.Thread(target=self.video_loop, daemon=True).start()

    def rgb565_to_rgb888(self, frame):
        r = ((frame >> 11) & 0x1F) * 255 // 31
        g = ((frame >> 5) & 0x3F) * 255 // 63
        b = (frame & 0x1F) * 255 // 31
        rgb = np.stack((r, g, b), axis=-1).astype(np.uint8)
        return rgb

    def video_loop(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((ESP32_IP, PORT))
                self.sock = s
                while self.running:
                    data = b''
                    while len(data) < BYTES_PER_FRAME:
                        packet = s.recv(BYTES_PER_FRAME - len(data))
                        if not packet:
                            self.running = False
                            return
                        data += packet

                    frame = np.frombuffer(data, dtype='<u2').reshape((HEIGHT, WIDTH))
                    rgb = self.rgb565_to_rgb888(frame)
                    img = Image.fromarray(rgb, 'RGB')
                    imgtk = ImageTk.PhotoImage(image=img)
                    self.canvas.imgtk = imgtk
                    self.canvas.configure(image=imgtk)
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
        finally:
            self.running = False

    def on_close(self):
        self.running = False
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = VideoClientGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()