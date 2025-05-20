import serial
import time
import sys
import argparse

def main():
    parser = argparse.ArgumentParser(description='ESP32-CAM Serial Monitor')
    parser.add_argument('--port', default='/dev/tty.usbmodem11101', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    
    # Open serial port with DTR and RTS disabled (like Arduino)
    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False
    )
    
    # Explicitly disable DTR and RTS
    ser.dtr = False
    ser.rts = False
    
    print("Connected! Press Ctrl+C to exit.")
    
    try:
        while True:
            # Read data if available
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
            
            # Check for input to send
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                ser.write(line.encode())
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nClosing connection")
        ser.close()
    except Exception as e:
        print(f"Error: {e}")
        ser.close()

if __name__ == "__main__":
    import select
    main()