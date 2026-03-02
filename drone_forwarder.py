from cProfile import label
import socket
import threading
import time
import sys
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

DRONE_IP = "10.5.0.2" # Drone's IP address on ad-hoc wfb-ng network
LAPTOP_IP = "10.55.0.2" #Laptop's IP address on ethernet-over-usb connection
VIDEO_PORT = "5602" # Port for video stream (UDP)
DATA_PORT = "5601" # Port for telemetry data (UDP)
COMMAND_PORT = "5603" # Port for command/control (TCP)

UDP_BUFFER_SIZE = 65507 # Maximum safe UDP payload size (65,535 - 8 byte header - 20 byte IP header)s

running = True # Global flag for thread control, set to False on interrupts

def forward_udp(listen_port, dest_ip, dest_port, name):
    """
    Forward UDP packets from specified port to designated IPv4 address/port
    
    :param listen_port: output port
    :param dest_ip: IPv4 destination address
    :param dest_port: destination port
    :param name: Data transmission operation (i.e. 'VIDEO' or 'DATA')
    
    """
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow for address reuse, avoiding restart errors 
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024) # 4MiB Buffer
    rx.bind(('0.0.0.0', listen_port)) #bind socket to all interfaces, externally
    
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # Transmitting socket
    
    logging.info(f'{name} UDP: {listen_port} -> {dest_ip}:{dest_port}') 
    
    packets = 0 # Packet count for logging
    
    while running:
        try:
            data, addr = rx.recvfrom(UDP_BUFFER_SIZE) # Read UDP packet
            tx.sendto(data, (dest_ip, dest_port)) # Forward to destination
            packets += 1 # Increment packet count
            if packets % 500 == 0: # Log every 500 packets
                logging.info(f'{name} forwarded {packets} packets')
        except socket.timeout:
            logging.warning('socket timeout')
            continue
        except Exception as e:
            if running:
                logging.error(f'{name} error: {e}')
                time.sleep(0.1)
    
    rx.close() 
    tx.close()
    logging.info(f'{name} stopped. Forwarded {packets} packets')
    
def handle_tcp_connection(conn, addr, drone_ip, drone_port, name):
    """Handles a single TCP connection from the laptop, relays to drone.
    
    :param conn: TCP connection socket
    :param addr: Address of the laptop connection
    :param drone_ip: IPv4 address of the drone
    :param drone_port: TCP port on the drone to forward to
    :param name: Data transmission operation (i.e. 'VIDEO' or 'DATA')

    """
    try:
        drone_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP socket to connect to drone
        drone_sock.settimeout(5.0) # Set timeout for connection attempts
        drone_sock.connect((drone_ip, drone_port)) # Connect to drone

        def relay(src, dst, label): # Relay data from src to dst, with logging
            try:
                while True:
                    data = src.recv(4096) # Read data from source (4KiB buffer)
                    if not data:
                        break
                    dst.sendall(data) # Send all data to destination
            except Exception:
                pass
            finally:
                src.close()
                dst.close()

        # Relay in both directions
        tcp_to_drone = threading.Thread(target=relay, args=(conn, drone_sock, f"{name} <-"), daemon=True) # Laptop to drone
        tcp_to_ground = threading.Thread(target=relay, args=(drone_sock, conn, f"{name} ->"), daemon=True) # Drone to laptop
        
        # Start both relay threads
        tcp_to_drone.start() 
        tcp_to_ground.start()
        tcp_to_drone.join()
        tcp_to_ground.join()

    except Exception as e:
        logging.error(f'{name} TCP connection error: {e}')
    finally:
        try: conn.close()
        except: pass

    
def forward_tcp(listen_port, dest_ip, dest_port, name):
    """
    Forward TCP packets from specified port to destination IPv4 address/port
    
    :param listen_port: output port
    :param dest_ip: IPv4 destination address
    :param dest_port: destination port
    :param name: Data transmission operation (i.e. 'VIDEO' or 'DATA')

    """    
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP socket for listening to laptop connections
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Allow for address reuse, avoiding restart errors
    server.bind(('0.0.0.0', listen_port)) 
    server.listen(5)
    server.settimeout(1.0)
    
    logging.info(f'{name} TCP: {listen_port} -> {dest_ip}:{dest_port}')
    
    while running:
        try:
            conn, addr = server.accept() # Wait for a laptop to connect
            tcp_handler_thread = threading.Thread(target=handle_tcp_connection, 
            args=(conn, addr, dest_ip, dest_port, name), daemon=True) # Handle connection in separate thread
            tcp_handler_thread.start() 
        except socket.timeout:
            logging.warning('socket timeout')
            continue
        except Exception as e:
            logging.error(f'{name} error: {e}')
            
    server.close()
    
def main():
    """
    Entry point for drone forwarder.
    Starts threads for forwarding VIDEO and DATA over UDP, and COMMAND over TCP.
    
    """
    
    global running
    
    print("Starting drone forwarder...")
    print(f"Forwarding VIDEO UDP: {VIDEO_PORT} -> {LAPTOP_IP}:{VIDEO_PORT}")
    print(f"Forwarding DATA UDP: {DATA_PORT} -> {LAPTOP_IP}:{DATA_PORT}")
    print(f"Forwarding COMMAND TCP: {COMMAND_PORT} -> {LAPTOP_IP}:{COMMAND_PORT}")
    
    video_thread = threading.Thread(target=forward_udp, args=(int(VIDEO_PORT), LAPTOP_IP, int(VIDEO_PORT), 'VIDEO'), daemon=True)
    data_thread = threading.Thread(target=forward_udp, args=(int(DATA_PORT), LAPTOP_IP, int(DATA_PORT), 'DATA'), daemon=True)
    command_thread = threading.Thread(target=forward_tcp, args=(int(COMMAND_PORT), LAPTOP_IP, int(COMMAND_PORT), 'COMMAND'), daemon=True)
    
    threads = [video_thread, data_thread, command_thread]
    
    for t in threads:
        t.start() # Start all forwarding threads
    
    logging.info('Drone forwarder running. Press Ctrl+C to stop.')
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info('Stopping drone forwarder...')
        running = False
        for t in threads:
            t.join(timeout=2)
        logging.info('Drone forwarder stopped.')
        sys.exit(0)

if __name__ == "__main__":
    main()