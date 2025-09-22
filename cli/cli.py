import socket
import threading

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('192.168.0.10', 3000))
sock.listen(5)


def client_thread(client, address):
    print(f"Connection from {address} has been established")
    client.send(open("splash").read().encode())
    while True:
        data = client.recv(1024)
        if not data:
            break
        print(f"Received from {address}: {data.decode()}")
        client.send(data)  # Echo back the received data
    client.close()
    print(f"Connection from {address} has been closed.")

if __name__ == "__main__":
    while True:
        client, address = sock.accept()
        threading.Thread(target=client_thread, args=(client, address)).start()
