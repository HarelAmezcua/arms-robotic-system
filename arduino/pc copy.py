from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
from pySerialTransfer.pySerialTransfer import Status

class Struct:
    def __init__(self):
        self.z = 0.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.q4 = 0.0
        self.q5 = 0.0
        self.q6 = 0.0

class DataStreamer:
    def __init__(self, port):
        self.link = txfer.SerialTransfer(port)
        self.testStruct = Struct()

    def open_connection(self):
        self.link.open()
        sleep(5)  # Allow time for the connection to stabilize
        print("Connection opened.")

    def close_connection(self):
        self.link.close()
        print("Connection closed.")

    def send_data(self):
        sendSize = 0
        sendSize = self.link.tx_obj(self.testStruct.z, start_pos=sendSize)
        sendSize = self.link.tx_obj(self.testStruct.q1, start_pos=sendSize)
        sendSize = self.link.tx_obj(self.testStruct.q2, start_pos=sendSize)
        sendSize = self.link.tx_obj(self.testStruct.q3, start_pos=sendSize)
        sendSize = self.link.tx_obj(self.testStruct.q4, start_pos=sendSize)
        sendSize = self.link.tx_obj(self.testStruct.q5, start_pos=sendSize)

        self.link.send(sendSize)
        print("Data sent.")

    def receive_data(self):
        if self.link.available():
            print("Received data")
            recSize = 0

            self.testStruct.z = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            self.testStruct.q1 = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            self.testStruct.q2 = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            self.testStruct.q3 = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            self.testStruct.q4 = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            self.testStruct.q5 = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            print('{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}'.format(
                self.testStruct.z, self.testStruct.q1, self.testStruct.q2,
                self.testStruct.q3, self.testStruct.q4, self.testStruct.q5))
        elif self.link.status.value <= 0:
            self.handle_error()

    def handle_error(self):
        if self.link.status == Status.CRC_ERROR:
            print("ERROR: CRC_ERROR")
        elif self.link.status == Status.PAYLOAD_ERROR:
            print("ERROR: PAYLOAD_ERROR")
        elif self.link.status == Status.STOP_BYTE_ERROR:
            print("ERROR: STOP_BYTE_ERROR")
        else:
            print(f"ERROR: {self.link.status.name}")

    def update_struct(self,array=None):
        self.testStruct.z =  array[0]
        self.testStruct.q1 = array[1]
        self.testStruct.q2 = array[2]
        self.testStruct.q3 = array[3]
        self.testStruct.q4 = array[4]
        self.testStruct.q5 = array[5]
        self.testStruct.q6 = array[6]        

if __name__ == '__main__':
    try:
        streamer = DataStreamer('/dev/ttyACM0')
        streamer.open_connection()

        while True:
            streamer.update_struct()
            streamer.send_data()
            sleep(1.0)
            streamer.receive_data()

    except KeyboardInterrupt:
        streamer.close_connection()