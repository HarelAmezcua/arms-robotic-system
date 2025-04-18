from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
from pySerialTransfer.pySerialTransfer import Status


class Struct:
    z = 0
    q1 = 0.0
    q2 = 0.0
    q3 = 0.0
    q4 = 0.0
    q5 = 0.0


if __name__ == '__main__':
    try:
        testStruct = Struct
        link = txfer.SerialTransfer('/dev/ttyACM0')
        
        link.open()
        sleep(5)

        print('Sending data...')
    
        while True:
            if link.available():
                print('Received data')                
                recSize = 0
                
                testStruct.z = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
                
                testStruct.q1 = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                testStruct.q2 = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                testStruct.q3 = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                testStruct.q4 = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

                testStruct.q5 = link.rx_obj(obj_type='f', start_pos=recSize)
                recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
                
                
                print('{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}'.format(testStruct.z, testStruct.q1, testStruct.q2, testStruct.q3, testStruct.q4, testStruct.q5))
                
            elif link.status.value <= 0:
                if link.status == Status.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == Status.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == Status.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status.name))
                        
    except KeyboardInterrupt:
        link.close()