import serial.tools.list_ports


def find_device(manufacturer=''):
    '''
    Return the device name of the first serial port 
    connected to a specified device.

    Returns: string
    '''
    ports = list(serial.tools.list_ports.comports())
    try:
        for port in ports:
            if manufacturer in port.manufacturer:
                return port.device
    except:
        print 'No matching serial device found.'
        return ''

if __name__ == '__main__':
    print 'arduino:', find_device('Arduino')
    print 'lock-in:', find_device('Prolific')
