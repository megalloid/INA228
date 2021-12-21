import api
from datetime import datetime

ina228 = api.INA228()

ina228.configure()

i = 0

while True:    

    ina228.get_vbus_voltage()

    ina228.get_current()

    ina228.get_power()

    if i < 1000:

        i = i +1
        print(i, datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

    else:

        exit()
