from data_providers.serial.data_readers import DataSensor
import typing as t
import sqlite3
import os.path
from datetime import datetime
from time import sleep

DATABASE = 'jocasta_readings.db'


def fetch_readings() -> t.Dict:
    sensor = DataSensor()
    return sensor.read()


def setup_db():
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()

    # Create table
    c.execute('''CREATE TABLE readings
                 (date text, humidity int, light float, temperature real)''')
    return conn


def update_db(readings):

    humidity = readings['humidity']
    light = readings['light']
    temperature = readings['temperature']

    if not os.path.isfile(DATABASE):
        conn = setup_db()
    else:
        conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    c.execute(f"INSERT INTO readings VALUES ('{datetime.now()}',{humidity},"
              f"{light},"
              f"{temperature})")
    conn.commit()
    conn.close()


def main():
    while True:
        readings = fetch_readings()
        update_db(readings)
        print(readings)
        sleep(60)


if __name__ == "__main__":
    # execute only if run as a script
    main()
